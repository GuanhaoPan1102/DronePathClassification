#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_timer.h" // 引入高精度硬體計時器
#include "esp_log.h"
#include "drv_4g_gnss_sync.h"

static const char *TAG = "4G_GNSS_SYNC";

// ==========================================
// 全域狀態與事件標誌
// ==========================================
static EventGroupHandle_t g_at_event_group;
#define AT_EVENT_OK    BIT0
#define AT_EVENT_ERROR BIT1

#define SURVEY_SAMPLES 60   
static int _rmc_count = 0; 
static int _gga_count = 0; 
static int64_t _sum_lat = 0;
static int64_t _sum_lon = 0;
static float   _sum_alt = 0.0f;
static gps_fix_t _final_fix = {0};

// ==========================================
// 核心魔法：寂靜模式與硬體時間補償變數
// ==========================================
static volatile bool _is_in_silent_mode = false;        // 是否進入寂靜模式
static volatile bool _pps_triggered = false;            // 通知 Task 執行時間同步
static volatile time_t _next_utc_sec = 0;               // 下一秒的 UTC 秒數預報
static volatile int64_t _pps_hardware_timestamp_us = 0; // 紀錄 PPS 發生的硬體瞬間
static volatile int64_t _last_pps_hw_us = 0;            // 上一次 PPS 的硬體時間 (用於漏跳交叉比對)

// ==========================================
// 輔助函式
// ==========================================
static int32_t _convert_nmea_to_scaled(float nmea_val) 
{
    int degrees = (int)(nmea_val / 100);             
    float minutes = nmea_val - (degrees * 100);      
    return (int32_t)((degrees + (minutes / 60.0f)) * 10000000);        
}

// ==========================================
// 零死鎖中斷與交叉比對防呆 (ISR)
// ==========================================
static void IRAM_ATTR pps_gpio_isr_handler(void* arg)
{
    // 1. 瞬間拍下硬體微秒快照
    int64_t current_hw_us = esp_timer_get_time();
    _pps_hardware_timestamp_us = current_hw_us;

    // 2. 寂靜模式 (NMEA 關閉) 下的自體計秒與漏跳補償
    if (_is_in_silent_mode && _last_pps_hw_us > 0) {
        int64_t diff_us = current_hw_us - _last_pps_hw_us;
        // 四捨五入計算經過的秒數 (防護 PPS 漏跳)
        int passed_sec = (diff_us + 500000) / 1000000; 
        
        if (passed_sec > 0) {
            _next_utc_sec += passed_sec;
        }
    }

    // 3. 記錄歷史時間並立旗，交給 Task 處理
    _last_pps_hw_us = current_hw_us;
    if (_next_utc_sec > 0) {
        _pps_triggered = true;
    }
}

// ==========================================
// 底層時間推遲處理 (Task Context)
// ==========================================
static void _process_time_sync_safe(void)
{
    if (_pps_triggered) {
        _pps_triggered = false; // 放下旗標

        // 1. 計算這段 Task 被延遲了幾微秒
        int64_t current_hw_time = esp_timer_get_time();
        int64_t processing_delay_us = current_hw_time - _pps_hardware_timestamp_us;
        
        // 2. 補償並寫入系統時間
        struct timeval tv;
        tv.tv_sec = _next_utc_sec;
        tv.tv_usec = processing_delay_us; 
        
        settimeofday(&tv, NULL);
        
        if (!_final_fix.is_time_synced) {
            _final_fix.is_time_synced = true;
            ESP_LOGI(TAG, "System Time Synced! Initial delay compensated: %lld us", processing_delay_us);

            // 獲取當前系統時間
            time_t now;
            struct tm timeinfo;
            char strftime_buf[64];
            time(&now); 
            
            // 加上 8 小時的秒數 (8 * 3600 = 28800) 轉換為 UTC+8
            time_t local_now = now + 28800; 
            
            // 改用 gmtime_r 避免受到系統 TZ 變數 (UTC) 的影響
            gmtime_r(&local_now, &timeinfo); 
            strftime(strftime_buf, sizeof(strftime_buf), "%Y-%m-%d %H:%M:%S", &timeinfo);
            // 標註為 UTC+8
            ESP_LOGI(TAG, "Current System Time: %s (UTC+8)", strftime_buf);
        }
    }
}

// ==========================================
// NMEA 解析引擎
// ==========================================
static void _parse_rmc(char *line)
{
    char *token;
    char *rest = line;
    int field = 0, raw_time = 0, raw_date = 0; 
    int32_t curr_lat = 0, curr_lon = 0;
    bool valid = false;

    while ((token = strsep(&rest, ",")) != NULL) {
        if (field == 1 && *token) raw_time = atoi(token); 
        else if (field == 2 && *token == 'A') valid = true;
        else if (field == 3 && *token && !_final_fix.is_fixed) curr_lat = _convert_nmea_to_scaled(strtof(token, NULL));
        else if (field == 4 && *token == 'S') curr_lat *= -1;
        else if (field == 5 && *token && !_final_fix.is_fixed) curr_lon = _convert_nmea_to_scaled(strtof(token, NULL));
        else if (field == 6 && *token == 'W') curr_lon *= -1;
        else if (field == 9 && *token) raw_date = atoi(token); 
        field++;
    }

    // 更新時間預報 (準備給下一次 PPS 觸發使用)
    if (raw_date > 0 && raw_time > 0) {
        struct tm tm_struct = {0};
        tm_struct.tm_hour = raw_time / 10000;
        tm_struct.tm_min  = (raw_time % 10000) / 100;
        tm_struct.tm_sec  = raw_time % 100;
        tm_struct.tm_mday = raw_date / 10000;
        tm_struct.tm_mon  = ((raw_date % 10000) / 100) - 1; 
        tm_struct.tm_year = (raw_date % 100) + 100;         

        _next_utc_sec = mktime(&tm_struct) + 1; // 預報下一秒
    }

    if (!_final_fix.is_fixed && valid) {
        _sum_lat += curr_lat;
        _sum_lon += curr_lon;
        _rmc_count++;
    }
}

static void _parse_gga(char *line)
{
    char *token;
    char *rest = line;
    int field = 0;
    bool quality_valid = false;
    float current_alt = 0.0f;

    while ((token = strsep(&rest, ",")) != NULL) {
        if (field == 6 && *token != '0') quality_valid = true;
        else if (field == 9 && *token && !_final_fix.is_fixed) current_alt = strtof(token, NULL);
        field++;
    }

    if (quality_valid && !_final_fix.is_fixed) {
        _sum_alt += current_alt;
        _gga_count++;
    }
}

void drv_4g_gnss_process_nmea(const char *nmea_line)
{
    char buf[128];
    strncpy(buf, nmea_line, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    if (strstr(buf, "RMC")) {
        _parse_rmc(buf);
        
        // 檢查 Survey-In
        if (!_final_fix.is_fixed && _rmc_count >= SURVEY_SAMPLES && _gga_count >= SURVEY_SAMPLES) {
            _final_fix.latitude = (int32_t)(_sum_lat / _rmc_count);
            _final_fix.longitude = (int32_t)(_sum_lon / _rmc_count);
            _final_fix.altitude = _sum_alt / _gga_count;
            _final_fix.is_fixed = true;
            ESP_LOGI(TAG, "MASTER Node Position FIXED! Wait for PPS...");
        }
    } else if (strstr(buf, "GGA")) {
        _parse_gga(buf);
    }
}

// ==========================================
// 字元級分流路由器 (Router Task)
// ==========================================
static void task_4g_uart_rx(void *pvParameters)
{
    uint8_t rx_byte;
    char line_buf[UART_4G_BUF_SIZE];
    int line_idx = 0;
    
    for (;;) {
        // 安全地處理背景時間同步
        _process_time_sync_safe();

        // 採用超短超時，確保系統敏捷度
        int len = uart_read_bytes(UART_4G_PORT_NUM, &rx_byte, 1, 10 / portTICK_PERIOD_MS);
        
        if (len > 0) {
            // 攔截不帶換行符的 TCP 特殊字元
            if (rx_byte == '>') {
                xEventGroupSetBits(g_at_event_group, AT_EVENT_OK);
                continue; 
            }

            if (line_idx < sizeof(line_buf) - 1) {
                line_buf[line_idx++] = (char)rx_byte;
            }

            // 行分割處理
            if (rx_byte == '\n') {
                line_buf[line_idx] = '\0';
                
                if (line_buf[0] == '$') {
                    // 只有在非寂靜模式下才解析 NMEA
                    if (!_is_in_silent_mode) {
                        drv_4g_gnss_process_nmea(line_buf);
                    }
                } 
                else {
                    if (strstr(line_buf, "OK") || strstr(line_buf, "+CIPOPEN: SUCCESS") || strstr(line_buf, "+CIPSEND:SUCCESS")) {
                        xEventGroupSetBits(g_at_event_group, AT_EVENT_OK);
                    } else if (strstr(line_buf, "ERROR")) {
                        xEventGroupSetBits(g_at_event_group, AT_EVENT_ERROR);
                    }
                }
                line_idx = 0; // 清空緩衝準備接下一行
            }
        }
    }
    vTaskDelete(NULL);
}

// ==========================================
// 初始化與 API
// ==========================================
esp_err_t drv_4g_init(void)
{
    ESP_LOGI(TAG, "Initializing 4G Module & GNSS Engine (V2 Architecture)...");

    setenv("TZ", "UTC", 1);
    tzset();

    g_at_event_group = xEventGroupCreate();
    if (g_at_event_group == NULL) return ESP_FAIL;

    uart_config_t uart_config = {
        .baud_rate = UART_4G_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    ESP_ERROR_CHECK(uart_driver_install(UART_4G_PORT_NUM, UART_4G_BUF_SIZE, UART_4G_BUF_SIZE, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_4G_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_4G_PORT_NUM, UART_4G_TX_PIN, UART_4G_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    if (MASTER_PPS_PIN >= 0) {
        gpio_config_t io_conf = {
            .intr_type = GPIO_INTR_POSEDGE,
            .pin_bit_mask = (1ULL << MASTER_PPS_PIN),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = 0,
            .pull_down_en = 1
        };
        gpio_config(&io_conf);
        gpio_install_isr_service(0);
        gpio_isr_handler_add(MASTER_PPS_PIN, pps_gpio_isr_handler, NULL);
    }
    drv_4g_gnss_reset_fix();
    xTaskCreate(task_4g_uart_rx, "4g_router_task", 4096, NULL, 5, NULL);
    return ESP_OK;
}

esp_err_t drv_4g_send_at_cmd(const char *cmd, const char *expected_response, uint32_t timeout_ms, char *out_response, size_t out_max_len)
{
    xEventGroupClearBits(g_at_event_group, AT_EVENT_OK | AT_EVENT_ERROR);
    if (cmd != NULL) {
        uart_write_bytes(UART_4G_PORT_NUM, cmd, strlen(cmd));
    }
    EventBits_t bits = xEventGroupWaitBits(g_at_event_group, AT_EVENT_OK | AT_EVENT_ERROR, pdTRUE, pdFALSE, pdMS_TO_TICKS(timeout_ms));
    if (bits & AT_EVENT_OK) return ESP_OK;
    if (bits & AT_EVENT_ERROR) return ESP_FAIL;
    return ESP_ERR_TIMEOUT;
}

esp_err_t drv_4g_gnss_power(bool enable) {
    const char *cmd = enable ? "AT+MGPSC=1\r\n" : "AT+MGPSC=0\r\n";
    return drv_4g_send_at_cmd(cmd, "OK", 3000, NULL, 0);
}

// 切換至：校準模式
esp_err_t drv_4g_start_nmea_stream(void) {
    _is_in_silent_mode = false; 
    ESP_LOGI(TAG, "Entering Calibration Mode (NMEA ON)");
    return drv_4g_send_at_cmd("AT+MGPSGET=ALL,1\r\n", "OK", 2000, NULL, 0);
}

// 切換至：寂靜模式
esp_err_t drv_4g_stop_nmea_stream(void) {
    esp_err_t err = drv_4g_send_at_cmd("AT+MGPSGET=ALL,0\r\n", "OK", 2000, NULL, 0);
    if (err == ESP_OK) {
        _is_in_silent_mode = true;
        ESP_LOGI(TAG, "Entering Silent Mode (NMEA OFF, PPS Auto-tracking ON)");
    }
    return err;
}

esp_err_t drv_4g_set_apn(void) {
    return drv_4g_send_at_cmd("AT+QICSGP=1,1,\"internet\",\"\",\"\"\r\n", "OK", 3000, NULL, 0);
}

esp_err_t drv_4g_tcp_connect(const char *ip, int port) {
    char cmd[128];
    snprintf(cmd, sizeof(cmd), "AT+TCPNUM=1,\"%s\",%d\r\n", ip, port);
    return drv_4g_send_at_cmd(cmd, "+CIPOPEN: SUCCESS,1", 8000, NULL, 0);
}

esp_err_t drv_4g_tcp_send(const uint8_t *data, size_t len) {
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "AT+CIPSEND=1,%d\r\n", len);
    if (drv_4g_send_at_cmd(cmd, ">", 3000, NULL, 0) != ESP_OK) return ESP_FAIL;
    uart_write_bytes(UART_4G_PORT_NUM, (const char*)data, len);
    return drv_4g_send_at_cmd(NULL, "+CIPSEND:SUCCESS", 5000, NULL, 0);
}

esp_err_t drv_4g_tcp_close(void) {
    return drv_4g_send_at_cmd("AT+CIPCLOSE=1\r\n", "+CIPCLOSE: SUCCESS,1", 3000, NULL, 0);
}

void drv_4g_gnss_reset_fix(void) {
    _rmc_count = 0; _gga_count = 0;
    _sum_lat = 0; _sum_lon = 0; _sum_alt = 0.0f;
    _final_fix.is_fixed = false;
    _final_fix.is_time_synced = false;
}

gps_fix_t drv_4g_gnss_get_fix(void) {
    return _final_fix;
}