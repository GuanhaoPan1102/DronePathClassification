#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "driver/uart.h"
#include "esp_timer.h" 
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
// 軟體 PPS (Leading-Edge Sync) 核心變數
// ==========================================
static volatile time_t _next_utc_sec = 0;               // 下一秒的 UTC 秒數預報
static int64_t _last_uart_rx_time_us = 0;               // 上次收到 UART 字元的系統微秒時間 (用於沉默期檢測)

// 定義 NMEA 爆發之間的最小沉默期 (微秒)。
// 假設 1Hz 輸出，爆發期約耗時 100ms，沉默期應大於 500ms (500,000us)
#define NMEA_SILENCE_THRESHOLD_US 500000 

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

    // 預報：解析當前的 RMC 時間，計算並儲存「下一秒」的標準 UTC 時間
    if (raw_date > 0 && raw_time > 0) {
        struct tm tm_struct = {0};
        tm_struct.tm_hour = raw_time / 10000;
        tm_struct.tm_min  = (raw_time % 10000) / 100;
        tm_struct.tm_sec  = raw_time % 100;
        tm_struct.tm_mday = raw_date / 10000;
        tm_struct.tm_mon  = ((raw_date % 10000) / 100) - 1; 
        tm_struct.tm_year = (raw_date % 100) + 100;         

        // 將解析出的時間轉為秒數，並加 1 秒作為下一次 NMEA 第一個 '$' 的目標時間
        _next_utc_sec = mktime(&tm_struct) + 1; 
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
            ESP_LOGI(TAG, "MASTER Node Position FIXED! Software PPS Sync Active.");
        }
    } else if (strstr(buf, "GGA")) {
        _parse_gga(buf);
    }
}

// ==========================================
// 字元級分流路由器 (Router Task) - 包含軟體 PPS 邏輯
// ==========================================
static void task_4g_uart_rx(void *pvParameters)
{
    uint8_t rx_byte;
    char line_buf[UART_4G_BUF_SIZE];
    int line_idx = 0;
    
    for (;;) {
        // 採用超短超時 (10ms)，確保能迅速抓到單一字元
        int len = uart_read_bytes(UART_4G_PORT_NUM, &rx_byte, 1, 10 / portTICK_PERIOD_MS);
        
        if (len > 0) {
            int64_t current_hw_us = esp_timer_get_time();
            
            // ==========================================
            // 軟體 PPS 核心邏輯：偵測沉默期與第一顆 '$'
            // ==========================================
            if (rx_byte == '$') {
                // 如果距離上次收到字元的時間大於沉默門檻，這就是新一秒的「第一顆 $」
                if ((current_hw_us - _last_uart_rx_time_us) > NMEA_SILENCE_THRESHOLD_US) {
                    
                    // 如果我們已經有下一秒的預報時間，立刻寫入系統時間！
                    if (_next_utc_sec > 0) {
                        struct timeval tv;
                        tv.tv_sec = _next_utc_sec;
                        // 這裡可以加入微小的固定補償值 (例如 GNSS 內部運算延遲，依實驗微調)
                        tv.tv_usec = 35000; // 假設 GNSS 模組從整秒到吐出第一個 $ 耗時 35ms
                        
                        settimeofday(&tv, NULL);
                        
                        if (!_final_fix.is_time_synced) {
                            _final_fix.is_time_synced = true;
                            ESP_LOGI(TAG, "System Time Synced via Software PPS (Leading Edge)!");
                        }
                    }
                }
            }
            
            // 更新最後一次收到字元的時間
            _last_uart_rx_time_us = current_hw_us;

            // ==========================================
            // 一般字串處理邏輯
            // ==========================================
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
                    // 永遠進行 NMEA 解析，以維持 _next_utc_sec 的更新
                    drv_4g_gnss_process_nmea(line_buf);
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
    ESP_LOGI(TAG, "Initializing 4G Module & GNSS Engine (Software PPS Architecture)...");

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
    
    // 為了降低硬體緩衝區造成的延遲，可以考慮在 uart_driver_install 後，將 RX Full Threshold 調低
    // 這裡我們維持預設，透過超短 Timeout (10ms) 在 Task 層級盡可能快地抓取字元
    ESP_ERROR_CHECK(uart_driver_install(UART_4G_PORT_NUM, UART_4G_BUF_SIZE, UART_4G_BUF_SIZE, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_4G_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_4G_PORT_NUM, UART_4G_TX_PIN, UART_4G_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // 移除所有實體 GPIO PPS 的註冊程式碼
    drv_4g_gnss_reset_fix();
    
    // 建立 Router Task
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
    if(ACTIVE_ANTENNA){
        drv_4g_send_at_cmd("AT+GPSACT", "OK", 3000, NULL, 0);
    }
    const char *cmd = enable ? "AT+MGPSC=1\r\n" : "AT+MGPSC=0\r\n";
    return drv_4g_send_at_cmd(cmd, "OK", 3000, NULL, 0);
}

// 啟動 NMEA 串流
esp_err_t drv_4g_start_nmea_stream(void) {
    ESP_LOGI(TAG, "Starting NMEA Stream (Required for Software PPS)");
    return drv_4g_send_at_cmd("AT+MGPSGET=ALL,1\r\n", "OK", 2000, NULL, 0);
}

// 停止 NMEA 串流 (警告：這會癱瘓軟體 PPS)
esp_err_t drv_4g_stop_nmea_stream(void) {
    ESP_LOGW(TAG, "WARNING: Stopping NMEA will halt Software PPS Time Sync!");
    _next_utc_sec = 0; 
    _final_fix.is_time_synced = false;
    return drv_4g_send_at_cmd("AT+MGPSGET=ALL,0\r\n", "OK", 2000, NULL, 0);
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
    _next_utc_sec = 0;
}

gps_fix_t drv_4g_gnss_get_fix(void) {
    return _final_fix;
}