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
#include "esp_log.h"
#include "drv_4g_gnss_sync.h"

static const char *TAG = "4G_GNSS_SYNC";

// ==========================================
// AT 指令事件標誌 (Event Group)
// ==========================================
// 用於協調背景 Router Task 與前景 AT 指令發送函式
static EventGroupHandle_t g_at_event_group;
#define AT_EVENT_OK    BIT0
#define AT_EVENT_ERROR BIT1

// ==========================================
// GNSS 同步與定位私有變數
// ==========================================
// PPS 錨點時間戳 (預計下一個 PPS 脈衝抵達時的 UNIX 秒數)
static volatile time_t _next_pps_timestamp = 0;

// Survey-In 樣本累加與計數器 (目標 60 次採樣)
#define SURVEY_SAMPLES 60   
static int _rmc_count = 0; 
static int _gga_count = 0; 

static int64_t _sum_lat = 0;
static int64_t _sum_lon = 0;
static float   _sum_alt = 0.0f;

// 儲存最終定位與同步狀態 (使用 sys_defs.h 定義之結構)
static gps_fix_t _final_fix = {0};

// ==========================================
// 輔助函式 (Internal Helper)
// ==========================================
/**
 * @brief 將 NMEA 格式 (DDMM.MMMM) 轉換為十進位度數 * 10^7 (符合 sys_defs.h 規範)
 */
static int32_t _convert_nmea_to_scaled(float nmea_val) 
{
    int degrees = (int)(nmea_val / 100);             
    float minutes = nmea_val - (degrees * 100);      
    float decimal_deg = degrees + (minutes / 60.0f); 
    return (int32_t)(decimal_deg * 10000000);        
}

// ==========================================
// [核心] PPS 中斷處理 (ISR) - 三階段解耦架構
// ==========================================
/**
 * @brief PPS 硬體中斷處理函式
 * @note 優先處理時間對齊與微秒歸零，確保 Master 節點具備與 Slave 同等的精準度
 */
static void IRAM_ATTR pps_gpio_isr_handler(void* arg)
{
    if (!_final_fix.is_time_synced) {
        // 階段一: 錨點初始化 (依據 NMEA 提供的預測時間)
        if (_next_pps_timestamp > 0) {
            struct timeval tv;
            tv.tv_sec = _next_pps_timestamp; 
            tv.tv_usec = 0;                  
            
            settimeofday(&tv, NULL);         
            _final_fix.is_time_synced = true; 
        }
    } else {
        // 階段二: 硬體同步 (由實體脈衝驅動，每一秒自動遞增並歸零微秒)
        struct timeval tv;
        gettimeofday(&tv, NULL);
        
        tv.tv_sec += 1; 
        tv.tv_usec = 0; 
        
        settimeofday(&tv, NULL);
    }
}

// ==========================================
// RMC 解析 (負責時間同步、日期與 2D 定位累加)
// ==========================================
static void _parse_rmc(char *line)
{
    char *rest = line;
    char *token;
    int field = 0;

    int raw_time = 0; 
    int raw_date = 0; 
    int32_t curr_lat = 0;
    int32_t curr_lon = 0;
    bool valid = false;

    // 使用 strsep 確保能正確跳過連續逗號 (,,)
    while ((token = strsep(&rest, ",")) != NULL) {
        size_t len = strlen(token);
        switch (field) {
            case 1: if (len > 0) raw_time = atoi(token); break; 
            case 2: if (len > 0 && token[0] == 'A') valid = true; break;
            case 3: if (!_final_fix.is_fixed && len > 0) curr_lat = _convert_nmea_to_scaled(strtof(token, NULL)); break;
            case 4: if (!_final_fix.is_fixed && len > 0 && token[0] == 'S') curr_lat *= -1; break;
            case 5: if (!_final_fix.is_fixed && len > 0) curr_lon = _convert_nmea_to_scaled(strtof(token, NULL)); break;
            case 6: if (!_final_fix.is_fixed && len > 0 && token[0] == 'W') curr_lon *= -1; break;
            case 9: if (len > 0) raw_date = atoi(token); break; 
        }
        field++;
    }

    // --- 任務 A: 時間同步與 Watchdog ---
    if (raw_date > 0 && raw_time > 0) {
        struct tm tm_struct = {0};
        tm_struct.tm_hour = raw_time / 10000;
        tm_struct.tm_min  = (raw_time % 10000) / 100;
        tm_struct.tm_sec  = raw_time % 100;
        tm_struct.tm_mday = raw_date / 10000;
        tm_struct.tm_mon  = ((raw_date % 10000) / 100) - 1; 
        tm_struct.tm_year = (raw_date % 100) + 100;         

        setenv("TZ", "UTC", 1);
        tzset();
        time_t current_gps_time = mktime(&tm_struct);
        
        if (!_final_fix.is_time_synced) {
            _next_pps_timestamp = current_gps_time + 1;
        } else {
            // 階段三: 背景稽核 (Watchdog)，防止跨秒延遲
            struct timeval tv;
            gettimeofday(&tv, NULL);
            int64_t diff = llabs((int64_t)tv.tv_sec - (int64_t)current_gps_time);
            if (diff >= 2) {
                ESP_LOGE(TAG, "Watchdog Warning: Time drift detected. Forcing re-sync.");
                _final_fix.is_time_synced = false; 
            }
        }
    }

    // --- 任務 B: 位置平均 (Survey-In) ---
    if (!_final_fix.is_fixed && valid) {
        _sum_lat += curr_lat;
        _sum_lon += curr_lon;
        _rmc_count++;
    }
}

// ==========================================
// GGA 解析 (負責高度累加)
// ==========================================
static void _parse_gga(char *line)
{
    char *rest = line;
    char *token;
    int field = 0;
    bool quality_valid = false;
    float current_alt = 0.0f;

    while ((token = strsep(&rest, ",")) != NULL) {
        size_t len = strlen(token);
        switch (field) {
            case 6: if (len > 0 && token[0] != '0') quality_valid = true; break;
            case 9: if (!_final_fix.is_fixed && len > 0) current_alt = strtof(token, NULL); break;
        }
        field++;
    }

    if (quality_valid && !_final_fix.is_fixed) {
        _sum_alt += current_alt;
        _gga_count++;
    }
}

// ==========================================
// 定位與解析入口 API
// ==========================================

/**
 * @brief 餵入 NMEA 字串進行解析
 * @details 由 Router Task 呼叫，負責判斷封包類型並觸發 Survey-In 鎖定邏輯
 */
void drv_4g_gnss_process_nmea(const char *nmea_line)
{
    // 由於 strsep 會修改字串內容，我們建立一個局部緩衝區處理
    char buf[128];
    strncpy(buf, nmea_line, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    if (strstr(buf, "RMC")) {
        _parse_rmc(buf);
        
        // 檢查是否滿足 Survey-In 鎖定條件 (RMC 與 GGA 均達到 60 次採樣)
        if (!_final_fix.is_fixed && _rmc_count >= SURVEY_SAMPLES && _gga_count >= SURVEY_SAMPLES) {
            _final_fix.latitude = (int32_t)(_sum_lat / _rmc_count);
            _final_fix.longitude = (int32_t)(_sum_lon / _rmc_count);
            _final_fix.altitude = _sum_alt / _gga_count;
            _final_fix.is_fixed = true;

            // --- 新增：取得並印出校正後的時間 (與 Slave 一致) ---
            struct timeval tv;
            gettimeofday(&tv, NULL);
            
            // 轉換為 UTC+8 台北時間
            time_t local_time = tv.tv_sec + (8 * 3600);
            struct tm ti;
            gmtime_r(&local_time, &ti);
            
            char time_str[32];
            strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", &ti);

            ESP_LOGI(TAG, "MASTER Node Position FIXED!");
            ESP_LOGI(TAG, "Lat:%d, Lon:%d, Alt:%.2f", 
                     _final_fix.latitude, _final_fix.longitude, _final_fix.altitude);
            ESP_LOGI(TAG, "Current Time: %s (UTC+8)", time_str);
            // -----------------------------------------------
        }
    } else if (strstr(buf, "GGA")) {
        _parse_gga(buf);
    }
}

// ==========================================
// 背景接收路由器任務 (Router Task)
// ==========================================
/**
 * @brief 唯一的 UART 接收入口，負責將 NMEA 與 AT 指令回覆分流
 */
static void task_4g_uart_rx(void *pvParameters)
{
    uint8_t *dtmp = (uint8_t *)malloc(UART_4G_BUF_SIZE);
    
    for (;;) {
        // 從 UART 讀取資料
        int len = uart_read_bytes(UART_4G_PORT_NUM, dtmp, UART_4G_BUF_SIZE - 1, 100 / portTICK_PERIOD_MS);
        
        if (len > 0) {
            dtmp[len] = '\0';
            char *line = (char *)dtmp;
            
            // --- 路由器分流邏輯 (Router Logic) ---
            
            // 情況 A: NMEA 定位資料 (以 $ 開頭)
            if (line[0] == '$') {
                // 呼叫上一段實作的處理函式
                drv_4g_gnss_process_nmea(line);
            } 
            // 情況 B: 4G 模組狀態回覆 (根據舊版 Driver 經驗匹配)
            else {
                if (strstr(line, "OK") || 
                    strstr(line, "+CIPOPEN: SUCCESS,1") || 
                    strstr(line, "+CIPSEND:SUCCESS") || 
                    strstr(line, "+CIPCLOSE: SUCCESS,1") ||
                    strstr(line, ">")) {
                    
                    // 觸發事件位元，喚醒正在等待的 drv_4g_send_at_cmd
                    xEventGroupSetBits(g_at_event_group, AT_EVENT_OK);
                } 
                else if (strstr(line, "ERROR")) {
                    xEventGroupSetBits(g_at_event_group, AT_EVENT_ERROR);
                }
            }
        }
    }
    free(dtmp);
    vTaskDelete(NULL);
}

// ==========================================
// 統一初始化函式
// ==========================================
esp_err_t drv_4g_init(void)
{
    ESP_LOGI(TAG, "Initializing 4G Module & GNSS Engine (Integrated)...");

    // 1. 建立 AT 事件標誌組
    g_at_event_group = xEventGroupCreate();
    if (g_at_event_group == NULL) return ESP_FAIL;

    // 2. 配置 UART 硬體 (依照 4G 模組 115200 需求)
    uart_config_t uart_config = {
        .baud_rate = UART_4G_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    // 安裝驅動，這裡不對外暴露 Queue，因為只有 Router Task 會讀取
    ESP_ERROR_CHECK(uart_driver_install(UART_4G_PORT_NUM, UART_4G_BUF_SIZE * 2, UART_4G_BUF_SIZE * 2, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_4G_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_4G_PORT_NUM, UART_4G_TX_PIN, UART_4G_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // 3. 配置 PPS 中斷腳位 (與 Slave 邏輯一致)
    if (MASTER_PPS_PIN >= 0) {
        gpio_config_t io_conf = {
            .intr_type = GPIO_INTR_POSEDGE,
            .pin_bit_mask = (1ULL << MASTER_PPS_PIN),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = 0,
            .pull_down_en = 0
        };
        gpio_config(&io_conf);
        
        // 確保 ISR 服務已安裝
        gpio_install_isr_service(0);
        gpio_isr_handler_add(MASTER_PPS_PIN, pps_gpio_isr_handler, NULL);
    }

    // 4. 啟動背景路由器 Task
    // 優先權設為 5，確保它比一般處理邏輯更高，避免 UART Buffer 溢位
    xTaskCreate(task_4g_uart_rx, "4g_router_task", 4096, NULL, 5, NULL);

    return ESP_OK;
}

// ==========================================
// 4G 網路與 AT 指令 API 實作
// ==========================================

/**
 * @brief 核心 AT 發送函式 (事件驅動版)
 */
esp_err_t drv_4g_send_at_cmd(const char *cmd, const char *expected_response, uint32_t timeout_ms, char *out_response, size_t out_max_len)
{
    // 1. 清除先前的事件標誌
    xEventGroupClearBits(g_at_event_group, AT_EVENT_OK | AT_EVENT_ERROR);

    // 2. 寫入 UART
    if (cmd != NULL) {
        uart_write_bytes(UART_4G_PORT_NUM, cmd, strlen(cmd));
        ESP_LOGD(TAG, "Sent: %s", cmd);
    }

    // 3. 阻塞等待 Router Task 的信號
    EventBits_t bits = xEventGroupWaitBits(
        g_at_event_group,
        AT_EVENT_OK | AT_EVENT_ERROR,
        pdTRUE, 
        pdFALSE, 
        pdMS_TO_TICKS(timeout_ms)
    );

    // 4. 返回結果
    if (bits & AT_EVENT_OK) return ESP_OK;
    if (bits & AT_EVENT_ERROR) return ESP_FAIL;
    return ESP_ERR_TIMEOUT;
}

/**
 * @brief 開啟/關閉 GPS 模組電源 (匹配舊版指令 AT+MGPSC)
 */
esp_err_t drv_4g_gnss_power(bool enable)
{
    const char *cmd = enable ? "AT+MGPSC=1\r\n" : "AT+MGPSC=0\r\n";
    return drv_4g_send_at_cmd(cmd, "OK", 3000, NULL, 0);
}

/**
 * @brief 開啟 NMEA 串流 (開始 Survey-In)
 */
esp_err_t drv_4g_start_nmea_stream(void)
{
    return drv_4g_send_at_cmd("AT+MGPSGET=ALL,1\r\n", "OK", 2000, NULL, 0);
}

/**
 * @brief 關閉 NMEA 串流 (釋放 UART)
 */
esp_err_t drv_4g_stop_nmea_stream(void)
{
    return drv_4g_send_at_cmd("AT+MGPSGET=ALL,0\r\n", "OK", 2000, NULL, 0);
}

/**
 * @brief 設定 APN (匹配舊版指令 AT+QICSGP)
 */
esp_err_t drv_4g_set_apn(void)
{
    return drv_4g_send_at_cmd("AT+QICSGP=1,1,\"internet\",\"\",\"\"\r\n", "OK", 3000, NULL, 0);
}

/**
 * @brief 建立 TCP 連線 (匹配舊版回應 +CIPOPEN: SUCCESS,1)
 */
esp_err_t drv_4g_tcp_connect(const char *ip, int port)
{
    char cmd[128];
    snprintf(cmd, sizeof(cmd), "AT+TCPNUM=1,\"%s\",%d\r\n", ip, port);
    // Router Task 會監控 +CIPOPEN 字串並觸發 AT_EVENT_OK
    return drv_4g_send_at_cmd(cmd, "+CIPOPEN: SUCCESS,1", 8000, NULL, 0);
}

/**
 * @brief 發送 TCP 資料 (兩段式：宣告長度 -> 灌資料 -> 等成功訊息)
 */
esp_err_t drv_4g_tcp_send(const uint8_t *data, size_t len)
{
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "AT+CIPSEND=1,%d\r\n", len);
    
    // 步驟 1: 預期拿到 '>' 符號 (Router Task 已支持)
    if (drv_4g_send_at_cmd(cmd, ">", 3000, NULL, 0) != ESP_OK) return ESP_FAIL;

    // 步驟 2: 寫入真實二進位資料
    uart_write_bytes(UART_4G_PORT_NUM, (const char*)data, len);

    // 步驟 3: 等待 +CIPSEND:SUCCESS
    return drv_4g_send_at_cmd(NULL, "+CIPSEND:SUCCESS", 5000, NULL, 0);
}

/**
 * @brief 關閉 TCP 連線 (匹配舊版回應 +CIPCLOSE: SUCCESS,1)
 */
esp_err_t drv_4g_tcp_close(void)
{
    return drv_4g_send_at_cmd("AT+CIPCLOSE=1\r\n", "+CIPCLOSE: SUCCESS,1", 3000, NULL, 0);
}

// ==========================================
// 定位與狀態 API
// ==========================================

void drv_4g_gnss_reset_fix(void)
{
    _rmc_count = 0; _gga_count = 0;
    _sum_lat = 0; _sum_lon = 0; _sum_alt = 0.0f;
    _final_fix.is_fixed = false;
    _final_fix.is_time_synced = false;
}

gps_fix_t drv_4g_gnss_get_fix(void)
{
    return _final_fix;
}