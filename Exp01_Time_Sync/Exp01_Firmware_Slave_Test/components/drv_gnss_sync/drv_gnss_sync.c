#include <stdio.h>
#include <stdlib.h>
#include "drv_gnss_sync.h"
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "GNSS_SLAVE";
#define BUF_SIZE 1024
#define RD_BUF_SIZE (BUF_SIZE)

static int g_gps_uart_num = UART_NUM_1; 
static QueueHandle_t uart_queue;

// --- PPS 時間同步變數 ---
static volatile time_t _next_pps_timestamp = 0;

// --- Survey-In 設定 ---
#define SURVEY_SAMPLES 60   
static int _rmc_count = 0; 
static int _gga_count = 0; 

static int64_t _sum_lat = 0;
static int64_t _sum_lon = 0;
static float   _sum_alt = 0.0f;

static gps_fix_t _final_fix = {0};

static int32_t _convert_nmea_to_scaled(float nmea_val) 
{
    int degrees = (int)(nmea_val / 100);             
    float minutes = nmea_val - (degrees * 100);      
    float decimal_deg = degrees + (minutes / 60.0f); 
    return (int32_t)(decimal_deg * 10000000);        
}

// ==========================================
// PPS 中斷處理 (ISR) - 三階段解耦架構
// ==========================================
static void IRAM_ATTR pps_gpio_isr_handler(void* arg)
{
    if (!_final_fix.is_time_synced) {
        // 階段一: 錨點初始化 (依據 NMEA 時間)
        if (_next_pps_timestamp > 0) {
            struct timeval tv;
            tv.tv_sec = _next_pps_timestamp; 
            tv.tv_usec = 0;                  
            
            settimeofday(&tv, NULL);         
            _final_fix.is_time_synced = true; 
        }
    } else {
        // 階段二: 硬體同步 (純硬體精準遞增)
        struct timeval tv;
        gettimeofday(&tv, NULL);
        
        tv.tv_sec += 1; 
        tv.tv_usec = 0; 
        
        settimeofday(&tv, NULL);
    }
}

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

    while ((token = strsep(&rest, ",")) != NULL) {
        size_t len = strlen(token);
        switch (field) {
            case 1: 
                if (len > 0) raw_time = atoi(token); 
                break; 
            case 2: 
                if (len > 0 && token[0] == 'A') valid = true; 
                break;
            case 3: 
                if (!_final_fix.is_fixed && len > 0) 
                    curr_lat = _convert_nmea_to_scaled(strtof(token, NULL)); 
                break;
            case 4: 
                if (!_final_fix.is_fixed && len > 0 && token[0] == 'S') 
                    curr_lat *= -1; 
                break;
            case 5: 
                if (!_final_fix.is_fixed && len > 0) 
                    curr_lon = _convert_nmea_to_scaled(strtof(token, NULL)); 
                break;
            case 6: 
                if (!_final_fix.is_fixed && len > 0 && token[0] == 'W') 
                    curr_lon *= -1; 
                break;
            case 9: 
                if (len > 0) raw_date = atoi(token); 
                break; 
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

        // 強制設定時區為 UTC，避免 mktime() 自動加計時區偏移
        setenv("TZ", "UTC", 1);
        tzset();

        time_t current_gps_time = mktime(&tm_struct);
        
        if (!_final_fix.is_time_synced) {
            // 階段一: 準備下一個 PPS 的時間戳
            _next_pps_timestamp = current_gps_time + 1;
        } else {
            // 階段三: 背景稽核 (Watchdog)
            struct timeval tv;
            gettimeofday(&tv, NULL);
            
            int64_t diff = llabs((int64_t)tv.tv_sec - (int64_t)current_gps_time);
            if (diff >= 2) {
                ESP_LOGE(TAG, "Watchdog Warning: Time drift detected (Diff: %llds). Forcing re-sync.", diff);
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
            case 6: 
                if (len > 0 && token[0] != '0') {
                    quality_valid = true;
                }
                break;
            case 9: 
                if (!_final_fix.is_fixed && len > 0) {
                    current_alt = strtof(token, NULL);
                }
                break;
        }
        field++;
    }

    if (quality_valid && !_final_fix.is_fixed) {
        _sum_alt += current_alt;
        _gga_count++;
    }
}

static void _process_nmea_line(char *line)
{
    if (strncmp(line, "$GNRMC", 6) == 0 || strncmp(line, "$GPRMC", 6) == 0) {
        _parse_rmc(line); 
        
        if (!_final_fix.is_fixed && 
        _rmc_count >= SURVEY_SAMPLES && 
        _gga_count >= SURVEY_SAMPLES) {
        
            _final_fix.latitude = (int32_t)(_sum_lat / _rmc_count);
            _final_fix.longitude = (int32_t)(_sum_lon / _rmc_count);
            _final_fix.altitude = _sum_alt / _gga_count;
            _final_fix.is_fixed = true;
 
        }
    } 
    else if (strncmp(line, "$GNGGA", 6) == 0 || strncmp(line, "$GPGGA", 6) == 0) {
        _parse_gga(line);
    }
}

static void gps_event_task(void *pvParameters)
{
    uart_event_t event;
    uint8_t* dtmp = NULL;
    
    while (dtmp == NULL) {
        dtmp = (uint8_t*) malloc(RD_BUF_SIZE);
        if (dtmp == NULL) {
            ESP_LOGE(TAG, "Malloc failed. Retrying in 1 sec...");
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    uart_flush_input(g_gps_uart_num); 
    xQueueReset(uart_queue);

    for (;;) {
        if (xQueueReceive(uart_queue, (void *)&event, (TickType_t)portMAX_DELAY)) {

            size_t buffered_size;
            uart_get_buffered_data_len(g_gps_uart_num, &buffered_size);

            // 防護機制: 放寬 flush 門檻至 2048 以適應 10Hz 資料量
            if (buffered_size > 2048) {
                ESP_LOGW(TAG, "Buffer full (%d bytes), flushing old data.", buffered_size);
                uart_flush_input(g_gps_uart_num); 
                xQueueReset(uart_queue);          
                continue;                         
            }

            memset(dtmp, 0, RD_BUF_SIZE);
            switch (event.type) {
            case UART_PATTERN_DET:
                int pos = uart_pattern_pop_pos(g_gps_uart_num);
                if (pos != -1) {
                    int len = uart_read_bytes(g_gps_uart_num, dtmp, pos+1, 100 / portTICK_PERIOD_MS);
                    dtmp[len] = '\0';
                    char *start = strchr((char *)dtmp, '$');
                    if (start != NULL) {
                        _process_nmea_line(start);
                    }
                } else {
                    uart_flush_input(g_gps_uart_num);
                }
                break;
            case UART_FIFO_OVF:
            case UART_BUFFER_FULL:
                uart_flush_input(g_gps_uart_num);
                xQueueReset(uart_queue);
                break;
            default:
                break;
            }
        }
    }
    free(dtmp);
    vTaskDelete(NULL);
}

void send_cmd(int uart_num, const char* cmd) {
    uart_write_bytes(uart_num, cmd, strlen(cmd));
    vTaskDelay(pdMS_TO_TICKS(100)); 
}

void configure_atgm336h(int uart_num)
{
    ESP_LOGI(TAG, "Configuring ATGM336H GPS...");

    // 1. 設定 Update Rate 為 10Hz
    send_cmd(uart_num, "$PCAS02,100*1E\r\n");
    ESP_LOGI(TAG, "Update Rate set to 10Hz");

    // 僅輸出 GGA 與 RMC 封包，減輕 UART 負載
    send_cmd(uart_num, "$PCAS03,1,0,0,0,1,0,0,0,0,0,,,0,0*02\r\n");

    // 2. 設定 Baud Rate 為 115200
    send_cmd(uart_num, "$PCAS01,5*19\r\n");
    ESP_LOGI(TAG, "Baud Rate set command sent. Switching ESP32 UART...");

    // 3. 切換 ESP32 UART 速度
    vTaskDelay(pdMS_TO_TICKS(200)); 
    uart_set_baudrate(uart_num, 115200);
    ESP_LOGI(TAG, "ESP32 UART switched to 115200");

    // 4. 儲存設定
    send_cmd(uart_num, "$PCAS00*01\r\n");
    ESP_LOGI(TAG, "Configuration Saved to Flash");
}

void gnss_sync_init(void)
{
    ESP_LOGI(TAG, "Initializing GNSS Parser...");
    g_gps_uart_num = GPS_UART_NUM;
    
    uart_config_t uart_config = {
        .baud_rate = GPS_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    ESP_ERROR_CHECK(uart_driver_install(GPS_UART_NUM, BUF_SIZE * 4, BUF_SIZE * 4, 100, &uart_queue, 0));
    ESP_ERROR_CHECK(uart_param_config(GPS_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(GPS_UART_NUM, GPS_TX_PIN, GPS_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    uart_enable_pattern_det_baud_intr(GPS_UART_NUM, '\n', 1, 9, 0, 0);
    uart_pattern_queue_reset(GPS_UART_NUM, 100);

    if (GPS_PPS_PIN >= 0) { 
        gpio_config_t io_conf = {};
        io_conf.intr_type = GPIO_INTR_POSEDGE; 
        io_conf.pin_bit_mask = (1ULL << GPS_PPS_PIN);
        io_conf.mode = GPIO_MODE_INPUT;
        io_conf.pull_up_en = 0;
        io_conf.pull_down_en = 0;
        gpio_config(&io_conf);

        if (gpio_install_isr_service(0) != ESP_OK) {
            ESP_LOGW(TAG, "ISR Service already installed, skipping init.");
        }
        gpio_isr_handler_add(GPS_PPS_PIN, pps_gpio_isr_handler, NULL);
        ESP_LOGI(TAG, "PPS enabled on GPIO %d", GPS_PPS_PIN);
    }

    configure_atgm336h(GPS_UART_NUM);

    xTaskCreate(gps_event_task, "gps_task", 4096, NULL, 12, NULL);

    ESP_LOGI(TAG, "GPS Init on UART%d (TX:%d, RX:%d, Baud:%d)", GPS_UART_NUM, GPS_TX_PIN, GPS_RX_PIN, GPS_BAUD);
}

void gnss_reset_fix(void) {
    _rmc_count = 0;
    _gga_count = 0;
    _sum_lat = 0;
    _sum_lon = 0;
    _sum_alt = 0.0f; 
    _final_fix.is_fixed = false;
    _final_fix.is_time_synced = false; 
    ESP_LOGW(TAG, "Fix Reset. Surveying & Sync started...");
}

gps_fix_t gnss_get_fix(void)
{
    return _final_fix;
}