#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "drv_gps_read.h"

static const char *TAG = "GNSS";
#define BUF_SIZE 1024
#define RD_BUF_SIZE (BUF_SIZE)

static int g_gps_uart_num = UART_NUM_1; 
static QueueHandle_t uart_queue;

// 🌟 新增：Mutex 用於保護跨 Task 的資料讀寫
static SemaphoreHandle_t gps_data_mutex = NULL;

// 儲存最新的解析結果
static gps_data_t _latest_data = {0};

static int32_t _convert_nmea_to_scaled(float nmea_val) // DDMM.MMMM
{
    int degrees = (int)(nmea_val / 100);             
    float minutes = nmea_val - (degrees * 100);      
    float decimal_deg = degrees + (minutes / 60.0f); 
    return (int32_t)(decimal_deg * 10000000);        
}

// 安全解析函式：使用本地副本避免破壞原始 RingBuffer 資料
static void _parse_rmc(const char *line)
{
    char buf[128];
    strncpy(buf, line, sizeof(buf)-1);
    buf[sizeof(buf)-1] = '\0';

    char *rest = buf;
    char *token;
    int field_index = 0;
    
    // 準備一個暫存的結構，解析完且驗證沒問題後，再一口氣鎖 Mutex 寫入
    gps_data_t temp_data = _latest_data; 

    while((token = strsep(&rest, ",")) != NULL ) {
        size_t len = strlen(token);

        switch (field_index) {
        case 2:
            if (len>0) {
                if (token[0] == 'A') temp_data.is_valid = 1;
                else if (token[0] == 'V') {
                    temp_data.is_valid = 0;
                    temp_data.latitude_scaled = 0;
                    temp_data.longitude_scaled = 0;
                    temp_data.speed_kph_scaled = 0;
                    // 若無效，直接更新並結束
                    if (xSemaphoreTake(gps_data_mutex, portMAX_DELAY)) {
                        _latest_data = temp_data;
                        xSemaphoreGive(gps_data_mutex);
                    }
                    return;
                }
            }
            break;

        case 3:
            if (len>0) temp_data.latitude_scaled = _convert_nmea_to_scaled(strtof(token, NULL));
            break;
        case 4:
            if (len>0 && token[0] == 'S') temp_data.latitude_scaled *= -1;
            break;
        case 5:
            if (len>0) temp_data.longitude_scaled = _convert_nmea_to_scaled(strtof(token, NULL));
            break;
        case 6:
            if (len>0 && token[0] == 'W') temp_data.longitude_scaled *= -1;
            break;
        case 7:
            if (len>0) temp_data.speed_kph_scaled = (int16_t)(strtof(token, NULL) * 185.2f);
            break;
        }
        field_index++;
    }

    // 解析成功，鎖上 Mutex 寫入全域變數
    if (xSemaphoreTake(gps_data_mutex, portMAX_DELAY)) {
        _latest_data.is_valid = temp_data.is_valid;
        _latest_data.latitude_scaled = temp_data.latitude_scaled;
        _latest_data.longitude_scaled = temp_data.longitude_scaled;
        _latest_data.speed_kph_scaled = temp_data.speed_kph_scaled;
        xSemaphoreGive(gps_data_mutex);
    }
}

static void _parse_gga(const char *line)
{
    char buf[128];
    strncpy(buf, line, sizeof(buf)-1);
    buf[sizeof(buf)-1] = '\0';

    char *rest = buf;
    char *token;
    int field_index = 0;
    
    int16_t temp_alt = 0;
    int valid_fix = 0;

    while((token = strsep(&rest, ",")) != NULL ) {
        size_t len = strlen(token);
        switch (field_index) {
        case 6: 
            if (len > 0) valid_fix = atoi(token);
            break;
        case 9: 
            if (len > 0) temp_alt = (int16_t)strtof(token, NULL);
            break;
        }
        field_index++;
    }

    if (xSemaphoreTake(gps_data_mutex, portMAX_DELAY)) {
        if (valid_fix > 0) {
            _latest_data.altitude_m = temp_alt;
        } else {
            _latest_data.altitude_m = 0;
        }
        xSemaphoreGive(gps_data_mutex);
    }
}

static void _process_nmea_line(const char *line)
{
    if (strncmp(line, "$GNRMC", 6) == 0 || strncmp(line, "$GPRMC", 6) == 0) {
        _parse_rmc(line);
    }
    else if (strncmp(line, "$GNGGA", 6) == 0 || strncmp(line, "$GPGGA", 6) == 0) {
        _parse_gga(line);
    }
}

void configure_neo6m(int uart_num)
{
    ESP_LOGI(TAG, "Configuring u-blox NEO-6M GPS...");

    // 1. 確保原有的 9600 緩衝區清空
    uart_flush_input(uart_num);

    // 2. 切換 5Hz 
    const uint8_t ubx_cfg_rate[] = {
        0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 
        0xC8, 0x00, 0x01, 0x00, 0x00, 0x00, 
        0xDD, 0x46
    };
    uart_write_bytes(uart_num, (const char*)ubx_cfg_rate, sizeof(ubx_cfg_rate));
    uart_wait_tx_done(uart_num, 100 / portTICK_PERIOD_MS); // 等待發送完畢
    ESP_LOGI(TAG, "Update Rate set to 5Hz");
    vTaskDelay(pdMS_TO_TICKS(50)); 

    // 3. 準備切換 115200 Baudrate
    const uint8_t ubx_cfg_prt[] = {
        0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 
        0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 
        0x00, 0xC2, 0x01, 0x00, 0x07, 0x00, 0x03, 0x00, 
        0x00, 0x00, 0x00, 0x00, 
        0xC0, 0x7E
    };
    uart_write_bytes(uart_num, (const char*)ubx_cfg_prt, sizeof(ubx_cfg_prt));
    uart_wait_tx_done(uart_num, 100 / portTICK_PERIOD_MS); // 必須等待指令完全送出
    
    // 立刻切換 ESP32 的 UART
    uart_set_baudrate(uart_num, 115200);
    ESP_LOGI(TAG, "ESP32 UART switched to 115200");
    vTaskDelay(pdMS_TO_TICKS(100)); // 給予模組重啟 UART 的時間

    // 4. 存檔 (現在是在 115200 速度下發送了)
    const uint8_t ubx_cfg_save[] = {
        0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 
        0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 
        0x00, 0x00, 0x00, 0x00, 0x07, 
        0x21, 0xAF  
    };
    uart_write_bytes(uart_num, (const char*)ubx_cfg_save, sizeof(ubx_cfg_save));
    uart_wait_tx_done(uart_num, 100 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "Configuration Saved to NEO-6M Flash/BBR");
}

static void gps_event_task(void *pvParameters)
{
    uart_event_t event;
    uint8_t* dtmp = NULL;
    
    while (dtmp == NULL) {
        dtmp = (uint8_t*) malloc(RD_BUF_SIZE);
        if (dtmp == NULL) {
            ESP_LOGE(TAG, "Malloc failed! Retrying in 1 sec...");
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    for (;;) {
        if (xQueueReceive(uart_queue, (void *)&event, (TickType_t)portMAX_DELAY)) {
            
            size_t buffered_size;
            uart_get_buffered_data_len(g_gps_uart_num, &buffered_size);
            
            if (buffered_size > 1000) {
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
                    if (len > 0) {
                        dtmp[len] = '\0';
                        char *start = strchr((char *)dtmp, '$');
                        if (start != NULL) {
                            _process_nmea_line(start);
                        }
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
}

void gnss_init(int uart_num, int tx_pin, int rx_pin, int baud_rate)
{
    ESP_LOGI(TAG, "Initializing GNSS Parser...");
    
    // 🌟 建立 Mutex
    if (gps_data_mutex == NULL) {
        gps_data_mutex = xSemaphoreCreateMutex();
    }

    g_gps_uart_num = uart_num;
    uart_config_t uart_config = {
        .baud_rate = baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    ESP_ERROR_CHECK(uart_driver_install(uart_num, BUF_SIZE * 2, BUF_SIZE * 2, 20, &uart_queue, 0));
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(uart_num, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    uart_enable_pattern_det_baud_intr(uart_num, '\n', 1, 9, 0, 0);
    uart_pattern_queue_reset(uart_num, 20);

    configure_neo6m(uart_num);

    xTaskCreate(gps_event_task, "gps_task", 4096, NULL, 12, NULL);
    ESP_LOGI(TAG, "GPS Init on UART%d (TX:%d, RX:%d)", uart_num, tx_pin, rx_pin);
}

// 安全讀取函式：使用 Mutex 保護
gps_data_t gnss_get_data(void)
{
    gps_data_t copy = {0};
    if (gps_data_mutex != NULL) {
        if (xSemaphoreTake(gps_data_mutex, portMAX_DELAY)) {
            copy = _latest_data;
            xSemaphoreGive(gps_data_mutex);
        }
    }
    return copy;
}