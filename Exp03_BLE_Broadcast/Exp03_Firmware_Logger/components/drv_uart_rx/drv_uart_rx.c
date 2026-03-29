#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"

#include "sys_defs.h"
#include "drv_uart_rx.h"

#define UART_PORT_NUM      UART_NUM_1
#define UART_TX_PIN        27
#define UART_RX_PIN        22
#define UART_RTS_PIN       33   
#define UART_CTS_PIN       25   
#define UART_BAUD_RATE     921600
#define UART_BUF_SIZE      1024

static const char *TAG = "UART_RECV";

static void uart_recv_task(void *pvParameters) {
    QueueHandle_t out_queue = (QueueHandle_t)pvParameters;
    
    // 一次最多抓取 128 Bytes，大幅減少 FreeRTOS context switch
    uint8_t rx_buf[128]; 
    int state = 0;
    
    ble_packet_queue_item_t temp_item;
    uint8_t *item_ptr = (uint8_t *)&temp_item;
    int item_idx = 0;

    ESP_LOGI(TAG, "UART Recv Task Started on Core 1 (Batch Mode). Waiting for BLE data...");

    while (1) {
        // 1. 批次讀取：最多等待 10ms，把硬體 FIFO 裡的資料一把抓出來
        int len = uart_read_bytes(UART_PORT_NUM, rx_buf, sizeof(rx_buf), pdMS_TO_TICKS(10));
        
        // 2. 記憶體內狀態機：在 RAM 裡面極速處理每一 Byte
        if (len > 0) {
            for (int i = 0; i < len; i++) {
                uint8_t rx_byte = rx_buf[i];
                
                switch (state) {
                    case 0: // 尋找 Header 0
                        if (rx_byte == UART_HEADER_0) state = 1;
                        break;
                        
                    case 1: // 尋找 Header 1
                        if (rx_byte == UART_HEADER_1) {
                            state = 2; 
                            item_idx = 0; // 準備接收 Payload
                        } else if (rx_byte == UART_HEADER_0) {
                            state = 1; // 處理連續的 0xEB 0xEB 情況
                        } else {
                            state = 0;
                        }
                        break;
                        
                    case 2: // 接收 Data Body
                        item_ptr[item_idx++] = rx_byte;
                        if (item_idx >= sizeof(ble_packet_queue_item_t)) {
                            state = 3; // 進入 Checksum 階段
                        }
                        break;
                        
                    case 3: { // 接收並驗證 Checksum
                        uint8_t checksum_recv = rx_byte;
                        uint8_t checksum_calc = 0;
                        
                        // 計算 XOR Checksum
                        for (int k = 0; k < sizeof(ble_packet_queue_item_t); k++) {
                            checksum_calc ^= item_ptr[k];
                        }

                        if (checksum_calc == checksum_recv) {
                            // 驗證成功瞬間，立刻抓取絕對時間戳
                            struct timeval tv;
                            gettimeofday(&tv, NULL);
                            
                            ttgo_log_item_t log_item;
                            log_item.timestamp_us = ((int64_t)tv.tv_sec * 1000000LL) + tv.tv_usec;
                            memcpy(&log_item.ble_data, &temp_item, sizeof(ble_packet_queue_item_t));

                            // 非阻塞丟入 Queue，交給 SD 卡任務慢慢寫
                            if (xQueueSend(out_queue, &log_item, 0) != pdTRUE) {
                                ESP_LOGW(TAG, "SD Card Queue Full! Dropping packet.");
                            }
                        } else {
                            ESP_LOGW(TAG, "B2B Checksum Error! Expected: %02X, Got: %02X", checksum_calc, checksum_recv);
                        }
                        
                        state = 0; // 重新等待下一個封包
                        break;
                    }
                }
            }
        }
    }
}

void uart_init(QueueHandle_t data_queue)
{
    const uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        // 若確定硬體 4 根線都有接妥，維持 CTS_RTS；若只接 TX/RX，請改為 UART_HW_FLOWCTRL_DISABLE
        .flow_ctrl = UART_HW_FLOWCTRL_CTS_RTS, 
        .rx_flow_ctrl_thresh = 122, 
        .source_clk = UART_SCLK_APB,
    };

    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, UART_TX_PIN, UART_RX_PIN, UART_RTS_PIN, UART_CTS_PIN));
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, UART_BUF_SIZE, 0, 0, NULL, 0));

    ESP_LOGI(TAG, "UART Initialized (921600 baud, CTS/RTS enabled)");

    xTaskCreatePinnedToCore(uart_recv_task, "uart_rx_task", 4096, (void *)data_queue, 5, NULL, 1);
}