#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"

#include "sys_defs.h"
#include "drv_uart_tx.h"

#define UART_PORT_NUM      UART_NUM_1
#define UART_TX_PIN        22
#define UART_RX_PIN        16
#define UART_BAUD_RATE     921600
#define UART_BUF_SIZE      1024

static const char *TAG = "UART_TX";

/**
 * @brief 計算 UART 相互通訊的 Checksum
 * 計算包含整個 ble_packet_queue_item_t 的所有內容
 */
static uint8_t _calculate_b2b_checksum(const uint8_t *data, size_t len) {
    uint8_t checksum = 0;
    for (size_t i = 0; i < len; i++) {
        checksum ^= data[i];
    }
    return checksum;
}

static void uart_send_task(void *pvParameters) {
    QueueHandle_t data_queue = (QueueHandle_t)pvParameters;
    ble_packet_queue_item_t item; 
    
    // 定義傳送緩衝區：Header(2) + Data(sizeof(item)) + Checksum(1)
    const size_t frame_size = 2 + sizeof(ble_packet_queue_item_t) + 1;
    uint8_t tx_frame[frame_size];

    ESP_LOGI(TAG, "UART Send Task Started on Core 1");

    while (1) {
        // 阻塞式等待 Scanner 掃描到的資料
        if (xQueueReceive(data_queue, &item, portMAX_DELAY) == pdTRUE) {
            
            // 1. 填入 Frame Header
            tx_frame[0] = UART_HEADER_0; // 0xEB
            tx_frame[1] = UART_HEADER_1; // 0x90

            // 2. COPY 資料主體 (Payload + RSSI)
            memcpy(&tx_frame[2], &item, sizeof(item));

            // 3. 計算板間 Checksum (只針對 Data 部分進行 XOR)
            tx_frame[frame_size - 1] = _calculate_b2b_checksum((uint8_t*)&item, sizeof(item));

            // 4. 一次性發送完整封包
            int len = uart_write_bytes(UART_PORT_NUM, (const char*)tx_frame, frame_size);
            
            if (len < 0) {
                ESP_LOGE(TAG, "UART write failed");
            }
        }
    }
}

void uart_init(QueueHandle_t data_queue)
{
    // UART 配置保持不變，但為了實驗室穩定性，若沒接線可暫時關閉 Flow Control
    const uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, // 先改為 Disable，除非你確定四根線都接了
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, UART_TX_PIN, UART_RX_PIN, -1, -1));
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, UART_BUF_SIZE, 0, 0, NULL, 0));

    xTaskCreatePinnedToCore(uart_send_task, "uart_tx_task", 4096, (void *)data_queue, 10, NULL, 1);
    ESP_LOGI(TAG, "B2B UART Sender Initialized (921600 baud)");
}