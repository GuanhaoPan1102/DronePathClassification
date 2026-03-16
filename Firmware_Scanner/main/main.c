#include <stdio.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "sys_defs.h"
#include "drv_ble_scan.h"
#include "drv_uart_tx.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

static const char *TAG = "SCANNER_MAIN";

void app_main(void)
{
    // 1. [核心修改] 系統啟動第一步：初始化 NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    esp_log_level_set(TAG, ESP_LOG_INFO);
    esp_log_level_set("UART_TX", ESP_LOG_INFO);
    esp_log_level_set("BLE_SCANNER", ESP_LOG_INFO);

    ESP_LOGI(TAG, "Scanner Node Starting...");

    // 2. 建立 BLE 和 UART 串接的 Queue (100 包緩衝區非常夠用)
    QueueHandle_t ble_to_uart_queue = xQueueCreate(100, sizeof(ble_packet_queue_item_t));
    if (ble_to_uart_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create UART Queue.");
        return;
    }

    // 3. 初始化組件
    // UART 先啟動，準備好發送
    uart_init(ble_to_uart_queue);
    
    // BLE 後啟動，開始抓取封包
    ble_scan_init(ble_to_uart_queue);

    ESP_LOGI(TAG, "System Ready. Scanner Core 0 <-> UART Core 1.");

    // 4. 讓 app_main 保持運作，作為心跳監控
    while (1) {
        // 可以定期在這裡印出 Queue 的使用狀況，監控有沒有塞車
        // uxQueueMessagesWaiting(ble_to_uart_queue); 
        vTaskDelay(pdMS_TO_TICKS(10000)); // 每 10 秒跳一下
        ESP_LOGD(TAG, "Heartbeat: Scanner is alive.");
    }
}