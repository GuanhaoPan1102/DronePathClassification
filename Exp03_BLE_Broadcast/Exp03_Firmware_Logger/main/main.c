#include <stdio.h>
#include "esp_log.h"
#include "driver/uart.h"
#include "sys_defs.h"

#include <time.h>
#include <sys/time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "drv_gnss_sync.h"
#include "drv_uart_rx.h"
#include "drv_sd_card.h"

static const char *TAG = "MAIN";

void app_main(void)
{
    QueueHandle_t recv_queue = xQueueCreate(50, sizeof(ttgo_log_item_t));

    gnss_sync_init(); 

    ESP_LOGI(TAG, "Waiting for System Ready (Position Fix & Time Sync)...");

    while (1) {
        gps_fix_t status = gnss_get_fix();

        // 嚴格條件：位置鎖定且時間同步完成
        if (status.is_fixed && status.is_time_synced) {
            
            setenv("TZ", "CST-8", 1);
            tzset();

            time_t now;
            struct tm timeinfo;
            time(&now);
            localtime_r(&now, &timeinfo);

            char time_str[64];
            strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", &timeinfo);

            ESP_LOGI(TAG, "============================================");
            ESP_LOGI(TAG, " TTGO System Ready & Synced!");
            ESP_LOGI(TAG, " [Position] Lat: %d, Lon: %d, Alt: %.2f m", 
                     status.latitude, status.longitude, status.altitude);
            ESP_LOGI(TAG, " [Time]     %s (Taiwan Time)", time_str);
            ESP_LOGI(TAG, "============================================");
            
            break; 
        }

        if (!status.is_fixed) {
            ESP_LOGI(TAG, "Wait: Surveying Position... (Need 60 samples)");
        } else if (!status.is_time_synced) {
            ESP_LOGI(TAG, "Wait: Position Fixed. Waiting for next PPS pulse...");
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // 當時間軸與真實世界對齊後，才初始化 SD 卡與 UART 接收器
    ESP_LOGI(TAG, "Step 2: Initializing SD Card & UART B2B...");
    
    sd_card_init(recv_queue);
    uart_init(recv_queue);
}