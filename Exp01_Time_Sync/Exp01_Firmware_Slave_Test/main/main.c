#include <stdio.h>
#include <sys/time.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"

// 引入你寫好的 GPS 驅動
#include "drv_gnss_sync.h"

static const char *TAG = "TEST_SLAVE";

void app_main(void)
{
    // 基礎初始化
    ESP_ERROR_CHECK(nvs_flash_init());

    // 初始化 GPS 與 PPS 中斷
    ESP_LOGI(TAG, "Initializing GNSS Hardware & PPS...");
    gnss_sync_init();

    ESP_LOGI(TAG, "Waiting for 3D Fix and PPS Sync (Make sure antenna is facing the sky)...");

    // 阻擋迴圈：嚴格等待定位與時間收斂
    while (1) {
        gps_fix_t fix = gnss_get_fix(); 
        
        if (fix.is_fixed && fix.is_time_synced) {
            ESP_LOGI(TAG, "========================================");
            ESP_LOGI(TAG, "SLAVE GPS LOCKED & TIME SYNCED!");
            ESP_LOGI(TAG, "Lat: %ld, Lon: %ld, Alt: %.2f m", fix.latitude, fix.longitude, fix.altitude);
            ESP_LOGI(TAG, "========================================");
            break; 
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // 成功對時後，進入無情印時間迴圈 (每秒印一次)
    while(1) {
        struct timeval tv;
        gettimeofday(&tv, NULL); // 抓取當下絕對系統時間

        // 轉換為 UTC+8 以利人類閱讀
        time_t local_time = tv.tv_sec + (8 * 3600);
        struct tm ti;
        gmtime_r(&local_time, &ti);

        // 關鍵：把微秒 (tv_usec) 印出來，觀察它是不是穩穩地跳動
        ESP_LOGI(TAG, "[Slave Clock] %04d-%02d-%02d %02d:%02d:%02d.%06ld", 
                 ti.tm_year + 1900, ti.tm_mon + 1, ti.tm_mday, 
                 ti.tm_hour, ti.tm_min, ti.tm_sec, 
                 tv.tv_usec);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}