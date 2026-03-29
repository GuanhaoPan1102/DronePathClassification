#include <stdio.h>
#include <sys/time.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/gpio.h"

// 引入 4G/GNSS 驅動
#include "drv_4g_gnss_sync.h"

static const char *TAG = "TEST_MASTER";

void app_main(void)
{
    // 電源腳位初始化 (確保 4G 模組順利開機)
    gpio_reset_pin(21);
    gpio_set_direction(21, GPIO_MODE_OUTPUT);

    // 基礎初始化
    ESP_ERROR_CHECK(nvs_flash_init());

    // 啟動 4G 模組的 GNSS 引擎
    ESP_LOGI(TAG, "Initializing 4G/GNSS Software Sync Engine...");
    ESP_ERROR_CHECK(drv_4g_init());
    drv_4g_gnss_power(true);
    drv_4g_start_nmea_stream();

    ESP_LOGI(TAG, "Waiting for 3D Fix and NMEA Edge Sync (Make sure antenna is facing the sky)...");

    // 阻擋迴圈：嚴格等待定位與時間收斂
    while (1) {
        gps_fix_t fix = drv_4g_gnss_get_fix();
        
        if (fix.is_fixed && fix.is_time_synced) {
            ESP_LOGI(TAG, "========================================");
            ESP_LOGI(TAG, "MASTER GPS LOCKED & TIME SYNCED!");
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

        // 關鍵：把微秒 (tv_usec) 印出來
        ESP_LOGI(TAG, "[Master Clock] %04d-%02d-%02d %02d:%02d:%02d.%06ld", 
                 ti.tm_year + 1900, ti.tm_mon + 1, ti.tm_mday, 
                 ti.tm_hour, ti.tm_min, ti.tm_sec, 
                 tv.tv_usec);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}