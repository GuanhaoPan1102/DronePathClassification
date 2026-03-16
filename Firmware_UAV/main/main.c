#include <stdio.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/uart.h"
#include "sys_defs.h"
#include "drv_gps_read.h"
#include "drv_ble_adv.h"

#define GPS_UART_NUM UART_NUM_1
#define GPS_TX_PIN   5
#define GPS_RX_PIN   4
#define GPS_BAUD     9600

static const char *TAG = "MAIN";

// 保持你的 _test_gps 邏輯不變，但在輸出時做點微調
void _test_gps(void)
{
    gps_data_t gps = gnss_get_data();

    if (gps.is_valid) {
        ESP_LOGD(TAG, " [GPS FIX] Lat:%ld.%07ld, Lon:%ld.%07ld, Spd:%d.%02d km/h, Alt:%d m", 
                gps.latitude_scaled / 10000000, abs(gps.latitude_scaled % 10000000),
                gps.longitude_scaled / 10000000, abs(gps.longitude_scaled % 10000000),
                gps.speed_kph_scaled / 100, abs(gps.speed_kph_scaled % 100),
                gps.altitude_m);
    } else {
        ESP_LOGW(TAG, " [GPS Searching...] Wait for satellite fix...");
    }
}

void app_main(void)
{
    // 1. [關鍵修改] 統一在入口處初始化 NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 2. 設定 Log Level
    esp_log_level_set(TAG, ESP_LOG_INFO);
    esp_log_level_set("GNSS", ESP_LOG_INFO);
    esp_log_level_set("BLE_BROADCAST", ESP_LOG_INFO);
    
    // 3. 初始化 GPS
    gnss_init(GPS_UART_NUM, GPS_TX_PIN, GPS_RX_PIN, GPS_BAUD);
    
    // 4. 初始化 BLE
    ble_adv_init();

    ESP_LOGI(TAG, "==========================================");
    ESP_LOGI(TAG, " UAV System Started");
    ESP_LOGI(TAG, " GNSS Rate: 5Hz, BLE Rate: 10Hz");
    ESP_LOGI(TAG, "==========================================");

    while (1) {
        // [DEBUG] 定期檢查 GPS 狀態並印出
        // _test_gps();
        
        // 每 5 秒監控一次心跳
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}