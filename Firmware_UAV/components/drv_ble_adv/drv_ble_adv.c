#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>

#include "sys_defs.h"
#include "drv_ble_adv.h"
#include "drv_gps_read.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_defs.h"
#include "esp_gap_ble_api.h"

static const char *TAG = "BLE_BROADCAST";

static bool g_is_advertising = false;   // 紀錄是否已經開始廣播

// 廣播參數 (Interval = 100ms)
// 0x00A0 = 160 * 0.625ms = 100ms
static esp_ble_adv_params_t ble_adv_params = {
    .adv_int_min        = 0x00A0,
    .adv_int_max        = 0x00A0,
    .adv_type           = ADV_TYPE_IND,            // 通用廣播 (可被掃描)
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy  = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

// 廣播資料緩衝區
// 這是真正存放我們要發送出去的資料的地方 (15 Bytes)
// 它會被映射到 Manufacturer Data 欄位中
static uint8_t manu_data[BLE_PAYLOAD_SIZE]; 

// 廣播資料結構設定
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = false,             // 廣播時是否包含裝置名稱
    .include_txpower = false,
    .min_interval = 0x00A0, 
    .max_interval = 0x00A0, 
    .appearance = 0x00,
    .manufacturer_len = sizeof(manu_data), 
    .p_manufacturer_data = manu_data,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = 0,
    .p_service_uuid = NULL,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static void _pack_telemetry(gps_data_t *gps)
{
    uav_ble_payload_t *payload = (uav_ble_payload_t *)manu_data;

    payload->identifier = BLE_PACKET_IDENTIFIER;

    static uint8_t seq_counter = 0;
    payload->seq_num = seq_counter++;
    
    payload->lat = gps->latitude_scaled;
    payload->lon = gps->longitude_scaled;
    payload->alt = gps->altitude_m;
    payload->spd = gps->speed_kph_scaled;

    // 從 header 開始 XOR 到倒數第二個 byte (checksum 欄位本身除外)
    uint8_t xor_sum = 0;
    for (int i = 0; i < BLE_PAYLOAD_SIZE - 1; i++) {
        xor_sum ^= manu_data[i];
    }
    payload->checksum = xor_sum;
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event)
    {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        if (!g_is_advertising) {
            esp_ble_gap_start_advertising(&ble_adv_params);
        }
        break;
    
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(TAG, "Advertising start failed");
        } else {
            // 標記為「已啟動」
            g_is_advertising = true;
            ESP_LOGI(TAG, "Advertising Started Successfully");
        }
        break;

    default:
        break;
    }
}

static void ble_broadcast_task(void *pvParameters)
{
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();

    // 定義週期 (100ms)
    const TickType_t xFrequency = pdMS_TO_TICKS(100);

    while(1) {
        gps_data_t current_gps = gnss_get_data();

        _pack_telemetry(&current_gps);

        esp_err_t err = esp_ble_gap_config_adv_data(&adv_data);

        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Config adv data failed: %d", err);
        }

        // D. 等待 100ms (10Hz)
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void ble_adv_init(void)
{
    ESP_LOGI(TAG, "Initializing BLE Broadcaster...");

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));
    
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_event_handler)); 

    xTaskCreate(ble_broadcast_task, "ble_tx_task", 4096, NULL, 10, NULL);
    
    ESP_LOGI(TAG, "BLE Init Done. Broadcasting task started.");
}
