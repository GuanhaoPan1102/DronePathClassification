#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#include "sys_defs.h"
#include "drv_ble_scan.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_log.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_defs.h"
#include "esp_gap_ble_api.h"

static const char *TAG = "BLE_SCANNER";
static QueueHandle_t g_ble_data_queue = NULL;

static esp_ble_scan_params_t ble_scan_params = {
    .scan_type              = BLE_SCAN_TYPE_PASSIVE,
    .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval          = 0x50,
    .scan_window            = 0x50,
    .scan_duplicate         = BLE_SCAN_DUPLICATE_DISABLE
};

static bool _verify_checksum(uav_ble_payload_t *data)
{
    uint8_t *raw_ptr = (uint8_t *)data;
    uint8_t xor_sum = 0;
    
    // 計算範圍：從頭開始，直到倒數第2個byte (不包含 checksum 本身)
    for (int i = 0; i < sizeof(uav_ble_payload_t) - 1; i++) {
        xor_sum ^= raw_ptr[i];
    }

    // 比對計算結果與封包內的 checksum
    return (xor_sum == data->checksum);
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    esp_err_t err;

    switch (event) {

    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT: {
        // the unit of the duration is second, 0 means scan permanently
        uint32_t duration = 0;
        esp_ble_gap_start_scanning(duration);
        break;
    }

    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
        //scan start complete event to indicate scan start successfully or failed
        if ((err = param->scan_start_cmpl.status) != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(TAG, "Scanning start failed, error %s", esp_err_to_name(err));
        } else {
            ESP_LOGI(TAG, "Scanning start successfully");
        }
        break;

    case ESP_GAP_BLE_SCAN_RESULT_EVT: {
        esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;
        switch (scan_result->scan_rst.search_evt) {
        case ESP_GAP_SEARCH_INQ_RES_EVT:
            uint8_t adv_data_len = 0;
            uint8_t *adv_data = esp_ble_resolve_adv_data(scan_result->scan_rst.ble_adv,
                                                         ESP_BLE_AD_MANUFACTURER_SPECIFIC_TYPE, 
                                                         &adv_data_len);
            if (adv_data != NULL && adv_data_len == sizeof(uav_ble_payload_t)) {
                uav_ble_payload_t *uav_packet = (uav_ble_payload_t *)adv_data;
                if (uav_packet->identifier == BLE_PACKET_IDENTIFIER) {
                    if (_verify_checksum(uav_packet)) {
                        
                        ble_packet_queue_item_t item;
                
                        // 1. 複製資料 (Memcpy)
                        memcpy(&item.payload, uav_packet, sizeof(uav_ble_payload_t));
                        item.rssi = scan_result->scan_rst.rssi;
                        // 2. 丟進 Queue (因為在 Callback 中，必須用 FromISR)
                        if (g_ble_data_queue != NULL) {
                            xQueueSendFromISR(g_ble_data_queue, &item, NULL);
                        }

                    } else {
                        ESP_LOGW(TAG, "Checksum Error!");
                    }
                }
            }
            break;
        default:
            break;
        }
        break;
    }

    case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
        if ((err = param->scan_stop_cmpl.status) != ESP_BT_STATUS_SUCCESS){
            ESP_LOGE(TAG, "Scanning stop failed, error %s", esp_err_to_name(err));
        }
        else {
            ESP_LOGI(TAG, "Scanning stop successfully");
        }
        break;

    default:
        break;
    }
}

void ble_scan_init(QueueHandle_t data_queue)
{
    ESP_LOGI(TAG, "Initializing BLE Scanner...");

    // 串聯 BLE 和 UART 的 Queue
    g_ble_data_queue = data_queue;

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));
    
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_event_handler));

    esp_ble_gap_set_scan_params(&ble_scan_params);

}
