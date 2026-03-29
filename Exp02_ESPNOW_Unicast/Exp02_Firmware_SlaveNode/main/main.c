#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_random.h"

// 引入我們剛改好的核心標頭檔與 GPS 驅動
#include "espnow_experiment_defs.h"
#include "drv_gnss_sync.h"

// ==========================================
// 燒錄前必改：設定這塊板子的編號與發射頻率
// ==========================================
#define MY_NODE_ID "S1" 

// 發射頻率 Macro：1 代表 1Hz (每秒1包)，10 代表 10Hz
#define TX_FREQ_HZ 1    

static const char *TAG = "EXP02_SLAVE";

// 填入 Master 的真實 MAC 位址 (Unicast 單播目標)
static uint8_t master_mac[6] = {0x2C, 0xBC, 0xBB, 0xA8, 0x4B, 0x5C}; 

void app_main(void)
{
    // ---------------------------------------------------------
    // 1. 基礎環境與 Wi-Fi 初始化
    // ---------------------------------------------------------
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR)); // LR 模式確保遠距離傳輸
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE));

    // ---------------------------------------------------------
    // 2. 初始化 GPS 並等待絕對時間同步 (繼承 Exp01 的心血)
    // ---------------------------------------------------------
    gnss_sync_init();
    ESP_LOGI(TAG, "Node %s: Waiting for GPS Fix and PPS Sync...", MY_NODE_ID);
    
    while (1) {
        gps_fix_t fix = gnss_get_fix(); 
        
        if (fix.is_fixed && fix.is_time_synced) {
            ESP_LOGI(TAG, "GPS Time Synced! Ready to start Exp02.");
            break; 
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // ---------------------------------------------------------
    // 3. ESP-NOW 初始化與註冊 Peer
    // ---------------------------------------------------------
    ESP_ERROR_CHECK(esp_now_init());
    esp_now_peer_info_t peer = {.channel = 0, .encrypt = false};
    memcpy(peer.peer_addr, master_mac, 6);
    ESP_ERROR_CHECK(esp_now_add_peer(&peer)); // 註冊 Master 為單播目標

    ESP_LOGI(TAG, "Slave %s Ready. Target Master: %02X:%02X:%02X:%02X:%02X:%02X", 
             MY_NODE_ID, master_mac[0], master_mac[1], master_mac[2], 
             master_mac[3], master_mac[4], master_mac[5]);
    ESP_LOGI(TAG, "TX Frequency set to: %d Hz", TX_FREQ_HZ);

    // ---------------------------------------------------------
    // 4. 壓力發射與延遲測試迴圈
    // ---------------------------------------------------------
    uav_data_pkt_t tx_pkt;
    memset(&tx_pkt, 0, sizeof(uav_data_pkt_t));
    tx_pkt.type = MSG_TYPE_UAV_DUMMY;
    strncpy(tx_pkt.node_id, MY_NODE_ID, 4);
    tx_pkt.record_count = MAX_BLE_RECORDS_PER_PACKET; // 固定填 10 筆

    uint32_t current_seq = 1;

    // 計算週期的 Tick 數 (例如 1Hz = 1000ms)
    const TickType_t xFrequency = pdMS_TO_TICKS(1000 / TX_FREQ_HZ);
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1) {
        // 嚴格等待至下一個週期點
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        tx_pkt.seq_num = current_seq++;

        // 填充 10 筆假資料，讓封包大小符合真實戰況
        for (int i = 0; i < MAX_BLE_RECORDS_PER_PACKET; i++) {
            struct timeval dummy_tv;
            gettimeofday(&dummy_tv, NULL);
            tx_pkt.records[i].timestamp_us = (int64_t)dummy_tv.tv_sec * 1000000LL + dummy_tv.tv_usec;
            
            tx_pkt.records[i].mac[0] = 0xAA;
            tx_pkt.records[i].mac[1] = 0xBB;
            tx_pkt.records[i].mac[2] = 0xCC;
            tx_pkt.records[i].mac[3] = 0xDD;
            tx_pkt.records[i].mac[4] = 0xEE;
            tx_pkt.records[i].mac[5] = (uint8_t)(esp_random() % 256);
            tx_pkt.records[i].rssi = -40 - (int8_t)(esp_random() % 50);
        }

        // 抓取發射前一瞬間的絕對時間，用來算傳輸延遲
        struct timeval tv_send;
        gettimeofday(&tv_send, NULL);
        tx_pkt.send_timestamp_us = (int64_t)tv_send.tv_sec * 1000000LL + tv_send.tv_usec;

        // 單播發送給 Master
        esp_err_t err = esp_now_send(master_mac, (uint8_t *)&tx_pkt, sizeof(tx_pkt));
        
        if (err == ESP_OK) {
            // 每 10 包印一次 Log，避免洗版
            if (current_seq % 10 == 0) {
                ESP_LOGI(TAG, "Sent Seq: %lu", tx_pkt.seq_num); 
            }
        } else {
            ESP_LOGE(TAG, "Send Failed! Seq: %lu, Err: %d", tx_pkt.seq_num, err);
        }
    }
}