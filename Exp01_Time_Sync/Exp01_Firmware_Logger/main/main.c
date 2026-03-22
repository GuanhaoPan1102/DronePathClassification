#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_timer.h"
#include "esp_random.h"
#include "driver/uart.h"

#include "sync_experiment_defs.h"
#include "drv_gnss_sync.h" // 引入你提供的 GPS 同步驅動

// ==========================================
// 🚀 燒錄前必改：設定這塊板子的編號 ("S1" ~ "S4")
// ==========================================
#define MY_NODE_ID "S2" 

static const char *TAG = "SYNC_SLAVE";
static uint8_t master_mac[6] = {0x2C, 0xBC, 0xBB, 0xA8, 0x4B, 0x5C}; // 建議改為 Master 實際 MAC

/**
 * @brief ESP-NOW 接收回調：同步實驗核心
 */
static void on_data_recv(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
    if (len != sizeof(sync_pkt_t)) return;
    sync_pkt_t *recv_pkt = (sync_pkt_t *)data;

    if (recv_pkt->type == MSG_TYPE_SYNC) {
        // 1. 收到觸發的瞬間，立刻抓取「真實系統時間」
        struct timeval tv;
        gettimeofday(&tv, NULL);

        // 2. 打包回傳
        sync_pkt_t report_pkt;
        report_pkt.type = MSG_TYPE_REPORT;
        report_pkt.seq_num = recv_pkt->seq_num;
        strncpy(report_pkt.node_id, MY_NODE_ID, 4);
        
        // 填入秒與微秒
        report_pkt.tv_sec = (int64_t)tv.tv_sec;
        report_pkt.tv_usec = tv.tv_usec;

        // 隨機避讓後回傳 (0~10ms)
        vTaskDelay(pdMS_TO_TICKS(esp_random() % 10));
        esp_now_send(recv_info->src_addr, (uint8_t *)&report_pkt, sizeof(report_pkt));
    }
}

void app_main(void)
{
    // 1. 初始化基礎環境
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    // Wi-Fi LR 模式設定
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR)); 
    ESP_ERROR_CHECK(esp_wifi_start());

    // 2. 初始化 GPS 同步驅動
    gnss_sync_init();

    // 3. 初始化 ESP-NOW
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(on_data_recv));
    esp_now_peer_info_t peer = {.channel = 0, .encrypt = false};
    memcpy(peer.peer_addr, master_mac, 6);
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));

    // 4. 等待 GPS 定位鎖定與時間同步完成
    ESP_LOGI(TAG, "Node %s: Waiting for GPS Fix and PPS Sync...", MY_NODE_ID);
    
    while (1) {
        gps_fix_t fix = gnss_get_fix(); // 取得目前 GPS 狀態
        
        if (fix.is_fixed && fix.is_time_synced) {
            ESP_LOGI(TAG, "GPS Fixed! Lat:%d, Lon:%d, Time Synced via PPS.", fix.latitude, fix.longitude);
            break; 
        }
        
        // 每秒檢查一次狀態
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // 5. 發送 READY 給 Master
    sync_pkt_t ready_pkt = {
        .type = MSG_TYPE_READY,
        .seq_num = 0,
        .tv_sec = 0,   // 改為秒
        .tv_usec = 0   // 改為微秒
    };
    strncpy(ready_pkt.node_id, MY_NODE_ID, 4);
    
    ESP_LOGI(TAG, "Localization complete. Reporting READY to Master.");
    
    // 持續發送直到 Master 收到 (或實驗開始)
    for(int i=0; i<5; i++) {
        esp_now_send(master_mac, (uint8_t *)&ready_pkt, sizeof(ready_pkt));
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    ESP_LOGI(TAG, "Slave %s is standing by for Sync pulses...", MY_NODE_ID);
    
    while(1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}