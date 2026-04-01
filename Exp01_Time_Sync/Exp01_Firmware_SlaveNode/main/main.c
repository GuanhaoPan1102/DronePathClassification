#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>
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
#include "drv_gnss_sync.h" 

// ==========================================
// 燒錄前必改：設定這塊板子的編號 ("S1", "S2", "S3")
// ==========================================
#define MY_NODE_ID "S1" 

static const char *TAG = "SYNC_SLAVE";

// 填入獨立廣播端 (Trigger) MCU 的 MAC 位址
static uint8_t trigger_mac[6] = {0x2C, 0xBC, 0xBB, 0xA8, 0x4B, 0x5C};

/**
 * @brief ESP-NOW 接收回調：處理同步與點名要求
 */
static void on_data_recv(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
    if (len != sizeof(sync_pkt_t)) return;
    sync_pkt_t *recv_pkt = (sync_pkt_t *)data;

    // 1. 處理高頻率的同步觸發訊號 (MSG_TYPE_SYNC)
    if (recv_pkt->type == MSG_TYPE_SYNC) {
        struct timeval tv;
        gettimeofday(&tv, NULL); // 抓取被 PPS 完美校正過的絕對時間

        sync_pkt_t report_pkt;
        memset(&report_pkt, 0, sizeof(sync_pkt_t));
        report_pkt.type = MSG_TYPE_REPORT;
        report_pkt.seq_num = recv_pkt->seq_num;
        strncpy(report_pkt.node_id, MY_NODE_ID, 4);
        report_pkt.tv_sec = (int64_t)tv.tv_sec;
        report_pkt.tv_usec = tv.tv_usec;

        // 隨機避讓 (0~10ms) 避免資料回傳空中撞包
        vTaskDelay(pdMS_TO_TICKS(esp_random() % 10));
        esp_now_send(recv_info->src_addr, (uint8_t *)&report_pkt, sizeof(report_pkt));
    }
    // 2. 處理 Trigger 批次實驗前的點名要求 (MSG_TYPE_POLL)
    else if (recv_pkt->type == MSG_TYPE_POLL) {
        gps_fix_t fix = gnss_get_fix(); 
        
        // 只有在 GPS 完美鎖定且 PPS 已經同步的情況下，才大聲回答 READY
        if (fix.is_fixed && fix.is_time_synced) {
            sync_pkt_t ready_pkt;
            memset(&ready_pkt, 0, sizeof(sync_pkt_t));
            ready_pkt.type = MSG_TYPE_READY;
            strncpy(ready_pkt.node_id, MY_NODE_ID, 4);
            
            // 隨機避讓 (0~50ms)，避免 4 台節點同時回覆導致 2.4GHz 碰撞
            vTaskDelay(pdMS_TO_TICKS(esp_random() % 50)); 
            esp_now_send(recv_info->src_addr, (uint8_t *)&ready_pkt, sizeof(ready_pkt));
            ESP_LOGI(TAG, "[POLL] Answered READY to Trigger.");
        } else {
            ESP_LOGW(TAG, "[POLL] Received poll, but GPS is NOT synced. Ignoring.");
        }
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
    ESP_ERROR_CHECK(esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE));

    // 2. 初始化 GPS 同步驅動
    gnss_sync_init();

    // 3. 初始化 ESP-NOW
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(on_data_recv));
    esp_now_peer_info_t peer = {.channel = 0, .encrypt = false};
    memcpy(peer.peer_addr, trigger_mac, 6);
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));

    ESP_LOGI(TAG, "Initialization Complete. Entering Idle Monitoring Mode...");

    // 4. 純狀態監控迴圈 (不再主動發送任何 ESP-NOW 封包)
    while (1) {
        gps_fix_t fix = gnss_get_fix(); 
        
        if (fix.is_fixed && fix.is_time_synced) {
            ESP_LOGI(TAG, "Status: [LOCKED] Lat: %d, Lon: %d | Standing by for POLL/SYNC", 
                     fix.latitude, fix.longitude);
        } else {
            ESP_LOGW(TAG, "Status: [SEARCHING] Waiting for 3D Fix or PPS Pulse...");
        }

        // 每 2 秒印一次狀態心跳包
        vTaskDelay(pdMS_TO_TICKS(2000)); 
    }
}