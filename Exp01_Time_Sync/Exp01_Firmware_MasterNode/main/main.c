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
#include "esp_random.h"
#include "driver/gpio.h"

// 引入自定義的標頭檔
#include "sync_experiment_defs.h" 
#include "drv_4g_gnss_sync.h"      

// ==========================================
// 實驗設定：Master 節點代號
// ==========================================
#define MY_NODE_ID "M1"

static const char *TAG = "SYNC_EXP01_MASTER";

// 填入獨立廣播端 (Trigger) MCU 的 MAC 位址
static uint8_t broadcaster_mac[6] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF}; 

// ==========================================
// ESP-NOW 接收回調 (核心測試邏輯)
// ==========================================
/**
 * @brief 當收到廣播端發出的同步觸發訊號時，瞬間抓取系統時間並回傳
 */
static void on_data_recv(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
    if (len != sizeof(sync_pkt_t)) return;
    sync_pkt_t *recv_pkt = (sync_pkt_t *)data;

    // 1. 確認是廣播端的同步觸發封包
    if (recv_pkt->type == MSG_TYPE_SYNC) {
        
        struct timeval tv;
        gettimeofday(&tv, NULL);

        time_t local_time = tv.tv_sec + (8 * 3600);
        struct tm ti;
        gmtime_r(&local_time, &ti);
        ESP_LOGI(TAG, "TRIGGER RECEIVED! Seq: %d | Time: %02d:%02d:%02d.%06ld", 
                 recv_pkt->seq_num, ti.tm_hour, ti.tm_min, ti.tm_sec, tv.tv_usec);

        // 2. 打包回傳封包給廣播端 (或是指定的紀錄端)
        sync_pkt_t report_pkt;
        memset(&report_pkt, 0, sizeof(sync_pkt_t));
        report_pkt.type = MSG_TYPE_REPORT;
        report_pkt.seq_num = recv_pkt->seq_num;
        strncpy(report_pkt.node_id, MY_NODE_ID, 4);
        
        report_pkt.tv_sec = (int64_t)tv.tv_sec;
        report_pkt.tv_usec = (int32_t)tv.tv_usec;

        // 3. 隨機避讓 (Random Backoff) 0~10ms，避免 Master 與 Slave 同時發送造成 2.4GHz 碰撞
        vTaskDelay(pdMS_TO_TICKS(esp_random() % 10));
        
        // 發送報告
        esp_now_send(recv_info->src_addr, (uint8_t *)&report_pkt, sizeof(report_pkt));
    }
}

// ==========================================
// 主程式
// ==========================================
void app_main(void)
{
    gpio_reset_pin(21);
    gpio_set_direction(21, GPIO_MODE_OUTPUT);

    // ==========================================
    // 階段一：基礎環境初始化 (NVS, WiFi, ESP-NOW)
    // ==========================================
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR)); // 使用 Long Range 模式增加穿透力
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(on_data_recv));
    
    // 註冊廣播端為 Peer
    esp_now_peer_info_t peer = {.channel = 0, .encrypt = false};
    memcpy(peer.peer_addr, broadcaster_mac, 6);
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));

    // ==========================================
    // 階段二：初始化 Master 專用的 4G/GNSS 驅動
    // ==========================================
    ESP_LOGI(TAG, "Initializing 4G/GNSS Sync Engine...");
    ESP_ERROR_CHECK(drv_4g_init());

    // 啟動 GPS 並進入校準模式 (NMEA ON)
    drv_4g_gnss_power(true);
    drv_4g_start_nmea_stream();

    ESP_LOGI(TAG, "Survey-In started. Waiting for 3D Fix and PPS Lock...");

    // ==========================================
    // 階段三：等待系統時間鎖定
    // ==========================================
    while (1) {
        gps_fix_t fix = drv_4g_gnss_get_fix();
        
        if (fix.is_fixed && fix.is_time_synced) {
            ESP_LOGI(TAG, "[LOCKED] Master GPS Fixed! System time is synced to UTC.");
            break; 
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // ==========================================
    // 階段四：通知廣播端，Master 已就緒
    // ==========================================
    sync_pkt_t ready_pkt;
    memset(&ready_pkt, 0, sizeof(sync_pkt_t));
    ready_pkt.type = MSG_TYPE_READY;
    ready_pkt.seq_num = 0;
    strncpy(ready_pkt.node_id, MY_NODE_ID, 4);
    
    ESP_LOGI(TAG, "Master Node %s READY. Standing by for Sync triggers...", MY_NODE_ID);
    
    // 連發 5 次確保廣播端收到
    for(int i = 0; i < 5; i++) {
        esp_now_send(broadcaster_mac, (uint8_t *)&ready_pkt, sizeof(ready_pkt));
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    // 主任務進入休眠
    while(1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}