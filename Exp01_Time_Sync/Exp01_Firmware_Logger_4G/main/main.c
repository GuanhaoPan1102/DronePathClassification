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

#include "sync_experiment_defs.h"
#include "drv_4g_gnss_sync.h" // Master 專用的同步驅動

// ==========================================
// 🚀 實驗設定：Master 在此實驗中代號為 "M1"
// ==========================================
#define MY_NODE_ID "M1" 

static const char *TAG = "SYNC_MASTER_NODE";
// 這裡填入你那個「獨立廣播端 MCU」的 MAC 位址
static uint8_t broadcaster_mac[6] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF}; 

/**
 * @brief ESP-NOW 接收回調：Master 節點受測邏輯
 */
static void on_data_recv(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
    if (len != sizeof(sync_pkt_t)) return;
    sync_pkt_t *recv_pkt = (sync_pkt_t *)data;

    // 1. 收到廣播端的同步觸發封包
    if (recv_pkt->type == MSG_TYPE_SYNC) {
        // 瞬間抓取由 4G 模組 PPS 校正後的系統時間
        struct timeval tv;
        gettimeofday(&tv, NULL);

        // 2. 打包回傳給廣播端 (或是你指定的紀錄端)
        sync_pkt_t report_pkt;
        report_pkt.type = MSG_TYPE_REPORT;
        report_pkt.seq_num = recv_pkt->seq_num;
        strncpy(report_pkt.node_id, MY_NODE_ID, 4);
        
        report_pkt.tv_sec = (int64_t)tv.tv_sec;
        report_pkt.tv_usec = tv.tv_usec;

        // 隨機避讓避免與 Slave 同時發送造成碰撞 (0~10ms)
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
    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR)); 
    ESP_ERROR_CHECK(esp_wifi_start());

    // 2. 初始化 Master 專用的 4G/GNSS 驅動 (整合了背景路由器任務)
    drv_4g_init();

    // 3. 初始化 ESP-NOW
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(on_data_recv));
    
    // 註冊廣播端為 Peer
    esp_now_peer_info_t peer = {.channel = 0, .encrypt = false};
    memcpy(peer.peer_addr, broadcaster_mac, 6);
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));

    // 4. 等待 4G 模組定位鎖定與 PPS 時間同步完成
    ESP_LOGI(TAG, "Master Node %s: Starting Survey-In and PPS Sync...", MY_NODE_ID);
    
    // 開啟 GNSS 電源與 NMEA 串流以啟動同步引擎
    drv_4g_gnss_power(true);
    drv_4g_start_nmea_stream();

    while (1) {
        gps_fix_t fix = drv_4g_gnss_get_fix(); // 取得驅動維護的狀態
        
        if (fix.is_fixed && fix.is_time_synced) {
            ESP_LOGI(TAG, "Master GPS Fixed! Lat:%d, Lon:%d, Altitude:%.2f", 
                     fix.latitude, fix.longitude, fix.altitude);
            break; 
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // 5. 鎖定完成後，關閉串流以確保實驗期間 UART 不會被 NMEA 灌爆
    // 這一步很重要，因為 Master 只有一組 UART 通道
    drv_4g_stop_nmea_stream();

    // 6. 回報 READY 給廣播端
    sync_pkt_t ready_pkt = {
        .type = MSG_TYPE_READY,
        .seq_num = 0,
    };
    strncpy(ready_pkt.node_id, MY_NODE_ID, 4);
    
    ESP_LOGI(TAG, "Master Node ready. Standing by for Sync pulses from Broadcaster...");
    
    for(int i = 0; i < 5; i++) {
        esp_now_send(broadcaster_mac, (uint8_t *)&ready_pkt, sizeof(ready_pkt));
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    while(1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}