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

#include "espnow_experiment_defs.h"
#include "drv_gnss_sync.h"

// 節點身分設定 (燒錄前需修改為 "S1", "S2" 或 "S3")
#define MY_NODE_ID "S1" 
static const char *TAG = "EXP02_SLAVE";

// Master 節點 MAC 位址
static uint8_t master_mac[6] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};

// 實驗排程全域變數
static volatile bool schedule_received = false;
static schedule_pkt_t current_schedule;

// ==========================================
// ESP-NOW 接收回調函數
// ==========================================
static void on_data_recv(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
    if (len == sizeof(sync_pkt_t)) {
        sync_pkt_t *pkt = (sync_pkt_t *)data;
        // 處理 Master 的點名要求
        if (pkt->type == MSG_TYPE_POLL) {
            gps_fix_t fix = gnss_get_fix();
            if (fix.is_fixed && fix.is_time_synced) {
                sync_pkt_t ready_pkt;
                memset(&ready_pkt, 0, sizeof(sync_pkt_t));
                ready_pkt.type = MSG_TYPE_READY;
                strncpy(ready_pkt.node_id, MY_NODE_ID, 4);
                
                // 隨機避讓 (0~50ms) 避免網路層碰撞
                vTaskDelay(pdMS_TO_TICKS(esp_random() % 50));
                esp_now_send(recv_info->src_addr, (uint8_t *)&ready_pkt, sizeof(ready_pkt));
            }
        }
    } 
    else if (len == sizeof(schedule_pkt_t)) {
        schedule_pkt_t *pkt = (schedule_pkt_t *)data;
        // 處理 Master 發布的預約開火指令
        if (pkt->type == MSG_TYPE_SCHEDULE) {
            memcpy(&current_schedule, pkt, sizeof(schedule_pkt_t));
            schedule_received = true;
        }
    }
}

// ==========================================
// 主程式
// ==========================================
void app_main(void)
{
    // 基礎環境初始化
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

    // GNSS 同步驅動初始化
    gnss_sync_init();

    // ESP-NOW 初始化
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(on_data_recv));
    esp_now_peer_info_t peer = {.channel = 0, .encrypt = false};
    memcpy(peer.peer_addr, master_mac, 6);
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));

    ESP_LOGI(TAG, "系統初始化完成，進入狀態監控模式。");

    // ==========================================
    // 主循環：狀態監控與排程執行
    // ==========================================
    while (1) {
        if (schedule_received) {
            schedule_received = false;
            
            ESP_LOGI(TAG, "接收排程指令。目標時間: %lld us, 頻率: %lu Hz, 時長: %lu s",
                     current_schedule.target_start_time_us, 
                     current_schedule.frequency_hz, 
                     current_schedule.duration_sec);

            // 1. 高精度絕對時間等待迴圈
            struct timeval tv;
            while (1) {
                gettimeofday(&tv, NULL);
                int64_t now_us = ((int64_t)tv.tv_sec * 1000000LL) + tv.tv_usec;
                
                if (now_us >= current_schedule.target_start_time_us) {
                    break; // 到達物理發射時間
                }
                
                // 距離目標時間大於 2ms 時釋放 CPU，小於 2ms 時進入 Busy-wait 確保微秒級精度
                if (current_schedule.target_start_time_us - now_us > 2000) {
                    vTaskDelay(1); 
                }
            }

            // 2. 執行定頻測試封包發送
            ESP_LOGI(TAG, "到達目標時間，啟動壓力測試。");
            
            uint32_t total_packets = current_schedule.duration_sec * current_schedule.frequency_hz;
            const TickType_t xFrequency = pdMS_TO_TICKS(1000 / current_schedule.frequency_hz);
            TickType_t xLastWakeTime = xTaskGetTickCount();

            // 初始化測試用假封包
            uav_data_pkt_t dummy_pkt;
            memset(&dummy_pkt, 0, sizeof(uav_data_pkt_t));
            dummy_pkt.type = MSG_TYPE_UAV_DUMMY;
            strncpy(dummy_pkt.node_id, MY_NODE_ID, 4);

            for (uint32_t seq = 1; seq <= total_packets; seq++) {
                // 紀錄發射瞬間的絕對時間
                gettimeofday(&tv, NULL);
                dummy_pkt.send_timestamp_us = ((int64_t)tv.tv_sec * 1000000LL) + tv.tv_usec;
                dummy_pkt.seq_num = seq;
                
                esp_now_send(master_mac, (uint8_t *)&dummy_pkt, sizeof(dummy_pkt));

                // 使用 FreeRTOS 絕對延遲，確保發送頻率不受執行時間影響
                vTaskDelayUntil(&xLastWakeTime, xFrequency);
            }
            
            ESP_LOGI(TAG, "壓力測試結束。總計發送 %lu 筆封包。返回待命狀態。", total_packets);
            
        } else {
            // 待命狀態：定期輸出 GPS 狀態
            gps_fix_t fix = gnss_get_fix(); 
            if (fix.is_fixed && fix.is_time_synced) {
                ESP_LOGI(TAG, "狀態: [已鎖定] 系統時間同步完成，等待排程指令...");
            } else {
                ESP_LOGW(TAG, "狀態: [搜尋中] 等待 GNSS 3D Fix 與 PPS 信號...");
            }
            vTaskDelay(pdMS_TO_TICKS(2000));
        }
    }
}