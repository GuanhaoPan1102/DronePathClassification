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
#include "driver/gpio.h"

// 引入 Exp02 專用資料結構與 4G/GNSS 驅動
#include "espnow_experiment_defs.h"
#include "drv_4g_gnss_sync.h"

// ==========================================
// 實驗設定
// ==========================================
#define MY_NODE_ID "M1"
static const char *TAG = "EXP02_MASTER";

// 統計報表結構
typedef struct {
    uint32_t expected_seq;     // 下一個預期收到的序號
    uint32_t rx_count;         // 成功收到幾包
    uint32_t lost_count;       // 途中遺失幾包
    int64_t  total_latency_us; // 累加延遲，用來算平均值
    int64_t  max_latency_us;   // 記錄最大延遲
} node_stats_t;

// 陣列索引: 0->S1, 1->S2, 2->S3
static node_stats_t stats[3] = {0}; 

// ==========================================
// ESP-NOW 接收回調：抓出掉包元凶與計算延遲
// ==========================================
static void on_data_recv(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
    // 收到封包的瞬間，立刻抓取絕對系統時間
    struct timeval tv;
    gettimeofday(&tv, NULL);
    int64_t recv_timestamp_us = (int64_t)tv.tv_sec * 1000000LL + tv.tv_usec;

    if (len != sizeof(uav_data_pkt_t)) return;
    uav_data_pkt_t *pkt = (uav_data_pkt_t *)data;

    if (pkt->type == MSG_TYPE_UAV_DUMMY) {
        int idx = -1;
        if (strcmp(pkt->node_id, "S1") == 0) idx = 0;
        else if (strcmp(pkt->node_id, "S2") == 0) idx = 1;
        else if (strcmp(pkt->node_id, "S3") == 0) idx = 2;

        if (idx >= 0) {
            uint32_t current_seq = pkt->seq_num;

            // 1. 計算端到端傳輸延遲 (空中飛行時間 + 底層處理時間)
            int64_t latency_us = recv_timestamp_us - pkt->send_timestamp_us;
            
            // 防呆：確保時間軸有正確收斂，剔除異常的負數延遲
            if (latency_us >= 0) {
                stats[idx].total_latency_us += latency_us;
                if (latency_us > stats[idx].max_latency_us) {
                    stats[idx].max_latency_us = latency_us;
                }
            }

            // 2. 掉包稽核邏輯
            if (stats[idx].expected_seq == 0) {
                // 第一包初始化基準點
                stats[idx].expected_seq = current_seq + 1;
                stats[idx].rx_count++;
            } else {
                if (current_seq == stats[idx].expected_seq) {
                    // 完美接續
                    stats[idx].rx_count++;
                    stats[idx].expected_seq++;
                } else if (current_seq > stats[idx].expected_seq) {
                    // 發生跳號 (掉包了！)
                    uint32_t lost = current_seq - stats[idx].expected_seq;
                    stats[idx].lost_count += lost;
                    stats[idx].rx_count++;
                    
                    ESP_LOGW(TAG, "[DROP] %s lost %lu packets! (Expected: %lu, Got: %lu)", 
                             pkt->node_id, lost, stats[idx].expected_seq, current_seq);
                    
                    // 基準點校正到下一包
                    stats[idx].expected_seq = current_seq + 1;
                } else {
                    // 亂序或重複封包
                    ESP_LOGW(TAG, "[OUT-OF-ORDER] %s (Expected: %lu, Got: %lu)", 
                             pkt->node_id, stats[idx].expected_seq, current_seq);
                }
            }
        }
    }
}

// ==========================================
// 報表列印任務：每 5 秒總結一次戰況
// ==========================================
static void stats_print_task(void *pvParameters) {
    while(1) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        
        printf("\n=========================================================================\n");
        printf("Exp02 QoS Report (Interval: 5s) | Unicast Packet Loss & Latency\n");
        printf("=========================================================================\n");
        
        for(int i=0; i<3; i++) {
            uint32_t total = stats[i].rx_count + stats[i].lost_count;
            float loss_rate = 0.0f;
            if (total > 0) {
                loss_rate = ((float)stats[i].lost_count / total) * 100.0f;
            }
            
            // 計算平均延遲 (轉換為毫秒)
            float avg_latency_ms = 0.0f;
            float max_latency_ms = (float)stats[i].max_latency_us / 1000.0f;
            if (stats[i].rx_count > 0) {
                avg_latency_ms = ((float)stats[i].total_latency_us / stats[i].rx_count) / 1000.0f;
            }
            
            printf("[S%d] RX: %-5lu | LOST: %-4lu | LOSS: %5.2f%% | AVG LAT: %6.2f ms | MAX LAT: %6.2f ms\n", 
                   i+1, stats[i].rx_count, stats[i].lost_count, loss_rate, avg_latency_ms, max_latency_ms);
        }
        printf("=========================================================================\n\n");
    }
}

// ==========================================
// 主程式
// ==========================================
void app_main(void)
{
    // 電源腳位重置 (繼承自你的 Exp01 Master 設定)
    gpio_reset_pin(21);
    gpio_set_direction(21, GPIO_MODE_OUTPUT);

    // ---------------------------------------------------------
    // 1. 基礎環境與 Wi-Fi 初始化
    // ---------------------------------------------------------
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR)); // 必須跟 Slave 一樣開 LR
    ESP_ERROR_CHECK(esp_wifi_start());
    
    // 鎖定 Channel 1，確保 100% 物理層對接
    ESP_ERROR_CHECK(esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE));

    // ---------------------------------------------------------
    // 2. 初始化 Master 專用的 4G/GNSS 驅動，等待時間校準
    // ---------------------------------------------------------
    ESP_LOGI(TAG, "Initializing 4G/GNSS Sync Engine...");
    ESP_ERROR_CHECK(drv_4g_init());

    drv_4g_gnss_power(true);
    drv_4g_start_nmea_stream();

    ESP_LOGI(TAG, "Waiting for 3D Fix and GNSS Time Sync...");

    while (1) {
        gps_fix_t fix = drv_4g_gnss_get_fix();
        
        if (fix.is_fixed && fix.is_time_synced) {
            ESP_LOGI(TAG, "[LOCKED] Master GPS Fixed! System time is synced to UTC.");
            break; 
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // ---------------------------------------------------------
    // 3. ESP-NOW 初始化與註冊接收端
    // ---------------------------------------------------------
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(on_data_recv));
    // 註：Master 在這個實驗只負責收，不負責主動發，所以不用 add_peer

    ESP_LOGI(TAG, "Master Ready. Waiting for Unicast dummy data from Slaves...");

    // ---------------------------------------------------------
    // 4. 啟動報表列印背景任務
    // ---------------------------------------------------------
    xTaskCreate(stats_print_task, "stats_task", 4096, NULL, 5, NULL);
}