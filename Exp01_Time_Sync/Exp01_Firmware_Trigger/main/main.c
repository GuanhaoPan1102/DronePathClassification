#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_timer.h"
#include "esp_vfs_fat.h"
#include "driver/sdmmc_host.h"
#include "sdmmc_cmd.h"

#include "sync_experiment_defs.h"

// ==========================================
// 實驗室切換開關
// 0: 居家 (只要 M1/S1 就開始)
// 1: 實驗室 (嚴格等待 4 台節點到齊才開始)
// ==========================================
#define WAIT_ALL_NODES 0 

static const char *TAG = "SYNC_MASTER_TRIGGER";
static uint8_t broadcast_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

static bool node_ready[TOTAL_NODES] = {false, false, false, false};
static uint32_t current_seq = 0;
static FILE *f_log = NULL;

// 數據對齊緩衝區：擴充為 4 個節點的真實時間 (秒與微秒)
typedef struct {
    int64_t s1_tv_sec; int32_t s1_tv_usec;
    int64_t s2_tv_sec; int32_t s2_tv_usec;
    int64_t s3_tv_sec; int32_t s3_tv_usec;
    int64_t s4_tv_sec; int32_t s4_tv_usec;
} sample_line_t;
static sample_line_t current_sample;

// ==========================================
// ESP-NOW 接收回調 (收集各節點回傳的微秒時間戳)
// ==========================================
static void on_data_recv(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
    if (len != sizeof(sync_pkt_t)) return;
    sync_pkt_t *pkt = (sync_pkt_t *)data;

    if (pkt->type == MSG_TYPE_READY) {
        // 🌟 巧思：將 Master(M1) 視同 S1 處理，完美兼容你的 4 節點架構
        if (strcmp(pkt->node_id, "S1") == 0 || strcmp(pkt->node_id, "M1") == 0) node_ready[0] = true;
        else if (strcmp(pkt->node_id, "S2") == 0) node_ready[1] = true;
        else if (strcmp(pkt->node_id, "S3") == 0) node_ready[2] = true;
        else if (strcmp(pkt->node_id, "S4") == 0) node_ready[3] = true;
        ESP_LOGI(TAG, "Node %s is READY", pkt->node_id);
    } 
    else if (pkt->type == MSG_TYPE_REPORT && pkt->seq_num == current_seq) {
        // 將收到的真實時間填入對應的緩衝區
        if (strcmp(pkt->node_id, "S1") == 0 || strcmp(pkt->node_id, "M1") == 0) {
            current_sample.s1_tv_sec = pkt->tv_sec; current_sample.s1_tv_usec = pkt->tv_usec;
        } else if (strcmp(pkt->node_id, "S2") == 0) {
            current_sample.s2_tv_sec = pkt->tv_sec; current_sample.s2_tv_usec = pkt->tv_usec;
        } else if (strcmp(pkt->node_id, "S3") == 0) {
            current_sample.s3_tv_sec = pkt->tv_sec; current_sample.s3_tv_usec = pkt->tv_usec;
        } else if (strcmp(pkt->node_id, "S4") == 0) {
            current_sample.s4_tv_sec = pkt->tv_sec; current_sample.s4_tv_usec = pkt->tv_usec;
        }
    }
}

// ==========================================
// SD 卡初始化
// ==========================================
esp_err_t init_sd_card() {
    vTaskDelay(pdMS_TO_TICKS(800)); // 穩定電源電壓

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false, 
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };

    sdmmc_card_t *card;
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.width = 1; 
    slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP; 

    ESP_LOGI(TAG, "Mounting SD card...");
    return esp_vfs_fat_sdmmc_mount("/sdcard", &host, &slot_config, &mount_config, &card);
}

// ==========================================
// 主程式
// ==========================================
void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    
    // Wi-Fi 啟動與頻道鎖定
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR)); 
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE));

    // SD 卡掛載與建立 CSV 檔案
    if (init_sd_card() == ESP_OK) {
        ESP_LOGI(TAG, "SD Card Mount OK. Opening CSV file...");
        f_log = fopen("/sdcard/sync_exp.csv", "w");
        if (f_log) {
            // 寫入 4 個節點的完整標頭 (M1 的資料會被記錄在 S1 欄位)
            fprintf(f_log, "Seq,S1_Sec,S1_uSec,S2_Sec,S2_uSec,S3_Sec,S3_uSec,S4_Sec,S4_uSec\n");
            ESP_LOGI(TAG, "CSV header written successfully.");
        } else {
            ESP_LOGE(TAG, "Failed to create CSV file!");
        }
    } else {
        ESP_LOGE(TAG, "SD Card Mount Failed!");
    }

    // ESP-NOW 初始化
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(on_data_recv));
    esp_now_peer_info_t peer = {.channel = 0, .encrypt = false};
    memcpy(peer.peer_addr, broadcast_mac, 6);
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));

    // ==========================================
    // 批次實驗外層迴圈 (執行 6 次，每次 5 分鐘)
    // ==========================================
    sync_pkt_t poll_pkt = {.type = MSG_TYPE_POLL, .seq_num = 0};
    sync_pkt_t sync_pkt = {.type = MSG_TYPE_SYNC};
    
    for (int iteration = 1; iteration <= EXPERIMENT_ITERATIONS; iteration++) {
        ESP_LOGI(TAG, "============================================");
        ESP_LOGI(TAG, "Preparing for Iteration %d / %d", iteration, EXPERIMENT_ITERATIONS);
        ESP_LOGI(TAG, "============================================");

        // 1. 重置所有節點的準備狀態
        node_ready[0] = false; node_ready[1] = false; 
        node_ready[2] = false; node_ready[3] = false;

        // 2. 點名循環 (不斷發送 POLL，直到所有需要的節點都回傳 READY)
        ESP_LOGI(TAG, "Polling nodes for READY status...");
        while (1) {
            esp_now_send(broadcast_mac, (uint8_t *)&poll_pkt, sizeof(poll_pkt));
            
            if (WAIT_ALL_NODES) {
                if (node_ready[0] && node_ready[1] && node_ready[2] && node_ready[3]) break;
            } else {
                if (node_ready[0]) break; // 居家模式只要 M1/S1 活著就過
            }
            vTaskDelay(pdMS_TO_TICKS(1000)); // 每秒點名一次
        }
        
        ESP_LOGI(TAG, "All required nodes are READY!");
        ESP_LOGI(TAG, "Iteration %d will start in 5 seconds...", iteration);
        vTaskDelay(pdMS_TO_TICKS(5000)); // 給予 5 秒的穩定緩衝期

        // 3. 執行單次 5 分鐘的採樣 (內層迴圈)
        TickType_t xLastWakeTime = xTaskGetTickCount();
        const TickType_t xFrequency = pdMS_TO_TICKS(SYNC_INTERVAL_MS);
        int samples_this_iter = 0;

        while (samples_this_iter < MAX_SAMPLES) {
            vTaskDelayUntil(&xLastWakeTime, xFrequency);

            current_seq++;          // 總序號 (1 ~ 9000)
            samples_this_iter++;    // 單次迴圈計數器 (1 ~ 1500)
            
            memset(&current_sample, 0, sizeof(sample_line_t));
            sync_pkt.seq_num = current_seq;
            
            esp_now_send(broadcast_mac, (uint8_t *)&sync_pkt, sizeof(sync_pkt));

            vTaskDelay(pdMS_TO_TICKS(20)); // 防禦性等待收集回傳

            if (f_log) {
                fprintf(f_log, "%lu,%lld,%ld,%lld,%ld,%lld,%ld,%lld,%ld\n", 
                        current_seq, 
                        current_sample.s1_tv_sec, current_sample.s1_tv_usec,
                        current_sample.s2_tv_sec, current_sample.s2_tv_usec,
                        current_sample.s3_tv_sec, current_sample.s3_tv_usec,
                        current_sample.s4_tv_sec, current_sample.s4_tv_usec);
            }
        }

        // 4. 單次實驗結束，強制將緩衝區寫入 SD 卡物理磁區
        if (f_log) {
            fflush(f_log);
            fsync(fileno(f_log)); 
        }
        ESP_LOGI(TAG, "Iteration %d completed. (%lu total samples collected)", iteration, current_seq);
        
        // 如果還沒到最後一次，休息 10 秒後再開始下一輪的點名
        if (iteration < EXPERIMENT_ITERATIONS) {
            ESP_LOGI(TAG, "Taking a 10-second break before next iteration...");
            vTaskDelay(pdMS_TO_TICKS(10000));
        }
    }

    if (f_log) fclose(f_log);
    ESP_LOGI(TAG, "ALL EXPERIMENTS COMPLETED SUCCESSFULLY! Total Data: %lu", current_seq);
    while(1) vTaskDelay(pdMS_TO_TICKS(10000));
}