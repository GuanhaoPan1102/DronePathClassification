#include <stdio.h>
#include <string.h>
#include <sys/unistd.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "driver/gpio.h"
#include "esp_vfs_fat.h"
#include "driver/sdmmc_host.h"
#include "sdmmc_cmd.h"

#include "espnow_experiment_defs.h" 
#include "drv_4g_gnss_sync.h"      

static const char *TAG = "EXP02_MASTER";

// Broadcast MAC address for POLL and SCHEDULE messages
static uint8_t broadcast_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; 

// Node readiness status (S1, S2, S3)
static bool node_ready[3] = {false, false, false}; 
static FILE *f_log = NULL;

// Experiment parameters
#define EXPERIMENT_ITERATIONS 6
#define EXPERIMENT_DURATION_SEC 300
static const uint32_t freq_schedule[EXPERIMENT_ITERATIONS] = {1, 1, 10, 10, 15, 15};

// ==========================================
// Asynchronous SD Card Write Queue
// ==========================================
typedef struct {
    int64_t recv_timestamp_us;
    char    node_id[4];
    uint32_t seq_num;
    int64_t send_timestamp_us;
} log_item_t;

static QueueHandle_t sd_write_queue;

static void sd_writer_task(void *pvParameters) {
    log_item_t item;
    ESP_LOGI(TAG, "SD writer task initialized.");
    
    while (1) {
        if (xQueueReceive(sd_write_queue, &item, portMAX_DELAY) == pdTRUE) {
            if (f_log != NULL) {
                int64_t latency_us = item.recv_timestamp_us - item.send_timestamp_us;
                fprintf(f_log, "%lld,%s,%lu,%lld,%lld\n", 
                        item.recv_timestamp_us, 
                        item.node_id, 
                        item.seq_num, 
                        item.send_timestamp_us, 
                        latency_us);
            }
        }
    }
}

// ==========================================
// SD Card Initialization
// ==========================================
esp_err_t init_sd_card() {
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
// ESP-NOW Receive Callback
// ==========================================
static void on_data_recv(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
    // Capture absolute time upon packet arrival
    struct timeval tv;
    gettimeofday(&tv, NULL);
    int64_t current_time_us = ((int64_t)tv.tv_sec * 1000000LL) + tv.tv_usec;

    if (len == sizeof(sync_pkt_t)) {
        sync_pkt_t *pkt = (sync_pkt_t *)data;
        if (pkt->type == MSG_TYPE_READY) {
            if (strcmp(pkt->node_id, "S1") == 0) node_ready[0] = true;
            else if (strcmp(pkt->node_id, "S2") == 0) node_ready[1] = true;
            else if (strcmp(pkt->node_id, "S3") == 0) node_ready[2] = true;
        }
    }
    else if (len == sizeof(uav_data_pkt_t)) {
        uav_data_pkt_t *pkt = (uav_data_pkt_t *)data;
        if (pkt->type == MSG_TYPE_UAV_DUMMY) {
            log_item_t log_item = {
                .recv_timestamp_us = current_time_us,
                .seq_num = pkt->seq_num,
                .send_timestamp_us = pkt->send_timestamp_us
            };
            strncpy(log_item.node_id, pkt->node_id, 4);
            
            // Non-blocking write to queue
            xQueueSendFromISR(sd_write_queue, &log_item, NULL);
        }
    }
}

// ==========================================
// Main Application
// ==========================================
void app_main(void)
{
    // Power on 4G/GNSS module
    gpio_reset_pin(21);
    gpio_set_direction(21, GPIO_MODE_OUTPUT);
    gpio_set_level(21, 1);

    // System and WiFi initialization
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE));

    // Initialize SD card and writer queue
    init_sd_card();
    sd_write_queue = xQueueCreate(500, sizeof(log_item_t));
    xTaskCreatePinnedToCore(sd_writer_task, "sd_writer", 4096, NULL, 5, NULL, 1);

    // ESP-NOW initialization
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(on_data_recv));
    esp_now_peer_info_t peer = {.channel = 0, .encrypt = false};
    memcpy(peer.peer_addr, broadcast_mac, 6);
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));

    // GNSS initialization and time synchronization
    ESP_LOGI(TAG, "Initializing 4G/GNSS sync engine...");
    ESP_ERROR_CHECK(drv_4g_init());
    drv_4g_gnss_power(true);
    drv_4g_start_nmea_stream();

    ESP_LOGI(TAG, "Waiting for GNSS fix and UTC time sync...");
    while (1) {
        gps_fix_t fix = drv_4g_gnss_get_fix(); 
        if (fix.is_fixed && fix.is_time_synced) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(1000)); 
    }
    ESP_LOGI(TAG, "Master time synchronized. Entering experiment scheduler.");

    // ==========================================
    // Experiment Scheduling Loop
    // ==========================================
    sync_pkt_t poll_pkt = {.type = MSG_TYPE_POLL, .seq_num = 0};
    schedule_pkt_t schedule_pkt = {.type = MSG_TYPE_SCHEDULE, .duration_sec = EXPERIMENT_DURATION_SEC};

    for (int iter = 0; iter < EXPERIMENT_ITERATIONS; iter++) {
        uint32_t current_freq = freq_schedule[iter];
        ESP_LOGI(TAG, "------------------------------------------------");
        ESP_LOGI(TAG, "Starting iteration %d/%d. Target frequency: %lu Hz", iter + 1, EXPERIMENT_ITERATIONS, current_freq);

        // 1. Create CSV file for the current iteration
        char filename[64];
        sprintf(filename, "/sdcard/exp02_%luHz_iter%d.csv", current_freq, iter + 1);
        f_log = fopen(filename, "w");
        if (f_log) {
            fprintf(f_log, "RecvTime_us,NodeID,SeqNum,SendTime_us,Latency_us\n");
            fflush(f_log);
            ESP_LOGI(TAG, "Log file created: %s", filename);
        } else {
            ESP_LOGE(TAG, "Failed to create log file: %s", filename);
        }

        // 2. Polling phase to ensure all Slaves are ready
        node_ready[0] = false; node_ready[1] = false; node_ready[2] = false;
        ESP_LOGI(TAG, "Polling Slave nodes for readiness...");
        while (!(node_ready[0] && node_ready[1] && node_ready[2])) {
            esp_now_send(broadcast_mac, (uint8_t *)&poll_pkt, sizeof(poll_pkt));
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        ESP_LOGI(TAG, "All Slave nodes reported READY.");

        // 3. Calculate target start time and broadcast schedule
        struct timeval tv;
        gettimeofday(&tv, NULL);
        int64_t now_us = ((int64_t)tv.tv_sec * 1000000LL) + tv.tv_usec;
        
        // Schedule start time: 10 seconds from current absolute time
        schedule_pkt.target_start_time_us = now_us + 10000000LL; 
        schedule_pkt.frequency_hz = current_freq;

        ESP_LOGI(TAG, "Broadcasting SCHEDULE. Experiment starting in 10 seconds.");
        for (int i = 0; i < 3; i++) {
            esp_now_send(broadcast_mac, (uint8_t *)&schedule_pkt, sizeof(schedule_pkt));
            vTaskDelay(pdMS_TO_TICKS(100)); // Send multiple times to prevent packet loss
        }

        // 4. Wait for the experiment to run (10s initial delay + duration + 2s buffer)
        uint32_t wait_time_sec = 10 + EXPERIMENT_DURATION_SEC + 2;
        ESP_LOGI(TAG, "Listening for incoming data for %lu seconds...", wait_time_sec);
        vTaskDelay(pdMS_TO_TICKS(wait_time_sec * 1000));

        // 5. Finalize log file
        if (f_log) {
            fflush(f_log);
            fsync(fileno(f_log));
            fclose(f_log);
            f_log = NULL;
            ESP_LOGI(TAG, "Iteration %d completed. Data successfully saved.", iter + 1);
        }

        // 6. Delay between iterations
        if (iter < EXPERIMENT_ITERATIONS - 1) {
            ESP_LOGI(TAG, "Waiting 10 seconds before starting the next iteration...");
            vTaskDelay(pdMS_TO_TICKS(10000));
        }
    }

    ESP_LOGI(TAG, "All experiment iterations completed successfully.");
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}