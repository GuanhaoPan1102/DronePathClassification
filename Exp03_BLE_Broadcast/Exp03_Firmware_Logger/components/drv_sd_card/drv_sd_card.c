#include <stdio.h>
#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"
#include "esp_log.h"
#include "drv_sd_card.h"
#include "sys_defs.h"

static const char *TAG = "SD_CARD";
#define MOUNT_POINT "/sdcard"

#define SESSION_TIMEOUT_MS 3000 
#define SYNC_INTERVAL 10

static sdmmc_card_t *card = NULL;
static bool is_mounted = false;
static TaskHandle_t s_sd_task_handle = NULL;
static int s_next_file_index = 1; 

static void find_next_free_index(void) {
    char test_filename[64];
    struct stat st;
    int i = 1;
    while (1) {
        snprintf(test_filename, sizeof(test_filename), MOUNT_POINT "/data%d.csv", i);
        if (stat(test_filename, &st) != 0) {
            s_next_file_index = i;
            ESP_LOGI(TAG, "Next available file index: %d (%s)", s_next_file_index, test_filename);
            break;
        }
        i++;
    }
}

static void sd_card_write_task(void *pvParameters)
{
    QueueHandle_t queue = (QueueHandle_t)pvParameters;
    ttgo_log_item_t item; 
    
    FILE *f = NULL;
    char current_filename[64] = {0}; 
    char line_buffer[128];
    int unsaved_count = 0;

    ESP_LOGI(TAG, "ML Data Logger Task Started on Core %d", xPortGetCoreID());

    while (1) {
        if (xQueueReceive(queue, &item, pdMS_TO_TICKS(SESSION_TIMEOUT_MS)) == pdTRUE) {
            
            if (!is_mounted) continue;

            if (f == NULL) {
                snprintf(current_filename, sizeof(current_filename), MOUNT_POINT "/data%d.csv", s_next_file_index);
                f = fopen(current_filename, "w");
                if (f == NULL) {
                    ESP_LOGE(TAG, "Failed to create file: %s", current_filename);
                    continue;
                }

                fprintf(f, "unix_time_us,rssi,seq_num,lat,lon,alt_m,spd_kmh\n");
                ESP_LOGI(TAG, ">>> Start Recording: %s", current_filename);
                s_next_file_index++; 
                unsaved_count = 0;
            }

            // 格式化資料：完美寫入高精度微秒時間戳
            int len = snprintf(line_buffer, sizeof(line_buffer), 
                    "%lld,%d,%d,%.7f,%.7f,%d,%.2f\n",
                    item.timestamp_us,
                    item.ble_data.rssi,
                    item.ble_data.payload.seq_num,
                    (double)item.ble_data.payload.lat / 10000000.0,
                    (double)item.ble_data.payload.lon / 10000000.0,
                    item.ble_data.payload.alt,
                    (double)item.ble_data.payload.spd / 100.0
            );

            fwrite(line_buffer, 1, len, f);
            unsaved_count++;

            // 防斷電機制：每 10 筆強迫寫入實體 SD 卡
            if (unsaved_count >= SYNC_INTERVAL) {
                fsync(fileno(f));
                unsaved_count = 0;
            }

        } else {
            if (f != NULL) {
                fclose(f);
                f = NULL;
                unsaved_count = 0;
                ESP_LOGI(TAG, "<<< Timeout (3s). Saved & Closed: %s", current_filename);
            }
        }
    }
}

esp_err_t sd_card_init(QueueHandle_t packet_queue)
{
    if (is_mounted) return ESP_OK;
    if (packet_queue == NULL) return ESP_ERR_INVALID_ARG;

    ESP_LOGI(TAG, "Initializing SD (1-bit, 20MHz)...");

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = true,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };

    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.width = 1;
    slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

    esp_err_t ret = esp_vfs_fat_sdmmc_mount(MOUNT_POINT, &host, &slot_config, &mount_config, &card);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Mount failed: %s", esp_err_to_name(ret));
        return ret;
    }
    is_mounted = true;

    find_next_free_index();

    xTaskCreatePinnedToCore(sd_card_write_task, "sd_ml_writer", 4096, (void*)packet_queue, 2, &s_sd_task_handle, 1);
    
    return ESP_OK;
}

void sd_card_deinit(void)
{
    if (s_sd_task_handle != NULL) {
        vTaskDelete(s_sd_task_handle);
        s_sd_task_handle = NULL;
    }
    if (is_mounted) {
        esp_vfs_fat_sdcard_unmount(MOUNT_POINT, card);
        is_mounted = false;
        card = NULL;
    }
}