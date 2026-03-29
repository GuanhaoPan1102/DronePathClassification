#ifndef DRV_SD_CARD_H
#define DRV_SD_CARD_H

#include <stdint.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化 SD 卡並啟動寫入任務
 * @param packet_queue 傳入建立好的 Queue Handle (item size 必須是 sizeof(ble_packet_queue_item_t))
 */
esp_err_t sd_card_init(QueueHandle_t packet_queue);

/**
 * @brief 卸載 SD 卡
 */
void sd_card_deinit(void);

#ifdef __cplusplus
}
#endif

#endif // SD_CARD_H