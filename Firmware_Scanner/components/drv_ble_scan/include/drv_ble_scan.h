#ifndef DRV_BLE_SCAN_H   // Include Guard
#define DRV_BLE_SCAN_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#ifdef __cplusplus
extern "C" {
#endif

void ble_scan_init(QueueHandle_t data_queue);

#ifdef __cplusplus
}
#endif

#endif