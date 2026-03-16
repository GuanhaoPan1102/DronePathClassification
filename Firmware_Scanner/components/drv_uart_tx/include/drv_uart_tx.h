#ifndef DRV_UART_TX_H   // Include Guard
#define DRV_UART_TX_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#ifdef __cplusplus
extern "C" {
#endif

void uart_init(QueueHandle_t data_queue);

#ifdef __cplusplus
}
#endif

#endif