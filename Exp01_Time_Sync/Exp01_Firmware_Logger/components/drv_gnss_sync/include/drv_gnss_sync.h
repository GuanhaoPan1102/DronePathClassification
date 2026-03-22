#ifndef DRV_GNSS_SYNC_H
#define DRV_GNSS_SYNC_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/uart.h"
#include "sys_defs.h"

// ==========================================
// GNSS 模組硬體腳位與通訊設定 (Slave 節點專用)
// ==========================================
#define GPS_UART_NUM  UART_NUM_2
#define GPS_TX_PIN    23
#define GPS_RX_PIN    18
#define GPS_PPS_PIN   4
#define GPS_BAUD      9600

// ==========================================
// 對外開放的 API 介面
// ==========================================

/**
 * @brief 初始化 GNSS 驅動與 PPS 同步引擎
 * @note 硬體參數已封裝於標頭檔的巨集，呼叫時無須傳入參數。
 */
void gnss_sync_init(void);

/**
 * @brief 重置 GNSS 定位與時間同步狀態 (重新啟動 60 次 Survey-In)
 */
void gnss_reset_fix(void);

/**
 * @brief 獲取目前的定位與同步狀態
 * @return gps_fix_t (該結構體定義於 sys_defs.h)
 */
gps_fix_t gnss_get_fix(void);

#endif // DRV_GNSS_SYNC_H