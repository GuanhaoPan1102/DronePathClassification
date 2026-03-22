#ifndef DRV_4G_GNSS_SYNC_H
#define DRV_4G_GNSS_SYNC_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "driver/uart.h"
#include "esp_err.h"
#include "sys_defs.h"  // 引入 gps_fix_t

// ==========================================
// 4G 模組與 GNSS 硬體腳位定義 (Master 節點專用)
// ==========================================
#define UART_4G_PORT_NUM      UART_NUM_2   // 使用 UART2
#define UART_4G_TX_PIN        23           // ESP32 TX
#define UART_4G_RX_PIN        18           // ESP32 RX
#define UART_4G_BAUD_RATE     115200       // 4G 模組速率
#define UART_4G_BUF_SIZE      2048         // 緩衝區大小
#define MASTER_PPS_PIN        4            // 接收 4G 模組 PPS 的 GPIO

// ==========================================
// 1. 系統初始化 API
// ==========================================

/**
 * @brief 統一初始化 4G 模組與 GNSS 同步引擎
 * @details 包含 UART 設定、PPS 中斷註冊、事件標誌組建立，並啟動背景路由器 Task
 * @return esp_err_t ESP_OK 表示初始化成功
 */
esp_err_t drv_4g_init(void);

// ==========================================
// 2. GNSS 定位與時間同步 API
// ==========================================

/**
 * @brief 重置 Master 定位與同步狀態 (重新啟動 Survey-In)
 */
void drv_4g_gnss_reset_fix(void);

/**
 * @brief 獲取 Master 目前的定位與同步狀態
 * @return gps_fix_t (定義於 sys_defs.h)
 */
gps_fix_t drv_4g_gnss_get_fix(void);

/**
 * @brief 餵入 NMEA 字串進行解析
 * @param nmea_line 以 '$' 開頭的 NMEA 原始字串
 */
void drv_4g_gnss_process_nmea(const char *nmea_line);

// ==========================================
// 3. 4G 網路與 AT 指令 API (事件驅動架構)
// ==========================================

/**
 * @brief 發送 AT 指令並等待預期的字串回覆
 * @param cmd 完整的 AT 指令
 * @param expected_response 預期收到的成功字串
 * @param timeout_ms 超時時間
 * @param out_response (可選) 存放回傳內容的 Buffer
 * @param out_max_len Buffer 最大長度
 */
esp_err_t drv_4g_send_at_cmd(const char *cmd, const char *expected_response, uint32_t timeout_ms, char *out_response, size_t out_max_len);

/**
 * @brief 開啟或關閉 GPS 模組電源 (AT+MGPSC)
 */
esp_err_t drv_4g_gnss_power(bool enable);

/**
 * @brief 開啟 NMEA 持續更新廣播 (AT+MGPSGET=ALL,1)
 */
esp_err_t drv_4g_start_nmea_stream(void);

/**
 * @brief 關閉 NMEA 持續更新廣播 (AT+MGPSGET=ALL,0)
 */
esp_err_t drv_4g_stop_nmea_stream(void);

/**
 * @brief 設定 APN 註冊網路 (AT+QICSGP)
 */
esp_err_t drv_4g_set_apn(void);

/**
 * @brief 建立 TCP 連線
 */
esp_err_t drv_4g_tcp_connect(const char *ip, int port);

/**
 * @brief 透過 TCP 發送定長資料
 */
esp_err_t drv_4g_tcp_send(const uint8_t *data, size_t len);

/**
 * @brief 關閉 TCP 連線
 */
esp_err_t drv_4g_tcp_close(void);

#endif // DRV_4G_GNSS_SYNC_H