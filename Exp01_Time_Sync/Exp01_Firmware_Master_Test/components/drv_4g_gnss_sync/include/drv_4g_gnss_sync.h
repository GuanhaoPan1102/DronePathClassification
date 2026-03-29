#ifndef DRV_4G_GNSS_SYNC_H
#define DRV_4G_GNSS_SYNC_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "driver/uart.h"
#include "esp_err.h"
#include "sys_defs.h"

// ==========================================
// 4G 模組與 GNSS 硬體腳位定義 (Master 節點專用)
// ==========================================
#define UART_4G_PORT_NUM      UART_NUM_2   // 使用 UART2
#define UART_4G_TX_PIN        23           // ESP32 TX
#define UART_4G_RX_PIN        18           // ESP32 RX
#define UART_4G_BAUD_RATE     115200       // 4G 模組速率
#define UART_4G_BUF_SIZE      2048         // 緩衝區大小

#define ACTIVE_ANTENNA 1                   // 是否使用主動式天線

#ifdef __cplusplus
extern "C" {
#endif

// ==========================================
// 1. 系統初始化 API
// ==========================================
esp_err_t drv_4g_init(void);

// ==========================================
// 2. GNSS 定位與時間同步 API
// ==========================================
void drv_4g_gnss_reset_fix(void);
gps_fix_t drv_4g_gnss_get_fix(void);
void drv_4g_gnss_process_nmea(const char *nmea_line);

// ==========================================
// 3. 4G 網路與 AT 指令 API (事件驅動與狀態切換)
// ==========================================
esp_err_t drv_4g_send_at_cmd(const char *cmd, const char *expected_response, uint32_t timeout_ms, char *out_response, size_t out_max_len);
esp_err_t drv_4g_gnss_power(bool enable);

// 核心切換機制：啟動校準模式 (NMEA ON)
esp_err_t drv_4g_start_nmea_stream(void);

esp_err_t drv_4g_stop_nmea_stream(void);

esp_err_t drv_4g_set_apn(void);
esp_err_t drv_4g_tcp_connect(const char *ip, int port);
esp_err_t drv_4g_tcp_send(const uint8_t *data, size_t len);
esp_err_t drv_4g_tcp_close(void);

#ifdef __cplusplus
}
#endif

#endif // DRV_4G_GNSS_SYNC_H