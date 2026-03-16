#ifndef COMMON_SYS_DEFS_H
#define COMMON_SYS_DEFS_H

#include <stdint.h>
#include <stdbool.h> 

#ifdef __cplusplus
extern "C" {
#endif

/// Fixed header byte for BLE telemetry packets (0xAA).
/// @note [Protocol Identifier]
/// This header is used to distinguish UAV telemetry packets from other ambient BLE noise 
/// (e.g., smartwatches, headphones).
/// In a standardized commercial deployment (e.g., Remote ID), this custom header 
/// would be replaced by a standard Service UUID
#define BLE_PACKET_IDENTIFIER   0xAA

#define UART_HEADER_0  0xEB
#define UART_HEADER_1  0x90

// ---------------------------------------------------------
// UAV GPS 資料結構
// ---------------------------------------------------------
// 加上 packed，確保未來若整塊寫入 SD 卡或傳輸時不會有 Padding
typedef struct __attribute__((packed)) {
    int32_t latitude_scaled;  // 緯度：單位為 度 * 10^7
    int32_t longitude_scaled; // 經度：單位為 度 * 10^7
    int16_t altitude_m;       // 高度：單位為 公尺 (m)
    int16_t speed_kph_scaled; // 地面速度：單位為 0.01 km/h
    
    uint8_t is_valid;         // 1 = 定位有效 (Fix), 0 = 搜尋中或無效
} gps_data_t;


// ---------------------------------------------------------
// BLE 封包結構
// ---------------------------------------------------------
typedef struct __attribute__((packed)) {
    uint8_t identifier;        // 標示碼
    uint8_t seq_num;           // 封包序列碼
    int32_t lat;               // 緯度 (單位為 度 * 10^7)
    int32_t lon;               // 經度 (單位為 度 * 10^7)
    int16_t alt;               // 高度 (單位為 公尺 (m))
    int16_t spd;               // 速度 (單位為 0.01 km/h)
    uint8_t checksum;          // 校驗碼
} uav_ble_payload_t;

#define BLE_PAYLOAD_SIZE        sizeof(uav_ble_payload_t)


// ---------------------------------------------------------
// BLE to Queue 資料結構
// ---------------------------------------------------------
typedef struct __attribute__((packed)) {
    uav_ble_payload_t payload; // 原本的 UAV 資料 (15 bytes)
    int8_t  rssi;              // 訊號強度
} ble_packet_queue_item_t;


// ---------------------------------------------------------
// Station GPS 資料結構
// ---------------------------------------------------------
typedef struct __attribute__((packed)) {
    int32_t latitude;       // 緯度 (例如 221234567 代表 22.1234567 度)
    int32_t longitude;      // 經度
    float   altitude;       // 高度 (單位: 公尺)
    uint8_t is_fixed;       // 是否已經完成 60 次平均並鎖定
    uint8_t is_time_synced; // 是否已經完成時間同步
} gps_fix_t;


// ---------------------------------------------------------
// ESP-NOW 訊息類型與傳輸 Payload
// ---------------------------------------------------------
typedef enum {
    MSG_TYPE_REGISTER = 0,    // 節點註冊 (傳送地面站 GPS 位置)
    MSG_TYPE_FILE_SAVED = 1,  // 檔案儲存完畢通知
    MSG_TYPE_BLE_DATA = 2     // 傳輸 BLE RSSI 用
} espnow_msg_type_t;

// ESP-NOW 傳輸 Payload
typedef struct __attribute__((packed)) {
    uint8_t  msg_type;   // 對應 espnow_msg_type_t
    uint8_t  node_id;    // 節點編號 (例如 Slave 1, 2, 3)
    
    union {
        // [MSG_TYPE_REGISTER] 註冊用的資料
        struct __attribute__((packed)) {
            int32_t lat;
            int32_t lon;
            float   alt;
            int64_t timestamp;
        } reg;

        // [MSG_TYPE_FILE_SAVED] 檔案儲存回報用的資料
        struct __attribute__((packed)) {
            int64_t timestamp;     // 儲存當下的時間戳 (已校正為純 UTC)
            char    filename[16];  // 檔名，例如 "DATA1.csv"
        } file;
        
        // [MSG_TYPE_BLE_DATA] 即時 BLE 資料
        struct __attribute__((packed)) {
            int64_t timestamp;
            ble_packet_queue_item_t ble_data; 
        } live_data;

    } data;
} espnow_payload_t;

#ifdef __cplusplus
}
#endif

#endif // COMMON_SYS_DEFS_H