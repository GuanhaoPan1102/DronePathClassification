#ifndef COMMON_SYS_DEFS_H
#define COMMON_SYS_DEFS_H

#include <stdint.h>
#include <stdbool.h> 

#ifdef __cplusplus
extern "C" {
#endif

#define BLE_PACKET_IDENTIFIER   0xAA
#define UART_HEADER_0  0xEB
#define UART_HEADER_1  0x90

// --- UAV GPS 資料結構 ---
typedef struct __attribute__((packed)) {
    int32_t latitude_scaled;  
    int32_t longitude_scaled; 
    int16_t altitude_m;       
    int16_t speed_kph_scaled; 
    uint8_t is_valid;         
} gps_data_t;

// --- BLE 封包結構 ---
typedef struct __attribute__((packed)) {
    uint8_t identifier;        
    uint8_t seq_num;           
    int32_t lat;               
    int32_t lon;               
    int16_t alt;               
    int16_t spd;               
    uint8_t checksum;          
} uav_ble_payload_t;

#define BLE_PAYLOAD_SIZE sizeof(uav_ble_payload_t)

// --- BLE to Queue 資料結構 (Scanner 傳來的原始樣貌) ---
typedef struct __attribute__((packed)) {
    uav_ble_payload_t payload; 
    int8_t  rssi;              
} ble_packet_queue_item_t;

// TTGO 內部專用，包含精準時間戳的資料結構
typedef struct {
    int64_t timestamp_us;             // UART 收到瞬間的絕對微秒時間
    ble_packet_queue_item_t ble_data; // 包含 UAV Payload 與 RSSI
} ttgo_log_item_t;

// --- Station GPS 資料結構 ---
typedef struct __attribute__((packed)) {
    int32_t latitude;       
    int32_t longitude;      
    float   altitude;       
    uint8_t is_fixed;       
    uint8_t is_time_synced; 
} gps_fix_t;

// --- ESP-NOW 訊息類型與傳輸 Payload (Exp04 備用) ---
typedef enum {
    MSG_TYPE_REGISTER = 0,    
    MSG_TYPE_FILE_SAVED = 1,  
    MSG_TYPE_BLE_DATA = 2     
} espnow_msg_type_t;

typedef struct __attribute__((packed)) {
    uint8_t  msg_type;   
    uint8_t  node_id;    
    union {
        struct __attribute__((packed)) {
            int32_t lat;
            int32_t lon;
            float   alt;
            int64_t timestamp;
        } reg;
        struct __attribute__((packed)) {
            int64_t timestamp;     
            char    filename[16];  
        } file;
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