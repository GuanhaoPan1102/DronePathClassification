#ifndef ESPNOW_EXPERIMENT_DEFS_H
#define ESPNOW_EXPERIMENT_DEFS_H

#include <stdint.h>

// ==========================================
// 實驗訊息類型定義 (Message Types)
// ==========================================
// Exp01 舊有保留 (維持相容性)
#define MSG_TYPE_SYNC      0x01
#define MSG_TYPE_REPORT    0x02
#define MSG_TYPE_READY     0x03

// 🌟 Exp02 新增：排程與壓力測試專用
#define MSG_TYPE_POLL      0x04 // Master -> Slave: 點名檢查 GPS 狀態
#define MSG_TYPE_SCHEDULE  0x05 // Master -> Slave: 發布預約開火時間與參數
#define MSG_TYPE_UAV_DUMMY 0x06 // Slave -> Master: Exp02 壓力測試假資料
#define MSG_TYPE_UAV_DATA  0x07 // 未來實戰：真實 BLE 數據回傳

// ==========================================
// Exp01: 基礎對時封包結構 
// ==========================================
typedef struct __attribute__((packed)) {
    uint8_t  type;
    uint32_t seq_num;
    char     node_id[4];
    int64_t  tv_sec;
    int32_t  tv_usec;
} sync_pkt_t;

// ==========================================
// 🌟 Exp02: 預約排程封包結構 (Master 發給 Slave)
// ==========================================
typedef struct __attribute__((packed)) {
    uint8_t  type;                 // 必須為 MSG_TYPE_SCHEDULE
    int64_t  target_start_time_us; // 預約開火的絕對微秒時間 (例如 10:00:15.000000)
    uint32_t frequency_hz;         // 發送頻率 (本次實驗將傳入 1, 10 或 15)
    uint32_t duration_sec;         // 實驗持續時間 (本次實驗為 300 秒)
} schedule_pkt_t;

// ==========================================
// Exp02 & 最終系統: ESP-NOW 無人機 BLE 數據結構 
// ==========================================
// 定義每批次打包的最大 BLE 紀錄數量
#define MAX_BLE_RECORDS_PER_PACKET 10

// ---------------------------------------------------------
// 1. 單筆 BLE 掃描紀錄 (Total: 16 Bytes)
// ---------------------------------------------------------
typedef struct __attribute__((packed)) {
    int64_t timestamp_us;  // 絕對微秒時間戳 (8 Bytes)
    uint8_t mac[6];        // 無人機 BLE MAC 位址 (6 Bytes)
    int8_t  rssi;          // 訊號強度 (1 Byte)
    uint8_t padding;       // 記憶體對齊填充 (1 Byte)
} ble_record_t;

// ---------------------------------------------------------
// 2. ESP-NOW 單播回傳封包 (Total Payload: 184 Bytes)
// ---------------------------------------------------------
typedef struct __attribute__((packed)) {
    // --- 封包表頭 (Header) : 24 Bytes ---
    uint8_t  type;               // 訊息類型 (MSG_TYPE_UAV_DUMMY 或 MSG_TYPE_UAV_DATA)
    char     node_id[4];         // 來源節點 ID，如 "S1\0\0" (4 Bytes)
    uint32_t seq_num;            // 封包流水號，算掉包率的絕對關鍵 (4 Bytes)
    uint8_t  record_count;       // 實際裝載的紀錄數量 (1 Byte)
    
    int64_t  send_timestamp_us;  // 發射瞬間的真實微秒時間戳 (8 Bytes)
    
    uint8_t  padding[6];         // 記憶體對齊填充 (6 Bytes) 
                                 // (1+4+4+1+8+6 = 24 Bytes 完美對齊)
    
    // --- 負載資料 (Payload) : 160 Bytes ---
    // 陣列：裝載 BLE 紀錄 (16 Bytes * 10 = 160 Bytes)
    ble_record_t records[MAX_BLE_RECORDS_PER_PACKET]; 
} uav_data_pkt_t;

#endif // ESPNOW_EXPERIMENT_DEFS_H