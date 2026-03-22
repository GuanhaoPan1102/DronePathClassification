#ifndef SYNC_EXPERIMENT_DEFS_H
#define SYNC_EXPERIMENT_DEFS_H

#include <stdint.h>

/* ==========================================================================
 * 實驗參數配置
 * ========================================================================== */
#define SYNC_FREQ_HZ        5           // 廣播頻率 (5Hz)
#define SYNC_INTERVAL_MS    200         // 週期時間 (200ms)
#define TOTAL_NODES         4           // Slave 節點總數 (S1, S2, S3, S4)
#define EXPERIMENT_MINS     5          // 實驗持續時間 (分鐘)

// 計算預計總樣本數：5Hz * 60秒 * EXPERIMENT_MINS
#define MAX_SAMPLES         (EXPERIMENT_MINS * 60 * SYNC_FREQ_HZ)

/* ==========================================================================
 * 通訊協議定義
 * ========================================================================== */

/**
 * @brief 訊息類型定義
 */
typedef enum {
    MSG_TYPE_READY = 0,   // Slave -> Master: 告知定位已完成
    MSG_TYPE_SYNC,        // Master -> Slave: 同步觸發脈衝
    MSG_TYPE_REPORT       // Slave -> Master: 回傳接收當下的時間戳
} sync_msg_type_t;

/**
 * @brief 實驗數據封包結構
 * 使用 __attribute__((packed)) 確保不同板子間的記憶體對齊一致
 */
// 放在 sync_experiment_defs.h 中
typedef struct {
    uint8_t  type;        // 封包類型 (SYNC, REPORT, READY)
    uint32_t seq_num;     // 序號
    char     node_id[4];  // 節點名稱

    int64_t  tv_sec;      // 真實世界時間 (秒)
    int32_t  tv_usec;     // 真實世界時間 (微秒)
} __attribute__((packed)) sync_pkt_t;

#endif // SYNC_EXPERIMENT_DEFS_H