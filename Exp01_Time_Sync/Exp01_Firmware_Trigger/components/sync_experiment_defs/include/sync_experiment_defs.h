#ifndef SYNC_EXPERIMENT_DEFS_H
#define SYNC_EXPERIMENT_DEFS_H

#include <stdint.h>

/* ==========================================================================
 * 實驗參數配置
 * ========================================================================== */
#define SYNC_FREQ_HZ          5           // 廣播頻率 (5Hz)
#define SYNC_INTERVAL_MS      200         // 週期時間 (200ms)
#define TOTAL_NODES           4           // 節點總數 (S1, S2, S3, M1)
#define EXPERIMENT_MINS       5           // 單次實驗持續時間 (分鐘)
#define EXPERIMENT_ITERATIONS 6           // 🌟 新增：總共執行幾次 (6次 = 30分鐘)

// 計算單次循環預計總樣本數：5Hz * 60秒 * 5分鐘 = 1500
#define MAX_SAMPLES           (EXPERIMENT_MINS * 60 * SYNC_FREQ_HZ)

/* ==========================================================================
 * 通訊協議定義
 * ========================================================================== */
typedef enum {
    MSG_TYPE_READY = 0,   // Slave -> Master: 告知定位已完成
    MSG_TYPE_SYNC,        // Master -> Slave: 同步觸發脈衝
    MSG_TYPE_REPORT,      // Slave -> Master: 回傳接收當下的時間戳
    MSG_TYPE_POLL         // 🌟 新增：Trigger -> All: 詢問節點是否依然 READY
} sync_msg_type_t;

typedef struct {
    uint8_t  type;        // 封包類型
    uint32_t seq_num;     // 序號
    char     node_id[4];  // 節點名稱

    int64_t  tv_sec;      // 真實世界時間 (秒)
    int32_t  tv_usec;     // 真實世界時間 (微秒)
} __attribute__((packed)) sync_pkt_t;

#endif // SYNC_EXPERIMENT_DEFS_H