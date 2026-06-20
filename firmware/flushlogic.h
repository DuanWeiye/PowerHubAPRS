// flushlogic.h — 配置B 存转日志的"何时切 LTE 上传"纯决策 + 调参常量。
// 无 Arduino 依赖（只用 <stdint.h>），故固件(flashlog.ino / config_b.ino)与
// PC 端仿真(sim/)共用同一份，保证核心判定单一真相源、可在 PC 上被测试。
#pragma once
#include <stdint.h>

// 段：满 FL_SEG_POINTS 点即封段。配置A 节奏(最快受 MIN_TX_GAP=30s 地板限制)下，
// 20 点 ≈ 最快 10 分钟一段；慢速/停车则久得多。
static const uint16_t FL_SEG_POINTS = 20;

// 触发"切 LTE 上传积压"的廉价时刻阈值——只在 GPS 瞎掉也不心疼时切。
static const uint32_t FL_STILL_MS  = 300000UL;  // 静止 ≥ 5 分钟（真停下，非等红灯）
static const uint32_t FL_NOFIX_MS  = 300000UL;  // 无定位 ≥ 5 分钟（室内/隧道，GPS 已瞎）
static const float    FL_STILL_KMH  = 2.5f;     // ≤ 此速视为静止（同 SB_LOW_SPEED_KMH）

// 单个 SHREQ 最多带几个点。设为 ≥ FL_SEG_POINTS，使一个段一次 SHREQ 发完
// （20 点 ≈ 2.3KB；实测 SIM7080G SHCONF BODYLEN 上限 4096，故用 4096）。
static const uint16_t FL_BATCH    = 20;
static const int      FL_BODYLEN  = 4096;   // 段上传的 SHCONF BODYLEN（实测上限）

// flush 调度器的输入快照。
struct FlushInputs {
    uint16_t sealedSegs;   // 已封段数（满段 / 上传前临封的半段）；上传的最小积压单位
    uint32_t stillMs;      // 连续静止时长（有定位且低速；0 = 当前非静止）
    uint32_t noFixMs;      // 连续无定位时长（0 = 当前有定位）
    bool     forced;       // 手动强制（长按大按钮）——主动选时机上传
};

// 是否该切 LTE 上传积压。纯函数：固件与仿真共用，PC 上可直接断言。
//   规则：长按无条件传；否则必须"至少攒满 1 个段"且命中一个廉价时刻(静止/无定位)。
//   无冷却——封一个新段最快也要 ~10min，本身就是上传频率的天然下限。
static inline bool flushDue(const FlushInputs& in) {
    if (in.forced) return true;
    if (in.sealedSegs < 1) return false;
    if (in.stillMs >= FL_STILL_MS) return true;
    if (in.noFixMs >= FL_NOFIX_MS) return true;
    return false;
}
