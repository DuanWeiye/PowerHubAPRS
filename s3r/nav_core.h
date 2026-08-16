// nav_core.h — GNSS+IMU 连续估计器核心（纯 C，无 Arduino 依赖）
//
// v2 返工：取代 fuse.ino v1 的锁存/DR/逃生门三个离散开关。0808 外场翻车的教训直接
// 写进结构（见 HANDOFF.md）：
//  1) GNSS 在场时永远是权威——IMU 不单独裁决位置（电车 accStd 0.19 < 人体静立 1.5，
//     振动幅度这个特征在"随身携带"下对"地理静止"不可分，任何"IMU 说静止就锁"都是错的）。
//  2) 无模式：停留收敛/移动跟踪/断档桥接全部从同一个 2D 匀速 KF 的协方差机制里连续
//     产生。没有开关就没有开关卡死、抖振（v1 的 ESC/ON 同毫秒循环）和切换跳变。
//  3) 诚实输出：推算点按位置 1σ 超"诚实界"即停发；安全额度随速度自动收缩
//     （v1 的"8s×车速=160m 漂移额度"这类与速度脱钩的定值在结构上不再可能）。
//
// IMU 的角色被数据限定为三件：陀螺航向角速率（断档时转动速度矢量）、高活动量
// （步行特征）加速"运动"判定/终止桥接、以及 P2 起的离线调参数据源。
//
// 同一份头被 test/replay.cpp 在主机上编译回放实测日志（先离线跑赢基线再上机），
// 也被 fuse.ino 在固件里实例化。全部经 NavCore* 显式传状态，无全局量。
#pragma once
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#define NAV_M_PER_DEG 111320.0

// ── 可调参数（回放器可扫参；固件用 navParamsDefault）─────────────────────────────
typedef struct {
    // 野点门（沿袭 PowerHub track.ino 验证过的语义：1s 尺度隐含速度上限 + 连拒强制放行）
    float    glMaxMps;        // 隐含速度超此即野点（m/s）
    uint32_t glForceMs;       // 连拒持续超此强制接受（重新对齐）
    uint32_t glResyncMs;      // 参照点超龄 → 放行并重建（KF 一并重置）
    // KF 量测/过程噪声
    float    posSigmaM;       // HDOP=1 时单次定位 1σ（米）
    float    accStdMove;      // 过程噪声：运动时假设加速度 1σ（m/s²）
    float    accStdStill;     // 停留收敛时的过程噪声下限（m/s²）
    double   nisGate;         // NIS 门限（2 自由度 χ²）
    double   initPosVar, initVelVar;
    double   reanchorM;       // 局部平面重锚定距离
    // 停留信念（still ∈ [0,1]，连续量，不是开关）
    float    stillSpdMax;     // KF 速度低于此才累积停留信念（m/s）
    float    stillInnovMax;   // 且量测创新距离低于此（米）
    float    stillEnterSec;   // 信念 0→1 的累积时间（秒）
    float    zuptOn;          // 信念超此 → 喂 v=0 伪量测
    float    zuptSigma;       // ZUPT 伪量测 1σ（m/s）
    float    moveSpdReset;    // KF 速度超此 → 信念清零（m/s）
    float    accStdActive;    // IMU 块 accStd 持续超此 → 视为"被携带"（步行量级）
    float    activeHoldSec;   // 上述持续时长门槛（秒）
    // 断档推算（coast）
    float    coastDecayPerS;  // 速度衰减率（1/s）
    float    sigmaEmitMaxM;   // 诚实界：位置 1σ 超此停止发推算点（米）
    uint32_t coastStillCapMs; // 停留保持推算的硬上限（毫秒，诚实兜底）
    float    crsMinMps;       // KF 速度低于此航向视为未知（m/s）
    // 0816 外场教训（进楼过渡 191m 幻影桥）：coast 的两个信任修正
    float    accStdCoastCarried; // "被携带"(步行)中 coast 的过程噪声——人转身/停步不可
                                 // 预测,σ 应比车辆滑行长得快尽早停发。0=沿用 accStdMove
    float    coastHdopRef;    // 断档前 hdop EMA 超此 → coast 初速按 ref/ema 折减
                              // （多径窗喂出的 KF 速度不配全额外推）。0=关闭
    float    coastHdopMax;    // 断档前 hdop EMA 超此 → 本段 coast 整段不发（外推资格
                              // 需要可信的近期状态）；停留保持(v=0 钉住)不是外推，不受限。
                              // 0=关闭
} NavParams;

static inline void navParamsDefault(NavParams* p) {
    p->glMaxMps       = 100.0f;
    p->glForceMs      = 4000;
    p->glResyncMs     = 30000;
    p->posSigmaM      = 4.0f;
    p->accStdMove     = 0.5f;     // P2 定版（0808 电车段调）：0.3 起点 A p90 21.6 输基线
                                  // 19.9；0.5 → 6.5/15.5 反超，B 双数据集改善，E 仍 0。
                                  // 代价：原始鬼影跳变不再被过度平滑吞掉（C 与基线 1:1）
    p->accStdStill    = 0.03f;
    p->nisGate        = 9.0;
    p->initPosVar     = 25.0;
    p->initVelVar     = 2500.0;
    p->reanchorM      = 3000.0;
    p->stillSpdMax    = 0.7f;
    p->stillInnovMax  = 8.0f;
    p->stillEnterSec  = 12.0f;
    p->zuptOn         = 0.8f;
    p->zuptSigma      = 0.1f;
    p->moveSpdReset   = 1.5f;
    p->accStdActive   = 1.0f;     // 0808 数据：步行/持握 ~1.5，电车巡航 ~0.19
    p->activeHoldSec  = 2.0f;
    p->coastDecayPerS = 0.03f;
    p->sigmaEmitMaxM  = 40.0f;
    p->coastStillCapMs= 120000;
    p->crsMinMps      = 0.83f;    // ≈3 km/h，与 PowerHub KF_COURSE_MIN_KMH 一致
    // 0816+0808 双数据集扫参定版（见 HANDOFF「P2 调参」）：
    // 逐桥分析：坏桥（0816 进楼 191.6m、0808 鬼影 113.2m）断档前 hdop 均 ≥6，
    // 好桥（电车 28s/37s、停留保持）均 ≤1.7 → 2.5 落在两群之间的空谷。
    p->accStdCoastCarried = 1.0f; // 步行(被携带)coast：人转身/停步不可预测，σ 加速越界
    p->coastHdopRef   = 0.0f;     // 关：门限已覆盖病灶，速度折减会给好桥引入低速偏差
    p->coastHdopMax   = 2.5f;     // 断档前质量差 → 整段禁发（外推资格门）
}

// ── 1D KF（位置+速度，两轴各一份；与 track.ino Kf1D 同构，实现同源以便对照）────────
typedef struct { double p, v, Pxx, Pxv, Pvv; } NavKf1;

static inline void navKf1Init(NavKf1* s, double posVar, double velVar) {
    s->p = 0; s->v = 0; s->Pxx = posVar; s->Pxv = 0; s->Pvv = velVar;
}
static inline void navKf1Predict(NavKf1* s, double dt, double accelVar) {
    double dt2 = dt * dt, dt3 = dt2 * dt, dt4 = dt3 * dt;
    s->p += s->v * dt;
    double PxxN = s->Pxx + 2.0 * dt * s->Pxv + dt2 * s->Pvv + accelVar * dt4 / 4.0;
    double PxvN = s->Pxv + dt * s->Pvv + accelVar * dt3 / 2.0;
    double PvvN = s->Pvv + accelVar * dt2;
    s->Pxx = PxxN; s->Pxv = PxvN; s->Pvv = PvvN;
}
static inline void navKf1Update(NavKf1* s, double z, double R) {
    double Sden = s->Pxx + R;
    double Kp = s->Pxx / Sden, Kv = s->Pxv / Sden;
    double innov = z - s->p;
    s->p += Kp * innov;
    s->v += Kv * innov;
    double PxxN = (1.0 - Kp) * s->Pxx;
    double PxvN = (1.0 - Kp) * s->Pxv;
    double PvvN = s->Pvv - Kv * s->Pxv;
    s->Pxx = PxxN; s->Pxv = PxvN; s->Pvv = PvvN;
}
// 速度伪量测（ZUPT：z_v=0，H=[0 1]）
static inline void navKf1UpdateVel(NavKf1* s, double zv, double R) {
    double Sden = s->Pvv + R;
    double Kp = s->Pxv / Sden, Kv = s->Pvv / Sden;
    double innov = zv - s->v;
    s->p += Kp * innov;
    s->v += Kv * innov;
    double PxxN = s->Pxx - Kp * s->Pxv;
    double PxvN = s->Pxv - Kp * s->Pvv;
    double PvvN = (1.0 - Kv) * s->Pvv;
    s->Pxx = PxxN; s->Pxv = PxvN; s->Pvv = PvvN;
}

// ── 核心状态 ─────────────────────────────────────────────────────────────────
typedef struct {
    NavParams prm;
    bool     have;                    // KF 已起算
    double   oLat, oLon, cosLat;      // 局部平面原点
    NavKf1   kx, ky;                  // X=东向 Y=北向（米）
    uint32_t tProp;                   // 上次预测推进的时刻
    uint32_t tFix;                    // 上次接受量测的时刻
    // 野点门
    double   glLat, glLon;
    uint32_t glMs, glRejSince;
    uint32_t tCand;                   // 最近一个 valid 候选（无论是否被拒）的时刻
    bool     glHave;
    bool     nisPrev;
    // 停留信念
    float    still;                   // 0..1
    int      innovHi;                 // 创新持续超界累计（漏桶）——GNSS 权威释放通道
    // IMU 特征（imu 块回调喂入；无 IMU 时保持 0/false 即自动退化为纯 GNSS KF）
    float    accStd, headRateDps;
    uint32_t tImu, tActiveSince;      // tActiveSince=持续高活动段起点（0=无）
    // coast 记账
    uint32_t tCoastStill0;            // 停留保持推算的起点（诚实硬上限用）
    float    hdopEma;                 // 接受量测的 hdop 指数均值（τ≈8s）——coast 信任修正用
    bool     inCoast;                 // 当前处于 coast 段（初速折减只在进入时做一次）
    bool     coastBlocked;            // 本段 coast 因断档前质量差被整段禁发（停留保持除外）
    // 首拍速度过渡（KF 速度未收敛时用模组自报值，同 track.ino 语义）
    bool     firstEpoch;
} NavCore;

// 每个输出点的完整描述（固件据此改写 NMEA；回放据此落 CSV）
typedef struct {
    bool   emit;        // false = 无可发（无定位且推算超界）——如实透传原句
    bool   est;         // true  = 推算点（GGA quality=6 / RMC mode=E）
    double lat, lon;
    float  spdMps;
    float  crsDeg;      // crsValid=false 时无意义
    bool   crsValid;
    float  sigmaM;      // 位置 1σ（米，两轴取大）——遥测/诚实界用
    float  still;       // 停留信念（遥测用）
} NavOut;

static inline void navInit(NavCore* n, const NavParams* p) {
    memset(n, 0, sizeof(*n));
    n->prm = *p;
}

// IMU 特征块（固件 0.25s/块直接喂；无 IMU 或 IMU 故障就从不调用，核心自动退化）
static inline void navImu(NavCore* n, uint32_t ms, float accStd, float headRateDps) {
    n->accStd = accStd;
    n->headRateDps = headRateDps;
    n->tImu = ms;
    if (accStd > n->prm.accStdActive) {
        if (!n->tActiveSince) n->tActiveSince = ms;
    } else {
        n->tActiveSince = 0;
    }
}

// "被携带中"：IMU 高活动量已持续 activeHoldSec（步行/持握量级；电车巡航到不了这阈值）
static inline bool navCarried(const NavCore* n, uint32_t ms) {
    return n->tActiveSince &&
           (ms - n->tActiveSince) >= (uint32_t)(n->prm.activeHoldSec * 1000.0f);
}

static inline void navLlToXy(const NavCore* n, double lat, double lon, double* x, double* y) {
    *x = (lon - n->oLon) * n->cosLat * NAV_M_PER_DEG;
    *y = (lat - n->oLat) * NAV_M_PER_DEG;
}
static inline void navXyToLl(const NavCore* n, double x, double y, double* lat, double* lon) {
    *lat = n->oLat + y / NAV_M_PER_DEG;
    *lon = n->oLon + x / (n->cosLat * NAV_M_PER_DEG);
}
static inline float navDistM(double lat1, double lon1, double lat2, double lon2) {
    float dN = (float)((lat2 - lat1) * NAV_M_PER_DEG);
    float dE = (float)((lon2 - lon1) * NAV_M_PER_DEG * cos(lat1 * M_PI / 180.0));
    return sqrtf(dN * dN + dE * dE);
}

static inline void navReset(NavCore* n, double lat, double lon) {
    n->oLat = lat; n->oLon = lon;
    n->cosLat = cos(lat * M_PI / 180.0);
    navKf1Init(&n->kx, n->prm.initPosVar, n->prm.initVelVar);
    navKf1Init(&n->ky, n->prm.initPosVar, n->prm.initVelVar);
    n->have = true;
    n->nisPrev = false;
    n->still = 0;
    n->tCoastStill0 = 0;
    n->firstEpoch = true;
}

// 预测推进到 ms（量测/coast 共用）。coast 时才转动速度矢量（有量测时由量测牵引）。
static inline void navPredictTo(NavCore* n, uint32_t ms, bool coasting) {
    if (!n->have) return;
    double dt = (ms - n->tProp) / 1000.0;
    if (dt <= 0) return;
    if (dt > 5.0) dt = 5.0;                       // 单步上限：长静默由 resync 路径处理
    n->tProp = ms;
    // 过程噪声在 log 空间随停留信念内插：still→accStdStill（收敛成点），动→accStdMove
    float s = n->still;
    double accStd = expf((1.0f - s) * logf(n->prm.accStdMove) + s * logf(n->prm.accStdStill));
    // coast 中检测到"被携带"→ 用运动档（或专用更大档）：σ 快速增长，尽快越过诚实界停发
    if (coasting && navCarried(n, ms))
        accStd = n->prm.accStdCoastCarried > 0 ? n->prm.accStdCoastCarried
                                               : n->prm.accStdMove;
    double accVar = accStd * accStd;
    if (coasting) {
        // 速度矢量按陀螺航向角速率转动（顺时针正=罗盘方向），并衰减（不确定性递增的保守化）
        double w = -n->headRateDps * M_PI / 180.0 * dt;   // 罗盘顺时针 = 数学角负向
        double c = cos(w), sn = sin(w);
        double vx = n->kx.v * c - n->ky.v * sn;
        double vy = n->kx.v * sn + n->ky.v * c;
        double decay = 1.0 - n->prm.coastDecayPerS * dt;
        if (decay < 0) decay = 0;
        n->kx.v = vx * decay; n->ky.v = vy * decay;
        if (n->still >= n->prm.zuptOn) { n->kx.v = 0; n->ky.v = 0; }  // 停留保持：位置钉住
    }
    navKf1Predict(&n->kx, dt, accVar);
    navKf1Predict(&n->ky, dt, accVar);
}

// 野点门（track.ino 同语义）：true=丢弃本候选
static inline bool navGlitch(NavCore* n, double lat, double lon, uint32_t ms) {
    if (!n->glHave) return false;
    uint32_t dtMs = ms - n->glMs;
    if (dtMs < 1000) dtMs = 1000;                 // 1s 下限：与采样率解耦
    if (dtMs >= n->prm.glResyncMs) return false;  // 参照超龄 → 放行重建
    float vms = navDistM(lat, lon, n->glLat, n->glLon) / (dtMs / 1000.0f);
    if (vms <= n->prm.glMaxMps) return false;
    if (!n->glRejSince) n->glRejSince = ms;
    else if (ms - n->glRejSince >= n->prm.glForceMs) return false;   // 认账重对齐
    return true;
}

static inline void navFillOut(NavCore* n, NavOut* o, bool est,
                              float rawSpdMps, float rawCrsDeg, bool haveRaw) {
    navXyToLl(n, n->kx.p, n->ky.p, &o->lat, &o->lon);
    o->emit = true;
    o->est  = est;
    if (n->firstEpoch && haveRaw) {               // 首拍 KF 速度未收敛：模组自报值过渡
        o->spdMps  = rawSpdMps;
        o->crsValid = rawCrsDeg >= 0;
        o->crsDeg  = rawCrsDeg < 0 ? 0 : rawCrsDeg;
    } else {
        float vE = (float)n->kx.v, vN = (float)n->ky.v;
        o->spdMps = sqrtf(vE * vE + vN * vN);
        o->crsValid = o->spdMps >= n->prm.crsMinMps;
        if (o->crsValid) {
            o->crsDeg = atan2f(vE, vN) * 180.0f / (float)M_PI;
            if (o->crsDeg < 0) o->crsDeg += 360.0f;
        } else o->crsDeg = 0;
    }
    double pmax = n->kx.Pxx > n->ky.Pxx ? n->kx.Pxx : n->ky.Pxx;
    o->sigmaM = (float)sqrt(pmax > 0 ? pmax : 0);
    o->still  = n->still;
}

// ── 主入口 1：一个 GNSS 定位候选（GGA 解析结果）。返回 false=本核心不出点（透传）。──
static inline bool navGnss(NavCore* n, uint32_t ms, double lat, double lon,
                           float hdop, float rawSpdMps, float rawCrsDeg, NavOut* out) {
    out->emit = false;
    n->tCand = ms;
    // 参照超龄/强制放行 → KF 重置从新点起算（避免拖影/速度被踢飞，track.ino 同策略）
    bool forced = n->glRejSince && (ms - n->glRejSince >= n->prm.glForceMs);
    bool stale  = n->glHave && (ms - n->glMs >= n->prm.glResyncMs);
    if (navGlitch(n, lat, lon, ms)) {             // 野点：丢弃（保持上个好状态）
        return false;                             // 调用方：本句按 coast 处理或原样转发
    }
    // 量测被接受：hdop 质量 EMA（τ≈8s，按拍距时变系数）+ 退出 coast 段
    {
        float h = hdop > 0.5f ? hdop : 0.5f;
        if (n->hdopEma <= 0) n->hdopEma = h;
        else {
            float a = 1.0f - expf(-(float)(ms - n->tFix) / 8000.0f);
            if (a < 0.02f) a = 0.02f;
            if (a > 1.0f)  a = 1.0f;
            n->hdopEma += a * (h - n->hdopEma);
        }
        n->inCoast = false;
    }
    if (!n->have || forced || stale) {
        navReset(n, lat, lon);
        n->tProp = ms; n->tFix = ms;
        n->glLat = lat; n->glLon = lon; n->glMs = ms;
        n->glHave = true; n->glRejSince = 0;
        navFillOut(n, out, false, rawSpdMps, rawCrsDeg, true);
        n->firstEpoch = false;
        return true;
    }
    n->glLat = lat; n->glLon = lon; n->glMs = ms; n->glRejSince = 0;

    navPredictTo(n, ms, false);
    n->tFix = ms;
    n->tCoastStill0 = 0;

    double x, y;
    navLlToXy(n, lat, lon, &x, &y);
    double sigma = (double)n->prm.posSigmaM * (hdop > 1.0f ? hdop : 1.0f);
    double R = sigma * sigma;
    // NIS two-strike（track.ino 0731 仿真验证的语义：压孤立离群，不拖真机动）
    double ix = x - n->kx.p, iy = y - n->ky.p;
    double innovM = sqrt(ix * ix + iy * iy);
    double nis = ix * ix / (n->kx.Pxx + R) + iy * iy / (n->ky.Pxx + R);
    bool nisHit = nis > n->prm.nisGate;
    if (nisHit && !n->nisPrev) R *= nis / n->prm.nisGate;
    n->nisPrev = nisHit;
    navKf1Update(&n->kx, x, R);
    navKf1Update(&n->ky, y, R);

    // 停留信念更新（连续量）：GNSS 是主证据——KF 速度小且创新小才累积；
    // 强运动证据（KF 速度大 / 连续 NIS / IMU 高活动持续）清零。
    float spd = sqrtf((float)(n->kx.v * n->kx.v + n->ky.v * n->ky.v));
    float dtS = 0.2f;                             // 5Hz 量测拍（用于信念积分步长）
    // GNSS 权威释放通道：ZUPT 会把 KF 速度钳在 0，"速度大才释放"可能结构性死锁——
    // 量测创新持续超界（漏桶累计 ~1.5s）说明 GNSS 与"停留"矛盾，信念直接清零。
    if (innovM > n->prm.stillInnovMax) { if (n->innovHi < 99) n->innovHi += 1; }
    else                               { n->innovHi -= 2; if (n->innovHi < 0) n->innovHi = 0; }
    if (spd > n->prm.moveSpdReset || (nisHit && n->nisPrev) || navCarried(n, ms)
            || n->innovHi >= 6) {
        n->still = 0;
    } else if (spd < n->prm.stillSpdMax && innovM < n->prm.stillInnovMax) {
        n->still += dtS / n->prm.stillEnterSec;
        if (n->still > 1.0f) n->still = 1.0f;
    } else {
        n->still -= dtS / n->prm.stillEnterSec;   // 中间带：缓慢泄放
        if (n->still < 0) n->still = 0;
    }
    if (n->still >= n->prm.zuptOn) {              // ZUPT：v=0 伪量测（连续，不是开关）
        double Rz = (double)n->prm.zuptSigma * n->prm.zuptSigma;
        navKf1UpdateVel(&n->kx, 0, Rz);
        navKf1UpdateVel(&n->ky, 0, Rz);
    }

    // 重锚定（长距离累积后保持局部平面近似有效）
    if (fabs(n->kx.p) > n->prm.reanchorM || fabs(n->ky.p) > n->prm.reanchorM) {
        double nl, no_;
        navXyToLl(n, n->kx.p, n->ky.p, &nl, &no_);
        n->oLat = nl; n->oLon = no_;
        n->cosLat = cos(nl * M_PI / 180.0);
        n->kx.p = 0; n->ky.p = 0;
    }
    navFillOut(n, out, false, rawSpdMps, rawCrsDeg, true);
    n->firstEpoch = false;
    return true;
}

// ── 主入口 2：无定位拍（GGA invalid / 野点被丢）→ 推算。返回 false=超诚实界，停发。──
static inline bool navCoast(NavCore* n, uint32_t ms, NavOut* out) {
    out->emit = false;
    if (!n->have) return false;
    // 野点门连拒窗口 = "GNSS 在场但与状态矛盾"的歧义窗，不是真断档——不发推算点
    // （0808 回放：鬼影段拒绝窗里 coast 续发造成 6 拍 >150m 伪点）。1.5s 内没有新候选
    // 说明矛盾源消失，恢复真断档语义。
    if (n->glRejSince && ms - n->tCand <= 1500) return false;
    if (n->still >= n->prm.zuptOn) {              // 停留保持：记账硬上限
        if (!n->tCoastStill0) n->tCoastStill0 = ms;
        if (ms - n->tCoastStill0 > n->prm.coastStillCapMs) return false;
    }
    // 进入 coast 的第一拍：断档前定位质量差（hdop EMA 高）→ 初速折减一次。
    // 多径窗（0816 进楼：hdop 4.5-12.8 的鬼影步进把 KF 速度打到 3-4m/s）喂出的
    // 速度不配全额外推；质量好时 factor=1 无损。
    if (!n->inCoast) {
        n->inCoast = true;
        n->coastBlocked = n->prm.coastHdopMax > 0 && n->hdopEma > n->prm.coastHdopMax;
        if (n->prm.coastHdopRef > 0 && n->hdopEma > n->prm.coastHdopRef) {
            float f = n->prm.coastHdopRef / n->hdopEma;
            n->kx.v *= f; n->ky.v *= f;
        }
    }
    navPredictTo(n, ms, true);
    if (n->coastBlocked && n->still < n->prm.zuptOn) return false;
    double pmax = n->kx.Pxx > n->ky.Pxx ? n->kx.Pxx : n->ky.Pxx;
    if (sqrt(pmax) > n->prm.sigmaEmitMaxM) return false;   // 诚实界
    navFillOut(n, out, true, 0, -1, false);
    return true;
}
