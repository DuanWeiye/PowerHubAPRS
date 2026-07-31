// track.ino — 轨迹点格式化 + 存转(store-and-forward)队列 + 自适应 beacon 决策
//            + GNSS 野点过滤/卡尔曼平滑流水线 + 统一位置访问器（本文件内 A/B 共用一份实现）
// 配置A/B 共用。位置一律经 fixXxx() 访问器读取，两边的原始定位源(TinyGPS++ / CGNSINF)都通过
// gnssFeedLiveFix() 接进同一套过滤+平滑逻辑，写共享的 liveFix——不重复实现两份一样的东西。
#include "defs.h"

// ── 统一位置访问器 —— 配置A/B 共用一份实现，读野点过滤+卡尔曼平滑之后的 liveFix ─────────
static inline bool    fixHasLoc()    { return liveFix.valid; }
static inline double  fixLat()       { return liveFix.lat; }
static inline double  fixLon()       { return liveFix.lon; }
static inline float   fixAltM()      { return liveFix.altM; }
static inline bool    fixHasSpeed()  { return liveFix.valid; }
static inline float   fixSpdKmh()    { return liveFix.spdKmh; }
static inline bool    fixHasHdop()   { return liveFix.valid; }
static inline float   fixHdop()      { return liveFix.hdop; }
static inline bool    fixHasSats()   { return liveFix.valid; }
static inline uint8_t fixSats()      { return liveFix.sats; }
static inline bool    fixHasCourse() { return liveFix.courseValid; }
static inline float   fixCourseDeg() { return liveFix.courseDeg; }

// Format one track point as a JSON object into buf; returns the length written.
static int fmtPoint(char* buf, int cap, const TrackPoint& p) {
    return snprintf(buf, cap,
        "{\"lat\":%.7f,\"lon\":%.7f,\"alt\":%d,\"spd\":%u,\"sat\":%u,"
        "\"hdop\":%.1f,\"bat_mv\":%u,\"bat_pct\":%u,\"ts\":%lu}",
        p.lat / 1e7, p.lon / 1e7, p.alt, p.spd, p.sat,
        p.hdop / 10.0, p.bat_mv, p.bat_pct, (unsigned long)p.ts);
}

// Snapshot the current GPS fix + battery into a compact TrackPoint.
static void buildTrackPoint(TrackPoint& p) {
    time_t now = time(nullptr);
    p.ts  = (now > 1735689600L) ? (uint32_t)now : 0;     // >2025-01-01 → RTC synced
    p.lat = (int32_t)lround(fixLat() * 1e7);
    p.lon = (int32_t)lround(fixLon() * 1e7);
    p.alt = (int16_t)fixAltM();
    float spd = fixHasSpeed() ? fixSpdKmh() : 0.0f;
    p.spd = spd > 255.0f ? 255 : (uint8_t)spd;
    p.sat = fixHasSats() ? fixSats() : 0;
    float h = fixHasHdop() ? fixHdop() : 25.5f;
    p.hdop = h * 10.0f > 255.0f ? 255 : (uint8_t)(h * 10.0f);
    p.bat_mv  = phVolt(VM_BAT);
    p.bat_pct = (uint8_t)batPct;
}

// 存转（断网积压）已统一到 flashlog.ino 的 LittleFS 段日志（断电不丢）：
//   · 失败入队 = flashLogAppend(p)；批量补发 = flashLogUpload()。
//   · 旧的 RAM 环形队列(trackEnqueue/trackFlush)已删除。

// ═══════════════════════════════════════════════════════════════════════════
// GNSS 定位质量流水线（配置A/B 共用）：野点过滤 + 二维匀速卡尔曼平滑(带 NIS 门)
// ═══════════════════════════════════════════════════════════════════════════
// 症状：个别采样把定位甩到几公里外(东京湾/太平洋)，下一次又跳回来。成因是单次 GNSS
// 解算的多径/坏星历野值，不是真移动。判据 = "与上一个已接受定位之间的隐含速度"：
// 距离 / 时间 超过 GLITCH_MAX_MPS（任何交通方式都达不到）即判野点丢弃。
// 三条兜底保证"通用、不锁死"：①无参照(首个定位)放行；②参照太旧(丢过定位、参照已失效)
// 放行并重建；③连续丢弃持续超时后强制接受最新点重新对齐（真的远距离移动最终也跟得上）。
// 全程不引用任何固定坐标，不对某片海域/某个用例特判——对一般输入成立。
static double   glLastLat = 0, glLastLon = 0;   // 上一个"已接受"定位（参照点）
static uint32_t glLastMs  = 0;                  // 参照点的 millis
static bool     glHave    = false;              // 是否已有参照
static uint32_t glRejSinceMs = 0;               // 当前连拒段的起点 millis（0=不在连拒中）

// ── 二维匀速卡尔曼滤波状态 —— 配置A/B 共用，替代原先的中位数+EMA 两级平滑 ───────────
// 0621 实测：HDOP/星数都"好"的连续快速移动区间仍会插入 1 个孤立跳变点(11:56:20 一例)，
// 加了中位数滤波压住。0711 实测又发现两个新问题——都是同一个根子：
//  ① 步行左右飘：19:15-20:39 全程 spd=0~8km/h(真步行速度)、HDOP 0.8~1.0、星数10~14都"好"，
//     EMA 已经全程按最强档在压，纬度序列仍有几米级来回摆动。查明是多径造成的位置偏差和
//     "人走路的真实缓慢位移"处在同一时间尺度(几十秒量级)——纯时间域低通/中位数滤波器
//     结构上分不清两者，继续调平滑系数只是在"压噪声"和"转弯追不上"之间来回拉扯。
//  ② 拐点飞出去：电车拐弯那几拍 CGNSINF 的 spd 字段抽风连续报 0(模块内部速度解与位置解不
//     同步，属已知现象)，原 EMA 拿 spd 选平滑档位，被抽风误判成"慢走静止"套最强平滑，输出
//     粘在拐弯前位置不追，等 spd 恢复正常后单步跳出 ~2km——错的是"用 spd 字段选平滑力度"
//     这个设计本身，不是某个阈值没调好。
// 卡尔曼滤波从架构上同时解决两者：显式维护"位置+速度"两个状态，不依赖任何模组自报字段。
// 多径偏差相对"当前速度朝当前方向匀速前进"的预测是无规律的高频扰动，被滤波器自然压低权重
// (对应步行时的压噪声效果)；真实的转弯/加速会体现为持续几拍的一致性残差，滤波器据此快速
// 调整速度状态跟上(对应拐点场景不再滞后)——噪声抑制和转弯响应不再是同一个旋钮的两端。
static bool     kfHave      = false;
static bool     kfNisPrevHit = false;           // 上一拍是否触发过 NIS 门（two-strike 判据用）
static Kf1D     kfX, kfY;                       // X=东向, Y=北向，局部平面坐标，米
static double   kfOriginLat = 0, kfOriginLon = 0, kfCosLat = 1.0;  // 局部平面原点(经纬度换算基准)
static uint32_t kfLastMs    = 0;
static const double KF_M_PER_DEG = 111320.0;    // 纬度方向 米/度；经度方向再乘 cos(纬度)

// 返回 true = 这个候选定位是野点，应丢弃（保留上一个好点）。
static bool gnssIsGlitch(double lat, double lon, uint32_t nowMs) {
    if (!glHave) return false;                          // 无参照 → 放行（调用方随后 accept 建参照）
    uint32_t dtMs = nowMs - glLastMs;
    if (dtMs < 1000) dtMs = 1000;                       // dt 下限 1s：门语义与采样率解耦——5Hz 下
                                                        // 相邻样本 0.2s，几十米多径步进按 0.2s 算会
                                                        // 虚高成>100m/s 被误杀(那是 NIS 门的活)；本门
                                                        // 只拦"1s 尺度上仍超 100m/s"的千米级跳变
    if (dtMs >= GLITCH_RESYNC_MS) return false;         // 参照太旧 → 无从判断，放行重建
    double dist = TinyGPSPlus::distanceBetween(lat, lon, glLastLat, glLastLon);  // 米
    float  vms  = (float)(dist / (dtMs / 1000.0));      // 隐含速度 m/s
    if (vms <= GLITCH_MAX_MPS) return false;            // 速度可信 → 正常点
    if (glRejSinceMs == 0) glRejSinceMs = nowMs;        // 连拒段开始计时
    else if (nowMs - glRejSinceMs >= GLITCH_FORCE_MS)   // 连拒超时 → 认账，强制重新对齐
        return false;
    return true;                                        // 判为野点 → 丢弃
}

// 一个定位被"接受"后调用：更新参照点、清连续丢弃计数。两种情况下卡尔曼滤波状态一并重置，
// 从新点重新起算（等同"无参照"时的处理）：
//  ① 参照太旧(丢过定位/长时间静默)——避免"重新捕获到的真实新位置"被旧位置拖着走出一条
//     虚假的缓慢轨迹(拖影)；
//  ② 连拒超时后的强制放行(连拒段已持续 GLITCH_FORCE_MS)——此时位置已实际跳远，若不重置，
//     KF 会吃一个公里级 innovation：位置要拖好几拍才追上(拖影)，速度状态更会被 Kv·innov
//     踢出上百 m/s，随后几拍预测过冲振荡。既然决定"认账重新对齐"，滤波器也要一起从新位置起算。
static void gnssAcceptFix(double lat, double lon, uint32_t nowMs) {
    bool forced = (glRejSinceMs != 0) && (nowMs - glRejSinceMs >= GLITCH_FORCE_MS);
    if (glHave && ((nowMs - glLastMs) >= GLITCH_RESYNC_MS || forced)) {
        kfHave = false;
    }
    glLastLat = lat; glLastLon = lon; glLastMs = nowMs;
    glHave = true;   glRejSinceMs = 0;
}

// ── 二维匀速卡尔曼滤波实现（状态声明见上方） ──────────────────────────────────────
static void llToLocalXY(double lat, double lon, double* x, double* y) {
    *x = (lon - kfOriginLon) * kfCosLat * KF_M_PER_DEG;
    *y = (lat - kfOriginLat) * KF_M_PER_DEG;
}
static void localXYToLL(double x, double y, double* lat, double* lon) {
    *lat = kfOriginLat + y / KF_M_PER_DEG;
    *lon = kfOriginLon + x / (kfCosLat * KF_M_PER_DEG);
}

// 匀速模型预测：位置按当前速度推进 dt，协方差按白噪声加速度模型(方差=accelVar)增长。
static void kf1dPredict(Kf1D& s, double dt, double accelVar) {
    double dt2 = dt * dt, dt3 = dt2 * dt, dt4 = dt3 * dt;
    s.p += s.v * dt;
    double PxxN = s.Pxx + 2.0 * dt * s.Pxv + dt2 * s.Pvv + accelVar * dt4 / 4.0;
    double PxvN = s.Pxv + dt * s.Pvv + accelVar * dt3 / 2.0;
    double PvvN = s.Pvv + accelVar * dt2;
    s.Pxx = PxxN; s.Pxv = PxvN; s.Pvv = PvvN;
}
// 用新的位置测量 z(米) 修正状态，R = 测量方差(米²，按 HDOP 缩放)。
static void kf1dUpdate(Kf1D& s, double z, double R) {
    double Sden = s.Pxx + R;
    double Kp = s.Pxx / Sden, Kv = s.Pxv / Sden;
    double innov = z - s.p;
    s.p += Kp * innov;
    s.v += Kv * innov;
    double PxxN = (1.0 - Kp) * s.Pxx;
    double PxvN = (1.0 - Kp) * s.Pxv;
    double PvvN = s.Pvv - Kv * s.Pxv;
    s.Pxx = PxxN; s.Pxv = PxvN; s.Pvv = PvvN;
}

// 主入口：喂一个"已过野点门"的候选定位，吐出卡尔曼滤波后的经纬度。
static void gnssKfSmooth(double lat, double lon, float hdop, uint32_t nowMs, double* outLat, double* outLon) {
    if (!kfHave) {
        kfOriginLat = lat; kfOriginLon = lon;
        kfCosLat = cos(radians(lat));
        kfX = (Kf1D){0, 0, KF_INIT_POS_VAR, 0, KF_INIT_VEL_VAR};
        kfY = (Kf1D){0, 0, KF_INIT_POS_VAR, 0, KF_INIT_VEL_VAR};
        kfLastMs = nowMs;
        kfHave = true;
        kfNisPrevHit = false;
        *outLat = lat; *outLon = lon;
        return;
    }
    double dt = (nowMs - kfLastMs) / 1000.0;
    if (dt <= 0) dt = 0.001;
    kfLastMs = nowMs;
    double accelVar = (double)KF_ACCEL_STD_MPS2 * KF_ACCEL_STD_MPS2;
    kf1dPredict(kfX, dt, accelVar);
    kf1dPredict(kfY, dt, accelVar);
    double x, y;
    llToLocalXY(lat, lon, &x, &y);
    double sigma = (double)KF_POS_SIGMA_M * (hdop > 1.0f ? hdop : 1.0f);
    double R = sigma * sigma;
    // innovation 一致性门(NIS，two-strike)：野点门只拦"隐含速度>GLITCH_MAX_MPS"的千米级
    // 跳变，几十米级的多径跳变(1s 内隐含速度<100m/s)过得了野点门，却会全权重进滤波把状态
    // 拽歪。这里算归一化 innovation(2自由度χ²)，超阈就按倍数膨胀 R 降权——但只压"孤立"
    // 离群：上一拍也触发过门就全权重放行(two-strike)。因为持续超阈说明不是多径尖刺而是
    // 真机动(急转弯/加减速)，继续降权会拖着滤波不让它跟。0731 仿真：50m 孤立跳变的输出
    // 偏移 17.2m→2.5m(压制有效)；15m/s 曲线急弯偏差 two-strike=17.8m 与不加门持平，
    // 而"常开门"版恶化到 48.7m——two-strike 是"压离群"与"跟机动"兼得的关键。
    double ix = x - kfX.p, iy = y - kfY.p;
    double nis = ix * ix / (kfX.Pxx + R) + iy * iy / (kfY.Pxx + R);
    bool nisHit = (nis > KF_NIS_GATE);
    if (nisHit && !kfNisPrevHit) R *= nis / KF_NIS_GATE;
    kfNisPrevHit = nisHit;
    kf1dUpdate(kfX, x, R);
    kf1dUpdate(kfY, y, R);
    // 离原点太远(长时间连续移动累积)时重新锚定坐标系，状态原地平移，不引入误差。
    if (fabs(kfX.p) > KF_REANCHOR_DIST_M || fabs(kfY.p) > KF_REANCHOR_DIST_M) {
        double newLat, newLon;
        localXYToLL(kfX.p, kfY.p, &newLat, &newLon);
        kfOriginLat = newLat; kfOriginLon = newLon;
        kfCosLat = cos(radians(kfOriginLat));
        kfX.p = 0; kfY.p = 0;
    }
    localXYToLL(kfX.p, kfY.p, outLat, outLon);
}

// ── 统一入口：野点过滤 + 卡尔曼平滑 → 写共享 liveFix —— 配置A/B 共用 ─────────────────
// A(TinyGPS++ 连续 NMEA)/B(CGNSINF 轮询) 各自的原始定位候选都喂到这里，不重复实现两份
// 一样的过滤逻辑。可选字段(course/hdop/sats)用 haveXxx 标出"这次候选是否带这个信息"，
// 不带就退化成"未知"，不假装有数据。
// 速度/航向从 KF 速度状态导出，不再透传模组自报字段——CGNSINF 的 spd 实测会在高速移动中
// 抽风报 0(曾把 EMA 误导进最强平滑、拐弯单步跳 2km)，NMEA 的 VTG/RMC 速度低速下也噪；
// KF 速度与滤波后的位置自洽且已平滑。仅 KF (重新)起算的第一拍速度尚未收敛(初始为 0)，
// 用模组自报值过渡一拍，2-3 拍后 KF 即收敛（初始速度方差给得大，见 KF_INIT_VEL_VAR）。
static float altEmaM = 0.0f;   // 高度一维 EMA 状态（随 KF 一起在第一拍重置）
static void gnssFeedLiveFix(double lat, double lon, uint32_t nowMs, float altM,
                             bool haveSpd, float spdKmh, bool haveCourse, float courseDeg,
                             bool haveHdop, float hdop, bool haveSats, uint8_t sats) {
    float nhdop = haveHdop ? hdop : 25.5f;
    if (gnssIsGlitch(lat, lon, nowMs)) {
        Serial.printf("[GPS] reject glitch lat=%.6f lon=%.6f (impossible jump)\n", lat, lon);
        if (haveSats) liveFix.sats = sats;   // 只更新可见星/HDOP(现场判断用)
        liveFix.hdop = nhdop;
        return;                              // liveFix 位置/valid/tMs 全保持上一个好点
    }
    gnssAcceptFix(lat, lon, nowMs);          // 内部可能置 kfHave=false(参照过旧/强制放行)
    bool kfFirst = !kfHave;                  // 本次是否为 KF (重新)起算的第一拍
    double outLat, outLon;
    gnssKfSmooth(lat, lon, nhdop, nowMs, &outLat, &outLon);
    float outSpd, outCrs;
    bool  crsValid;
    if (kfFirst) {                           // 第一拍：KF 速度还是 0，退回模组自报值过渡
        outSpd   = haveSpd ? spdKmh : 0.0f;
        crsValid = haveCourse;
        outCrs   = haveCourse ? courseDeg : -1.0f;
    } else {                                 // 稳态：速度/航向都从 KF 速度矢量(东/北)导出
        float vE = (float)kfX.v, vN = (float)kfY.v;
        outSpd   = sqrtf(vE * vE + vN * vN) * 3.6f;       // m/s → km/h
        crsValid = (outSpd >= KF_COURSE_MIN_KMH);         // 慢速下方向是噪声，标未知
        if (crsValid) {
            outCrs = atan2f(vE, vN) * RAD_TO_DEG;         // atan2(东,北) = 罗盘方位角
            if (outCrs < 0) outCrs += 360.0f;
        } else {
            outCrs = -1.0f;
        }
    }
    // 高度：单点噪声 ±10m 级、动态远比水平慢，轻量 EMA 低通即可，不值得进 KF
    altEmaM = kfFirst ? altM : altEmaM + ALT_EMA_ALPHA * (altM - altEmaM);
    liveFix.valid       = true;
    liveFix.lat         = outLat;
    liveFix.lon         = outLon;
    liveFix.altM        = altEmaM;
    liveFix.spdKmh      = outSpd;
    liveFix.courseValid = crsValid;
    liveFix.courseDeg   = outCrs;
    liveFix.hdop        = nhdop;
    liveFix.sats        = haveSats ? sats : 0;
    liveFix.tMs         = nowMs;
}

// ═══════════════════════════════════════════════════════════════════════════
// Adaptive beaconing decision (SmartBeaconing + decay + corner pegging)
// ═══════════════════════════════════════════════════════════════════════════

// Snapshot the just-sent fix as the new anchor (position + course + time).
static void recordAnchor() {
    tLastSend = millis();
    if (fixHasLoc()) {
        lastTxLat  = fixLat();
        lastTxLon  = fixLon();
        haveAnchor = true;
    }
    lastTxCourse = fixHasCourse() ? fixCourseDeg() : -1.0;
}

// Decide whether a beacon is due right now. Call only with a good GPS fix.
// On true, *why points to a short reason string and *stopped tells the caller
// whether we're in the stopped (decay) regime, so it can grow/reset the decay.
static bool beaconDue(const char** why, bool* stopped) {
    uint32_t now     = millis();
    uint32_t elapsed = now - tLastSend;
    float    spd     = fixHasSpeed() ? fixSpdKmh() : 0.0f;
    bool     hdopOk  = fixHasHdop() && fixHdop() < HDOP_MAX
                       && fixHasSats() && fixSats() >= SAT_MIN;
    *stopped = (spd < SB_LOW_SPEED_KMH);

    // 1. First beacon after boot / fresh fix
    if (!haveAnchor) { *why = "first"; return true; }

    // Hard rate floor: a send already takes ~20 s, so never beacon more often
    // than this. This also tames the "moved" trigger — at cycling/原付 speed 40 m
    // is reached every few seconds, but we cap it to one beacon per MIN_TX_GAP.
    if (elapsed < MIN_TX_GAP_MS) return false;

    // 1b. Last send failed: retry sooner than the normal cadence so a transient
    //     IPv6-only bearer or a dead cell doesn't leave us stuck red for a whole
    //     stopped interval (5-30 min). sendGpsData() re-attaches/re-IPv4s as needed.
    if (catmState == CM_ERR && elapsed >= CATM_FAIL_RETRY_MS) { *why = "retry"; return true; }

    // 2. Moved beyond threshold from the anchor (HDOP-gated to reject jitter)
    if (hdopOk) {
        double dist = TinyGPSPlus::distanceBetween(
            fixLat(), fixLon(), lastTxLat, lastTxLon);
        if (dist > MOVE_THRESH_M) { *why = "moved"; return true; }
    }

    // 3. Corner pegging — only when moving fast enough and heading is known
    //    (GPS course is noisy at walking speed; walking turns are caught by #2).
    if (spd > TURN_MIN_SPEED && lastTxCourse >= 0.0 && fixHasCourse()) {
        float diff = fabsf(fixCourseDeg() - (float)lastTxCourse);
        if (diff > 180.0f) diff = 360.0f - diff;
        if (diff > TURN_MIN_DEG + TURN_SLOPE / spd && elapsed > TURN_MIN_MS) {
            *why = "turn"; return true;
        }
    }

    // 4. Interval elapsed: decay when stopped, SmartBeaconing linear when moving
    uint32_t interval;
    if (*stopped) {
        interval = decayInterval;
    } else if (spd >= SB_HIGH_SPEED_KMH) {
        interval = SB_FAST_MS;
    } else {
        float frac = (SB_HIGH_SPEED_KMH - spd) / (SB_HIGH_SPEED_KMH - SB_LOW_SPEED_KMH);
        if (frac < 0.0f) frac = 0.0f;
        if (frac > 1.0f) frac = 1.0f;
        interval = SB_FAST_MS + (uint32_t)(frac * (SB_SLOW_MS - SB_FAST_MS));
    }
    if (elapsed >= interval) { *why = "interval"; return true; }

    return false;
}
