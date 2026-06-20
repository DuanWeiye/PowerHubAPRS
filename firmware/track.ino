// track.ino — 轨迹点格式化 + 存转(store-and-forward)队列 + 自适应 beacon 决策
// 配置A/B 共用。位置一律经 fixXxx() 访问器读取（实现见 config_a/config_b.ino），
// 故本文件对"位置从哪来"完全无感。
#include "defs.h"

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
