// fuse.ino — 融合 v1：GPS→链路整行泵 + 按需改写 NMEA（PowerHub 零改动）
//
// 三个机制（全部只在"证据充分"时出手，其余场合逐字节透明）：
//   1) 静止锁存：IMU 判静止(≥3s)后，把 GGA/RMC 坐标锁到最近 3s 定位的中位数、速度置 0。
//      消灭停留时的 GPS 漂移云（多径 wander 正是在治的头号可见伪影）。
//   2) GNSS 逃生门：锁存期间原始定位持续(8s)远离锁点(>35m,HDOP<4) → 强制解锁。
//      IMU 误判（电梯/扶梯/平稳起步的电车）永远不可能把轨迹锁死——GNSS 说了算。
//   3) 短断档 DR 桥接：移动中丢定位 ≤15s/60m，用"最后好定位的速度 + 陀螺航向角速率"
//      续写坐标；GGA quality=6（NMEA 标准"估算/DR"）诚实标注，超限即停、如实丢定位。
// 改写只对校验和合法的行做；不认识/不完整/坏行一律原样转发（透传兜底）。
// 解析/重建核心在 nmea_rw.h（纯 C，主机单测过再上机）。
#include "defs.h"

// ── 开关（NVS 持久，console `fuse on/off` / 链路 $S3R,FUSE）──
static bool     fuseEn      = true;

// ── 定位历史环（锁点取分量中位数：单个多径野点当不了锁点）──
struct FixHist { double lat, lon; float hdop; uint32_t t; };
static FixHist  fh[FUSE_HIST_N];
static int      fhCount     = 0;
static int      fhHead      = 0;

// ── 静止锁存 ──
static bool     latched     = false;
static double   latchLat = 0, latchLon = 0;
static uint32_t tLatch      = 0;
static uint32_t tEscSince   = 0;     // 逃生门计时（0=未在计）

// ── 最后一个"好定位"（DR 起点）+ 最近 RMC 速度/航向 ──
static bool     lgValid     = false;
static double   lgLat = 0, lgLon = 0;
static float    lgSpdMps = 0, lgCrsDeg = 0, lgAltM = 0;
static uint32_t lgT         = 0;
static float    rawSpdMps   = 0;     // 最近有效 RMC 的速度/航向（喂 lastGood/GNSS 日志）
static float    rawCrsDeg   = -1;

// ── DR 桥接 ──
static bool     drOn        = false;
static double   drLat = 0, drLon = 0;
static float    drSpdMps = 0, drCrsDeg = 0, drDistM = 0;
static uint32_t tDrStart = 0, tDrLast = 0;

// ── 行泵/诊断 ──
static char     flBuf[128];
static uint8_t  flLen       = 0;
static bool     flOverflow  = false;
static uint32_t tPfuse      = 0;
static uint32_t tFuseLogPt  = 0;     // ILOG_FUSE 1Hz 节流

static void fuseInit() {
    // 默认 OFF：v1 已被 0808 外场证伪（误锁存劣化轨迹），P2 期间纯透传采数据，
    // v2(nav_core) 在 P3 替换本文件的三开关逻辑。NVS 全擦后也保持安全默认。
    fuseEn = prefs.getUChar("fuse", 0) != 0;
    Serial.printf("[FUS] fusion %s (NVS)\n", fuseEn ? "ON" : "OFF");
}

static bool fuseEnabledGet() { return fuseEn; }
static bool fuseIsLatched()  { return latched; }
static bool fuseIsDr()       { return drOn; }

static void fuseSetEnabled(bool on) {
    fuseEn = on;
    prefs.putUChar("fuse", on ? 1 : 0);
    if (!on) {                       // 关掉即刻放开一切干预
        if (latched) { latched = false; imulogEvent(EV_LATCH_OFF, 0); }
        if (drOn)    { drOn = false;    imulogEvent(EV_DR_ABORT, 0); }
    }
    Serial.printf("[FUS] fusion %s\n", on ? "ON" : "OFF");
}

// ── 小工具 ──────────────────────────────────────────────────────────────────

// 等矩形近似距离（米）：本用途全在 <100m 尺度，误差可忽略
static float fuseDistM(double lat1, double lon1, double lat2, double lon2) {
    float dN = (float)((lat2 - lat1) * 111320.0);
    float dE = (float)((lon2 - lon1) * 111320.0 * cos(lat1 * M_PI / 180.0));
    return sqrtf(dN * dN + dE * dE);
}

static double fuseMedian(double* v, int n) {     // 插入排序取中位（n≤15）
    for (int i = 1; i < n; i++) {
        double x = v[i]; int j = i - 1;
        while (j >= 0 && v[j] > x) { v[j + 1] = v[j]; j--; }
        v[j + 1] = x;
    }
    return v[n / 2];
}

static void fuseHistPush(double lat, double lon, float hdop, uint32_t t) {
    fh[fhHead] = { lat, lon, hdop, t };
    fhHead = (fhHead + 1) % FUSE_HIST_N;
    if (fhCount < FUSE_HIST_N) fhCount++;
}

// 用新鲜历史点的分量中位数定锁点；不足返回 false
static bool fuseComputeLatch(uint32_t now, double* olat, double* olon) {
    double la[FUSE_HIST_N], lo[FUSE_HIST_N];
    int n = 0;
    for (int i = 0; i < fhCount; i++) {
        const FixHist& e = fh[i];
        if (now - e.t <= FUSE_HIST_FRESH_MS) { la[n] = e.lat; lo[n] = e.lon; n++; }
    }
    if (n < FUSE_HIST_MIN) return false;
    *olat = fuseMedian(la, n);
    *olon = fuseMedian(lo, n);
    return true;
}

static void fuseLatchRelease(uint8_t evt) {
    if (!latched) return;
    latched = false;
    tEscSince = 0;
    imulogEvent(evt, 0);
    Serial.printf("[FUS] latch released (%s)\n",
                  evt == EV_LATCH_ESC ? "GNSS escape" : "motion");
}

// DR 推进一步（dt 秒）：航向按陀螺角速率转动，速度缓慢衰减
static void fuseDrPropagate(uint32_t now) {
    float dt = (now - tDrLast) / 1000.0f;
    if (dt <= 0) return;
    tDrLast = now;
    drCrsDeg += imuHeadingRateDps() * dt;
    while (drCrsDeg >= 360.0f) drCrsDeg -= 360.0f;
    while (drCrsDeg < 0.0f)    drCrsDeg += 360.0f;
    drSpdMps *= (1.0f - FUSE_DR_SPD_DECAY * dt);
    float d = drSpdMps * dt;
    drDistM += d;
    double cr = drCrsDeg * M_PI / 180.0;
    drLat += (double)(d * cos(cr)) / 111320.0;
    drLon += (double)(d * sin(cr)) / (111320.0 * cos(drLat * M_PI / 180.0));
}

// ═══════════════════════════════════════════════════════════════════════════
// GGA / RMC 处理（解析→记录→状态机→按需改写→转发）
// ═══════════════════════════════════════════════════════════════════════════

static void fuseForwardRaw(const char* line) {
    linkSerial.print(line);
    linkSerial.print("\r\n");
}

static void fuseHandleGGA(const char* line, uint32_t now) {
    char work[128], out[140];
    strlcpy(work, line, sizeof(work));
    char* f[NMEA_MAX_FIELDS];
    int n = nmeaSplit(work, f, NMEA_MAX_FIELDS);
    if (n < 10) { fuseForwardRaw(line); return; }

    int    qual  = atoi(f[6]);
    double lat = 0, lon = 0;
    bool   valid = qual >= 1 && nmeaLatLonParse(f[2], f[3], &lat)
                             && nmeaLatLonParse(f[4], f[5], &lon);
    float  hdop  = f[8][0] ? atof(f[8]) : 25.5f;
    uint8_t sats = (uint8_t)atoi(f[7]);
    float  alt   = f[9][0] ? atof(f[9]) : 0.0f;

    // 原始 GNSS 落盘（速度/航向取最近有效 RMC 的）
    imulogGnss(now, lat, lon, alt, rawSpdMps, rawCrsDeg < 0 ? 0 : rawCrsDeg,
               hdop, sats, valid);

    // ── 簿记（与开关无关）──
    if (valid) {
        if (hdop <= FUSE_HIST_HDOP_MAX) fuseHistPush(lat, lon, hdop, now);
        if (hdop <= FUSE_DR_HDOP_OK) {
            lgLat = lat; lgLon = lon; lgAltM = alt;
            lgSpdMps = rawSpdMps; lgCrsDeg = (rawCrsDeg < 0 ? 0 : rawCrsDeg);
            lgT = now; lgValid = true;
        }
        if (drOn) {                          // 定位回归 → 结束 DR，记桥接误差
            drOn = false;
            float err = fuseDistM(drLat, drLon, lat, lon);
            imulogEvent(EV_DR_OFF, err);
            Serial.printf("[FUS] DR end: bridged %.0fm, err vs fix %.0fm\n", drDistM, err);
        }
    }

    // ── 锁存状态机 ──
    if (latched && !imuIsStationary()) fuseLatchRelease(EV_LATCH_OFF);
    if (latched && valid && hdop <= FUSE_ESC_HDOP) {
        if (fuseDistM(latchLat, latchLon, lat, lon) > FUSE_ESC_DIST_M) {
            if (!tEscSince) tEscSince = now;
            else if (now - tEscSince >= FUSE_ESC_MS) fuseLatchRelease(EV_LATCH_ESC);
        } else tEscSince = 0;
    }
    if (!latched && fuseEn && imuOk() && imuIsStationary()
            && now - imuStationarySince() >= FUSE_LATCH_MIN_STAT_MS
            && fuseComputeLatch(now, &latchLat, &latchLon)) {
        latched = true;
        tLatch = now;
        tEscSince = 0;
        imulogEvent(EV_LATCH_ON, 0);
        Serial.printf("[FUS] latch ON @%.6f,%.6f\n", latchLat, latchLon);
    }

    // ── DR 进入（本句无定位时）──
    if (!valid && !drOn && fuseEn && imuOk() && imuBiasKnown() && !imuIsStationary()
            && lgValid && now - lgT <= FUSE_DR_START_MS && lgSpdMps >= FUSE_DR_MIN_SPD) {
        drOn = true;
        drLat = lgLat; drLon = lgLon;
        drSpdMps = lgSpdMps; drCrsDeg = lgCrsDeg; drDistM = 0;
        tDrStart = now; tDrLast = lgT;           // 从最后好定位时刻起推
        imulogEvent(EV_DR_ON, lgSpdMps);
        Serial.printf("[FUS] DR start @%.1f m/s crs %.0f\n", lgSpdMps, lgCrsDeg);
    }
    // DR 超限/静止 → 停（诚实把无定位交回去）
    if (drOn && (now - tDrStart > FUSE_DR_MAX_MS || drDistM > FUSE_DR_MAX_DIST_M
                 || imuIsStationary())) {
        drOn = false;
        imulogEvent(EV_DR_ABORT, drDistM);
        Serial.printf("[FUS] DR abort (%.0fm bridged)\n", drDistM);
    }

    // ── 输出 ──
    char dmLat[NMEA_NUM_BUF], dmLon[NMEA_NUM_BUF], hemiLat[2] = {0}, hemiLon[2] = {0};
    char hdopBuf[NMEA_NUM_BUF], altBuf[NMEA_NUM_BUF], qualBuf[2];
    if (latched) {
        // 锁存输出。定位还在：只替换坐标（quality/hdop/sats 保持原样，诚实）。
        // 定位丢了（高架下/楼影里静止等待）：IMU 明确知道人没动 → 用锁点续发，
        // quality=6 标注估算。无 GNSS 时逃生门失效，但静止检测一退出立即放行，风险有界。
        nmeaLatLonFmt(latchLat, false, dmLat, sizeof(dmLat), &hemiLat[0]);
        nmeaLatLonFmt(latchLon, true,  dmLon, sizeof(dmLon), &hemiLon[0]);
        f[2] = dmLat; f[3] = hemiLat; f[4] = dmLon; f[5] = hemiLon;
        if (!valid) {
            qualBuf[0] = '6'; qualBuf[1] = 0;
            f[6] = qualBuf;
            if (!f[8][0]) { strcpy(hdopBuf, "5.0"); f[8] = hdopBuf; }
            if (!f[9][0]) { snprintf(altBuf, sizeof(altBuf), "%.1f", lgAltM); f[9] = altBuf; }
        }
        if (nmeaRebuild(out, sizeof(out), f, n) > 0) linkSerial.print(out);
        else fuseForwardRaw(line);
        if (now - tFuseLogPt >= 1000) { tFuseLogPt = now; imulogFuse(now, latchLat, latchLon, 1); }
        return;
    }
    if (drOn && !valid) {
        fuseDrPropagate(now);
        nmeaLatLonFmt(drLat, false, dmLat, sizeof(dmLat), &hemiLat[0]);
        nmeaLatLonFmt(drLon, true,  dmLon, sizeof(dmLon), &hemiLon[0]);
        qualBuf[0] = '6'; qualBuf[1] = 0;        // NMEA 标准：6 = estimated/DR
        f[2] = dmLat; f[3] = hemiLat; f[4] = dmLon; f[5] = hemiLon; f[6] = qualBuf;
        if (!f[8][0]) { strcpy(hdopBuf, "5.0"); f[8] = hdopBuf; }
        if (!f[9][0]) { snprintf(altBuf, sizeof(altBuf), "%.1f", lgAltM); f[9] = altBuf; }
        if (nmeaRebuild(out, sizeof(out), f, n) > 0) linkSerial.print(out);
        else fuseForwardRaw(line);
        if (now - tFuseLogPt >= 1000) { tFuseLogPt = now; imulogFuse(now, drLat, drLon, 2); }
        return;
    }
    fuseForwardRaw(line);
}

static void fuseHandleRMC(const char* line, uint32_t now) {
    char work[128], out[140];
    strlcpy(work, line, sizeof(work));
    char* f[NMEA_MAX_FIELDS];
    int n = nmeaSplit(work, f, NMEA_MAX_FIELDS);
    if (n < 10) { fuseForwardRaw(line); return; }

    bool   valid = (f[2][0] == 'A');
    double lat = 0, lon = 0;
    if (valid) valid = nmeaLatLonParse(f[3], f[4], &lat)
                    && nmeaLatLonParse(f[5], f[6], &lon);
    if (valid) {
        if (f[7][0]) rawSpdMps = atof(f[7]) * 0.514444f;   // 节 → m/s
        if (f[8][0]) rawCrsDeg = atof(f[8]);
    }

    char dmLat[NMEA_NUM_BUF], dmLon[NMEA_NUM_BUF], hemiLat[2] = {0}, hemiLon[2] = {0};
    char spdBuf[NMEA_NUM_BUF], crsBuf[NMEA_NUM_BUF], stBuf[2];
    if (latched) {
        // 同 GGA：定位在丢失中也用锁点续发（status=A + mode=E 标注估算）
        nmeaLatLonFmt(latchLat, false, dmLat, sizeof(dmLat), &hemiLat[0]);
        nmeaLatLonFmt(latchLon, true,  dmLon, sizeof(dmLon), &hemiLon[0]);
        strcpy(spdBuf, "0.00");
        f[3] = dmLat; f[4] = hemiLat; f[5] = dmLon; f[6] = hemiLon; f[7] = spdBuf;
        if (!valid) {
            stBuf[0] = 'A'; stBuf[1] = 0;
            f[2] = stBuf;
            if (n >= 13) f[12] = (char*)"E";
        }
        if (nmeaRebuild(out, sizeof(out), f, n) > 0) { linkSerial.print(out); return; }
        fuseForwardRaw(line);
        return;
    }
    if (drOn && !valid) {
        fuseDrPropagate(now);
        nmeaLatLonFmt(drLat, false, dmLat, sizeof(dmLat), &hemiLat[0]);
        nmeaLatLonFmt(drLon, true,  dmLon, sizeof(dmLon), &hemiLon[0]);
        stBuf[0] = 'A'; stBuf[1] = 0;
        snprintf(spdBuf, sizeof(spdBuf), "%.2f", drSpdMps / 0.514444f);
        snprintf(crsBuf, sizeof(crsBuf), "%.1f", drCrsDeg);
        f[2] = stBuf; f[3] = dmLat; f[4] = hemiLat; f[5] = dmLon; f[6] = hemiLon;
        f[7] = spdBuf; f[8] = crsBuf;
        if (n >= 13) f[12] = (char*)"E";         // NMEA 4.x 模式指示：E = estimated
        if (nmeaRebuild(out, sizeof(out), f, n) > 0) { linkSerial.print(out); return; }
        fuseForwardRaw(line);
        return;
    }
    fuseForwardRaw(line);
}

// ═══════════════════════════════════════════════════════════════════════════
// 行泵 + 周期 tick
// ═══════════════════════════════════════════════════════════════════════════

static void fuseProcessLine(const char* line, uint32_t now) {
    // 只碰校验和合法的 GGA/RMC；其余（GSV/GSA/TXT/坏行/半行）原样转发
    if (line[0] == '$' && nmeaChecksumOk(line)) {
        if (nmeaIsType(line, "GGA")) { fuseHandleGGA(line, now); return; }
        if (nmeaIsType(line, "RMC")) { fuseHandleRMC(line, now); return; }
    }
    fuseForwardRaw(line);
}

// GPS→link 的整行泵（替代逐字节直转发；一行延迟 ~7ms@115200，PowerHub 无感）
static void fusePumpChar(char c) {
    if (flOverflow) {                            // 超长行：raw 直通直到行尾
        linkSerial.write((uint8_t)c);
        if (c == '\n' || c == '\r') flOverflow = false;
        return;
    }
    if (c == '\n' || c == '\r') {
        if (flLen) {
            flBuf[flLen] = 0;
            flLen = 0;
            fuseProcessLine(flBuf, millis());
        }
        return;                                  // 行尾由转发方自行补 \r\n
    }
    if (flLen < sizeof(flBuf) - 1) {
        flBuf[flLen++] = c;
    } else {                                     // NMEA 最长 82，正常到不了这里
        flBuf[flLen] = 0;
        linkSerial.print(flBuf);
        linkSerial.write((uint8_t)c);
        flLen = 0;
        flOverflow = true;
    }
}

// 1Hz：$PFUSE 诊断句 + 与句流无关的锁存释放兜底
static void fuseTick(uint32_t now) {
    if (latched && !imuIsStationary()) fuseLatchRelease(EV_LATCH_OFF);
    if (now - tPfuse < FUSE_PFUSE_MS) return;
    tPfuse = now;
    char body[96], out[110];
    char mode = latched ? 'L' : (drOn ? 'D' : 'P');
    snprintf(body, sizeof(body), "PFUSE,1,%d,%d,%c,%lu,%.0f,%.2f,%.2f,%.2f",
             fuseEn ? 1 : 0, imuIsStationary() ? 1 : 0, mode,
             latched ? (unsigned long)((now - tLatch) / 1000) : 0UL,
             drOn ? drDistM : 0.0f, imuHeadingRateDps(), imuAccStd(), imuGyroMag());
    snprintf(out, sizeof(out), "$%s*%02X\r\n", body, nmeaChecksum(body));
    linkSerial.print(out);
}

// console `fuse`
static void fusePrintStatus() {
    Serial.printf("[FUS] en=%d mode=%c latched=%d(%lus) dr=%d(%.0fm) hist=%d lg=%d(%.1fs)\n",
                  fuseEn, latched ? 'L' : (drOn ? 'D' : 'P'),
                  latched, latched ? (unsigned long)((millis() - tLatch) / 1000) : 0UL,
                  drOn, drDistM, fhCount, lgValid,
                  lgValid ? (millis() - lgT) / 1000.0f : 0.0f);
    if (latched)
        Serial.printf("[FUS] latch @%.6f,%.6f\n", latchLat, latchLon);
}
