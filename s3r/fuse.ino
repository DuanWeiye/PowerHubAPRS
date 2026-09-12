// fuse.ino — 融合 v2：GPS→链路整行泵 + nav_core 估计器按需改写 GGA/RMC（PowerHub 可零改动）
//
// v1 的静止锁存/DR/逃生门三个开关已整体删除（0808 外场证伪，见 HANDOFF）。v2 只有一个
// 估计器（nav_core.h：2D 匀速 KF + 连续停留信念 + 诚实 coast），行为全部由它的协方差机制
// 连续产生；本文件只负责"喂它、把它的输出写回 NMEA"。
//
// 每拍语句序（ATGM336H 实测）：RMC → VTG → GGA。估计器由 GGA 驱动（它带 HDOP，和回放
// 调参时的输入语义一致），所以 RMC 先暂存，GGA 算完后按同一份输出先放改写后的 RMC、再放
// GGA——同拍两句坐标/速度严格一致。GGA 缺失超时则把暂存的 RMC 原样放行（透传兜底）。
//
// 输出规则（诚实优先）：
//   估计器有点：GGA/RMC 坐标换成估计；推算点 GGA quality=6 / RMC 模式 E。
//   估计器无点：原句有定位（被野点门拒）→ GGA quality=0 / RMC status=V（不让下游采信）；
//               原句本就无定位 → 原样转发。
//   fuse 关 / 坏行 / 非 GGA·RMC → 逐字节透明（透传语义兜底）。
#include "defs.h"
#include "nav_core.h"

// ── 开关（NVS 持久，console `fuse on/off` / 链路 $S3R,FUSE）──
static bool     fuseEn      = true;
static NavCore  nav;

// ── 本拍状态（GGA 驱动）──
static NavOut   epochOut;              // 本拍估计器输出（emit=false 表示无点）
static float    rawSpdMps   = 0;       // 本拍 RMC 原始速度/航向（喂估计器首拍过渡 + 原始日志）
static float    rawCrsDeg   = -1;
static float    lastAltM    = 0;       // 最近有效 GGA 海拔（推算点补字段）
static char     fuseMode    = 'P';     // P=透传(关) F=跟踪 C=推算 N=本拍无点
static uint32_t navRej      = 0;       // 候选被野点门拒绝计数（遥测）
static uint32_t tStillSince = 0;       // still ≥ zuptOn 的起始（imulog 省流判据；0=未满足）

// ── RMC 暂存（RMC 先于 GGA 到达）──
static char     heldRmc[128];
static bool     rmcHeld     = false;
static bool     heldRmcValid= false;   // 暂存的 RMC 自身 status=A
static uint32_t tRmcHeld    = 0;

// ── IMU 特征：最近 4 块(1s)滑动最大——与摘要层 FEAT.accStdMax / 回放 feat=device 同语义 ──
static float    blkStd[4]   = {0};
static uint8_t  blkIdx      = 0;

// ── 行泵/诊断 ──
static char     flBuf[128];
static uint8_t  flLen       = 0;
static bool     flOverflow  = false;
static uint32_t tPfuse      = 0;
static uint32_t tFuseLogPt  = 0;       // ILOG_FUSE 1Hz 节流
static uint32_t tInjectUntil= 0;       // 台面注入窗：此前收到过 $S3R,NMEA → 真 GPS 的 GGA/RMC 暂不进估计器

static void fuseNavRestart() {
    NavParams prm;
    navParamsDefault(&prm);            // P2 定版参数（0808/0816/0822 三份数据）
    navInit(&nav, &prm);
    epochOut.emit = false;
    fuseMode = fuseEn ? 'N' : 'P';
    tStillSince = 0;
}

static void fuseInit() {
    // 默认 ON：v2 已在三份外场数据上回放验证 ≥ 基线；NVS 里有值以其为准（s3rfuse on/off）
    fuseEn = prefs.getUChar("fuse", 1) != 0;
    fuseNavRestart();
    Serial.printf("[FUS] nav v2 %s (NVS)\n", fuseEn ? "ON" : "OFF");
}

static bool  fuseEnabledGet()  { return fuseEn; }
static char  fuseModeGet()     { return fuseMode; }
static float fuseStillGet()    { return nav.have ? nav.still : 0.0f; }
static float fuseSigmaGet()    { return navSigmaM(&nav); }
static bool  fuseStillLatched(){ return tStillSince != 0; }
static uint32_t fuseStillSince(){ return tStillSince; }

static void fuseSetEnabled(bool on) {
    fuseEn = on;
    prefs.putUChar("fuse", on ? 1 : 0);
    fuseNavRestart();                  // 开关切换都从干净状态起算（关=放开一切干预）
    Serial.printf("[FUS] nav v2 %s\n", on ? "ON" : "OFF");
}

// imu.ino 每个 0.25s 统计块结束时调用：滑动 1s 最大 accStd + 当前航向角速率 → 估计器
static void fuseImuBlock(float accStd, float headRateDps, uint32_t now) {
    blkStd[blkIdx] = accStd;
    blkIdx = (blkIdx + 1) & 3;
    float mx = blkStd[0];
    for (int i = 1; i < 4; i++) if (blkStd[i] > mx) mx = blkStd[i];
    if (fuseEn) navImu(&nav, now, mx, headRateDps);
}

// ═══════════════════════════════════════════════════════════════════════════
// 转发/暂存
// ═══════════════════════════════════════════════════════════════════════════

static void fuseForwardRaw(const char* line) {
    linkSerial.print(line);
    linkSerial.print("\r\n");
}

// 暂存的 RMC 按本拍输出放行：有点→改写；原句有效但本拍无点→作废；其余原样
static void fuseReleaseHeldRmc(bool useEpoch) {
    if (!rmcHeld) return;
    rmcHeld = false;
    if (useEpoch) {
        char work[128], out[140];
        strlcpy(work, heldRmc, sizeof(work));
        char* f[NMEA_MAX_FIELDS];
        int n = nmeaSplit(work, f, NMEA_MAX_FIELDS);
        NmeaEdit e;
        bool ok = false;
        if (epochOut.emit)
            ok = nmeaRmcApply(f, n, epochOut.lat, epochOut.lon, epochOut.spdMps,
                              epochOut.crsDeg, epochOut.crsValid, epochOut.est, &e);
        else if (heldRmcValid)
            ok = nmeaRmcInvalidate(f, n, &e);
        else { fuseForwardRaw(heldRmc); return; }
        if (ok && nmeaRebuild(out, sizeof(out), f, n) > 0) { linkSerial.print(out); return; }
    }
    fuseForwardRaw(heldRmc);
}

// ═══════════════════════════════════════════════════════════════════════════
// RMC：记原始速度/航向 → 暂存到 GGA 算完
// ═══════════════════════════════════════════════════════════════════════════
static void fuseHandleRMC(const char* line, uint32_t now) {
    char work[128];
    strlcpy(work, line, sizeof(work));
    char* f[NMEA_MAX_FIELDS];
    int n = nmeaSplit(work, f, NMEA_MAX_FIELDS);
    if (n < 10) { fuseForwardRaw(line); return; }

    bool   valid = (f[2][0] == 'A');
    double lat = 0, lon = 0;
    if (valid) valid = nmeaLatLonParse(f[3], f[4], &lat) && nmeaLatLonParse(f[5], f[6], &lon);
    if (valid) {
        rawSpdMps = f[7][0] ? atof(f[7]) * 0.514444f : 0.0f;   // 节 → m/s
        rawCrsDeg = f[8][0] ? atof(f[8]) : -1.0f;
    } else { rawSpdMps = 0; rawCrsDeg = -1; }

    if (!fuseEn) { fuseForwardRaw(line); return; }
    fuseReleaseHeldRmc(false);         // 上一拍 GGA 没来 → 先把旧的原样放掉
    strlcpy(heldRmc, line, sizeof(heldRmc));
    heldRmcValid = valid;
    rmcHeld = true;
    tRmcHeld = now;
}

// ═══════════════════════════════════════════════════════════════════════════
// GGA：原始日志 → 估计器一步 → 放 RMC → 放 GGA
// ═══════════════════════════════════════════════════════════════════════════
static void fuseHandleGGA(const char* line, uint32_t now) {
    char work[128], out[140];
    strlcpy(work, line, sizeof(work));
    char* f[NMEA_MAX_FIELDS];
    int n = nmeaSplit(work, f, NMEA_MAX_FIELDS);
    if (n < 10) { fuseReleaseHeldRmc(false); fuseForwardRaw(line); return; }

    int    qual  = atoi(f[6]);
    double lat = 0, lon = 0;
    bool   valid = qual >= 1 && nmeaLatLonParse(f[2], f[3], &lat)
                             && nmeaLatLonParse(f[4], f[5], &lon)
                             && !(lat == 0.0 && lon == 0.0);   // 空字段解析成 0,0 ≠ 定位

    float  hdop  = f[8][0] ? atof(f[8]) : 25.5f;
    uint8_t sats = (uint8_t)atoi(f[7]);
    float  alt   = f[9][0] ? atof(f[9]) : 0.0f;

    // 原始 GNSS 落盘（与开关无关；速度/航向来自本拍 RMC）
    imulogGnss(now, lat, lon, alt, rawSpdMps, rawCrsDeg < 0 ? 0 : rawCrsDeg, hdop, sats, valid);
    if (valid) lastAltM = alt;

    if (!fuseEn) { fuseReleaseHeldRmc(false); fuseForwardRaw(line); return; }

    // ── 估计器一步 ──
    NavOut o;
    bool got;
    if (valid) {
        got = navGnss(&nav, now, lat, lon, hdop, rawSpdMps, rawCrsDeg, &o);
        if (!got) { navRej++; got = navCoast(&nav, now, &o); }   // 野点拍 → 推算
    } else {
        got = navCoast(&nav, now, &o);
    }
    if (got && o.emit) epochOut = o; else epochOut.emit = false;
    fuseMode = epochOut.emit ? (epochOut.est ? 'C' : 'F') : 'N';
    if (nav.have && nav.still >= nav.prm.zuptOn) { if (!tStillSince) tStillSince = now; }
    else tStillSince = 0;

    // ── 输出：先 RMC（同拍），再 GGA ──
    fuseReleaseHeldRmc(true);
    NmeaEdit e;
    bool ok = false;
    if (epochOut.emit)
        ok = nmeaGgaApply(f, n, epochOut.lat, epochOut.lon, epochOut.est,
                          NAV_EST_HDOP_FILL, lastAltM, &e);
    else if (valid)
        ok = nmeaGgaInvalidate(f, n, &e);
    else { fuseForwardRaw(line); return; }
    if (ok && nmeaRebuild(out, sizeof(out), f, n) > 0) linkSerial.print(out);
    else fuseForwardRaw(line);

    if (epochOut.emit && now - tFuseLogPt >= 1000) {
        tFuseLogPt = now;
        imulogFuse(now, epochOut.lat, epochOut.lon, epochOut.est ? 2 : 1);
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// 行泵 + 周期 tick
// ═══════════════════════════════════════════════════════════════════════════

static void fuseProcessLineEx(const char* line, uint32_t now, bool injected) {
    // 只碰校验和合法的 GGA/RMC；其余（GSV/GSA/VTG/TXT/坏行/半行）原样转发
    if (line[0] == '$' && nmeaChecksumOk(line)) {
        bool rmc = nmeaIsType(line, "RMC"), gga = nmeaIsType(line, "GGA");
        // 注入窗内真 GPS 的定位句整条丢弃（屋里无定位的真句每拍都会触发 coast，搅混台面测试）；
        // 窗口 3s 无注入自动关闭，外场永远不会进入。
        if ((rmc || gga) && !injected && tInjectUntil && (int32_t)(tInjectUntil - now) > 0) return;
        if (rmc) { fuseHandleRMC(line, now); return; }
        if (gga) { fuseHandleGGA(line, now); return; }
    }
    fuseForwardRaw(line);
}
static void fuseProcessLine(const char* line, uint32_t now) { fuseProcessLineEx(line, now, false); }

// 台面注入（ota.ino $S3R,NMEA,…）：整句当 GPS 输出喂估计器，并开 3s 注入窗
static void fuseInjectLine(const char* line) {
    uint32_t now = millis();
    tInjectUntil = now + 3000;
    fuseProcessLineEx(line, now, true);
}

// GPS→link 的整行泵（一行延迟 ~7ms@115200；RMC 另加暂存到 GGA ≈ 2 句 ~15ms，PowerHub 无感）
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

// 1Hz：$PFUSE 心跳/诊断句 + 暂存 RMC 超时兜底
static void fuseTick(uint32_t now) {
    if (rmcHeld && now - tRmcHeld > NAV_RMC_HOLD_MS) fuseReleaseHeldRmc(false);   // GGA 缺失
    if (now - tPfuse < FUSE_PFUSE_MS) return;
    tPfuse = now;
    // $PFUSE,2,<en>,<mode>,<σm>,<still>,<spd m/s>,<hdopEma>,<rej>,<carried>,<accStd1s>,<headRate>
    // PowerHub 据 en+新鲜度决定是否旁路自家野点门/KF（单估计器原则），其余字段为遥测。
    float mx = blkStd[0];
    for (int i = 1; i < 4; i++) if (blkStd[i] > mx) mx = blkStd[i];
    float spd = nav.have ? sqrtf((float)(nav.kx.v * nav.kx.v + nav.ky.v * nav.ky.v)) : 0.0f;
    char body[110], out[124];
    snprintf(body, sizeof(body), "PFUSE,2,%d,%c,%.1f,%.2f,%.1f,%.1f,%lu,%d,%.2f,%.1f",
             fuseEn ? 1 : 0, fuseMode, navSigmaM(&nav), fuseStillGet(), spd,
             nav.hdopEma, (unsigned long)navRej, navCarried(&nav, now) ? 1 : 0,
             mx, nav.headRateDps);
    snprintf(out, sizeof(out), "$%s*%02X\r\n", body, nmeaChecksum(body));
    linkSerial.print(out);
}

// console `fuse`
static void fusePrintStatus() {
    Serial.printf("[FUS] en=%d mode=%c have=%d sigma=%.1fm still=%.2f(%lus) rej=%lu "
                  "hdopEma=%.1f coast=%d blocked=%d carried=%d rmcHeld=%d\n",
                  fuseEn, fuseMode, nav.have, navSigmaM(&nav), fuseStillGet(),
                  tStillSince ? (unsigned long)((millis() - tStillSince) / 1000) : 0UL,
                  (unsigned long)navRej, nav.hdopEma, nav.inCoast, nav.coastBlocked,
                  navCarried(&nav, millis()), rmcHeld);
    if (epochOut.emit)
        Serial.printf("[FUS] out @%.6f,%.6f spd=%.1f crs=%.0f(%d) est=%d\n",
                      epochOut.lat, epochOut.lon, epochOut.spdMps, epochOut.crsDeg,
                      epochOut.crsValid, epochOut.est);
}
