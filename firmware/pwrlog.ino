// pwrlog.ino — RTC 电量日志环形缓冲 + GNSS 信号(GSV/TXT)自解析 + USB 串口命令台
// 配置A/B 共用。
#include "defs.h"

// 全星座最强 CN0（遥测 cn0 字段）
static uint8_t gnssMaxCn0() {
    uint8_t m = 0;
    for (int k = 0; k < 6; k++) if (gnssCN0[k] > m) m = gnssCN0[k];
    return m;
}

// ═══════════════════════════════════════════════════════════════════════════
// GNSS 信号诊断解析（解析一整行 NMEA）
// 只看 GSV（星座/CN0/可见星数）与 TXT（天线状态）；不碰 TinyGPS++（它另收同样字节）。
// ═══════════════════════════════════════════════════════════════════════════
// NMEA 校验和验证。链上 S3R 写 Flash 瞬间偶尔掉字节（坏行原样转发），字段错位会把
// 方位角(0-359)当 CN0、垃圾值当可见星数记进电量日志（实测出现过 cn0=213/sats=111），
// 坏行必须在解析前拦掉。TinyGPS++ 自带校验不受影响，这里只管诊断解析这条路。
static bool nmeaCsOk(const char* s) {
    uint8_t cs = 0;
    const char* p = s + 1;
    for (; *p && *p != '*'; p++) cs ^= (uint8_t)*p;
    if (*p != '*' || !isxdigit((unsigned char)p[1]) || !isxdigit((unsigned char)p[2]))
        return false;
    return (uint8_t)strtol(p + 1, nullptr, 16) == cs;
}

static void gnssDiagLine(const char* s) {
    if (s[0] != '$') return;
    if (!nmeaCsOk(s)) return;                     // 坏行直接丢，不污染诊断计数
    const char* ty = s + 3;                       // 句型在 talker(2) 之后
    if (ty[0]=='T' && ty[1]=='X' && ty[2]=='T') { // 天线状态 $xxTXT,...,ANTENNA OK/OPEN/SHORT
        if      (strstr(s, "ANTENNA OK"))    gnssAnt = 1;
        else if (strstr(s, "ANTENNA OPEN"))  gnssAnt = 2;
        else if (strstr(s, "ANTENNA SHORT")) gnssAnt = 3;
        return;
    }
    if (!(ty[0]=='G' && ty[1]=='S' && ty[2]=='V')) return;
    const char* tk = s + 1;                       // talker 两字符
    int slot;
    if      (tk[0]=='G' && tk[1]=='P') slot = 0;  // GPS（QZSS 旧固件也可能混在此，按 PRN 另判）
    else if (tk[0]=='G' && tk[1]=='L') slot = 1;  // GLONASS
    else if ((tk[0]=='B'&&tk[1]=='D') || (tk[0]=='G'&&tk[1]=='B')) slot = 2;  // BDS
    else if (tk[0]=='G' && tk[1]=='A') slot = 3;  // Galileo
    else if (tk[0]=='G' && tk[1]=='Q') slot = 4;  // QZSS
    else slot = 5;                                 // SBAS/其它

    // 收集逗号字段起点。GSV: f1=总条数 f2=本条序号 f3=可见星数，之后每 4 个 {prn,elev,az,snr}
    const char* f[24];
    int nf = 0;
    f[nf++] = s;
    for (const char* p = s; *p && *p!='*' && *p!='\r' && *p!='\n' && nf < 24; p++)
        if (*p == ',') f[nf++] = p + 1;
    if (nf < 4) return;
    int totalMsgs = atoi(f[1]);
    int msgNum    = atoi(f[2]);
    int numSV     = atoi(f[3]);
    if (msgNum <= 0) return;
    if (msgNum == 1) gnssAccCN0[slot] = 0;        // 本系统新周期开始，清累加器
    gnssInView[slot] = (uint8_t)numSV;            // 一个周期内各条相同
    for (int b = 4; b + 3 < nf; b += 4) {         // 逐颗卫星块
        int prn = atoi(f[b]);
        int snr = atoi(f[b + 3]);                 // 空字段 atoi=0
        if (snr > gnssAccCN0[slot]) gnssAccCN0[slot] = (uint8_t)snr;
        if (slot == 0 && prn >= 193 && prn <= 202 && snr > 0) {  // QZSS 混在 GPGSV 的兜底
            if (gnssInView[4] == 0) gnssInView[4] = 1;
            if (snr > gnssCN0[4]) gnssCN0[4] = (uint8_t)snr;
        }
    }
    if (msgNum >= totalMsgs) gnssCN0[slot] = gnssAccCN0[slot];   // 周期结束，提交本系统最强 CN0
}

// ═══════════════════════════════════════════════════════════════════════════
// Power log (RTC ring buffer) + USB serial command console
// ═══════════════════════════════════════════════════════════════════════════

// Validate RTC contents; re-init on cold boot (NOINIT garbage) or corruption.
static void pwrlogInit() {
    if (pwrlogMagic != PWRLOG_MAGIC || pwrlogHead >= PWRLOG_CAP
            || pwrlogCount > PWRLOG_CAP) {
        pwrlogMagic = PWRLOG_MAGIC;
        pwrlogHead  = 0;
        pwrlogCount = 0;
    }
}

static void pwrlogClear() {
    pwrlogMagic = PWRLOG_MAGIC;
    pwrlogHead  = 0;
    pwrlogCount = 0;
}

// Append one sample. Ring buffer: overwrites the oldest entry once full, so it
// can never overflow RTC memory.
static void pwrlogAppend() {
    PwrLogEntry e;
    time_t now = time(nullptr);
    e.ts  = (now > 1735689600L) ? (uint32_t)now : 0;   // >2025-01-01 → RTC synced
    e.mv  = phVolt(VM_BAT);
    e.ma  = phCurr(VM_BAT);
    e.pct = (uint8_t)batPct;
    e.flags = (hasExtPower ? 0x01 : 0)
            | (phRd8(REG_CHG) ? 0x02 : 0)
            | (gpsState == GS_FIX_GOOD ? 0x04 : 0)
            | ((uint8_t)catmState << 4);
    // GNSS 信号快照：星座位掩码 + 最强 CN0 + 可见星总数 + 天线
    uint8_t mask = 0, sats = 0, cn0 = 0;
    for (int k = 0; k < 6; k++) {
        if (gnssInView[k]) { mask |= (1 << k); sats += gnssInView[k]; }
        if (gnssCN0[k] > cn0) cn0 = gnssCN0[k];
    }
    e.cn0  = cn0;
    e.gnss = mask | (uint8_t)((gnssAnt & 0x03) << 6);
    e.sats = sats;
    pwrlogBuf[pwrlogHead] = e;
    pwrlogHead = (pwrlogHead + 1) % PWRLOG_CAP;
    if (pwrlogCount < PWRLOG_CAP) pwrlogCount++;
}

// Dump the whole log as CSV (oldest → newest) plus a battery-only current summary.
static void pwrlogDump() {
    Serial.printf("[PWRLOG] %u entries (cap %u, every %lus). "
                  "ma: -=charging(into batt), +=discharge(draw). on battery(ext=0) ma=draw\n",
                  pwrlogCount, PWRLOG_CAP, (unsigned long)(PWRLOG_MS / 1000));
    Serial.println("idx,ts,mv,ma,pct,ext,chg,fix,catm,cn0,gnss,sats");
    uint16_t start = (pwrlogCount == PWRLOG_CAP) ? pwrlogHead : 0;
    long sumDisc = 0; int nDisc = 0; int16_t minMa = 32767, maxMa = -32768;
    for (uint16_t i = 0; i < pwrlogCount; i++) {
        PwrLogEntry &e = pwrlogBuf[(start + i) % PWRLOG_CAP];
        uint8_t ext = e.flags & 1, chg = (e.flags >> 1) & 1, fix = (e.flags >> 2) & 1;
        uint8_t catm = (e.flags >> 4) & 0x0F;
        Serial.printf("%u,%lu,%u,%d,%u,%u,%u,%u,%u,%u,%u,%u\n",
                      i, (unsigned long)e.ts, e.mv, e.ma, e.pct, ext, chg, fix, catm,
                      e.cn0, e.gnss, e.sats);
        if (!ext) { sumDisc += e.ma; nDisc++;
                    if (e.ma < minMa) minMa = e.ma; if (e.ma > maxMa) maxMa = e.ma; }
        if ((i & 0x1F) == 0x1F) Serial.flush();   // help a slow CDC reader keep up
    }
    if (nDisc > 0)
        Serial.printf("[PWRLOG] battery-only: %d samples, avg ma=%ld (min=%d max=%d)\n",
                      nDisc, sumDisc / nDisc, minMa, maxMa);
    else
        Serial.println("[PWRLOG] no battery-only samples yet (always on external power)");
    Serial.println("[PWRLOG] end");
    Serial.flush();
}

// Non-blocking single-line command reader on the USB serial console:
//   log  = dump power log | logclear = erase | gnsstest = 分时切换测速 | help
static void checkSerialCommands() {
    static char buf[160];                  // gpsin 一整句 NMEA（≤82）+ 前缀
    static uint8_t len = 0;
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (len == 0) continue;
            buf[len] = 0; len = 0;
            // 原始 AT 透传：以 "at"/"AT" 开头的整行（保留大小写）直接转发给模组，
            // 打印应答。用于现场对锁死/异常的 SIM7080G 逐条试探与恢复实验。
            // 例外："atscan" 是本机命令，不能被当成 AT 行吞掉。
            if ((buf[0] == 'a' || buf[0] == 'A') && (buf[1] == 't' || buf[1] == 'T')
                    && strcasecmp(buf, "atscan") != 0) {
                Serial.printf("[AT>] %s\n", buf);
                Serial.printf("[AT<] %s\n", catmCmd(String(buf), 16000).c_str());
                continue;
            }
            // 链路注入（保留大小写）：把一行原样发到 PORT.C 的 GPS 链路。用途：①给模块发
            // PCAS 配置；②台面假点注入——S3R 收到 "$S3R,NMEA,<句>" 会把该句当 GPS 输出喂进
            // 估计器，改写结果原路回到这里被正常解析，不出门即可端到端验证融合链。
            if (!strncmp(buf, "gpsin ", 6)) {
                gpsSerial.print(buf + 6); gpsSerial.print("\r\n");
                continue;
            }
            for (char *p = buf; *p; p++) if (*p >= 'A' && *p <= 'Z') *p += 32;
            if      (!strcmp(buf, "log"))      pwrlogDump();
            else if (!strcmp(buf, "logclear")) { pwrlogClear(); Serial.println("[PWRLOG] cleared"); }
            else if (!strcmp(buf, "sendtest"))  { forceSendReq = true; Serial.println("[CMD] 强制发包"); }
            else if (!strcmp(buf, "gnsstest")) gnssSwitchTest();
            else if (!strcmp(buf, "atscan"))   atScan();
            // PORT.C 供电手控：S3R 接 USB 前先 pcoff，避免 USB 5V 与 Grove 5V 并联（无隔离）
            else if (!strcmp(buf, "pcoff")) { phPower(PC_UART, false); Serial.println("[PH] PORT.C 5V OFF"); }
            else if (!strcmp(buf, "pcon"))  { phPower(PC_UART, true);  Serial.println("[PH] PORT.C 5V ON"); }
#if !GNSS_TIMESHARE
            else if (!strcmp(buf, "s3rnav")) {
                uint32_t now = millis();
                Serial.printf("[S3R] nav %s: en=%d mode=%c sig=%.1fm still=%.2f spd=%.1fm/s hdopEma=%.1f "
                              "rej=%lu carried=%d accStd=%.2f hr=%.1f lines=%lu age=%lums lastGGAq=%u\n",
                              s3rNavActive(now) ? "ONLINE(旁路自家KF)" : "offline(自家KF)",
                              s3rNav.en, s3rNav.mode, s3rNav.sigmaM, s3rNav.still, s3rNav.spdMps,
                              s3rNav.hdopEma, (unsigned long)s3rNav.rej, s3rNav.carried, s3rNav.accStd,
                              s3rNav.headRate, (unsigned long)s3rNav.lines,
                              s3rNav.tLast ? (unsigned long)(now - s3rNav.tLast) : 0UL, lastGgaQual);
                Serial.printf("[S3R] liveFix valid=%d est=%d lat=%.6f lon=%.6f spd=%.1f hdop=%.1f sats=%u age=%lums\n",
                              liveFix.valid, liveFix.est, liveFix.lat, liveFix.lon, liveFix.spdKmh,
                              liveFix.hdop, liveFix.sats, liveFix.valid ? (unsigned long)(now - liveFix.tMs) : 0UL);
            }
            // ── S3R 协处理器链路（配置A：PORT.C，见 config_a.ino s3r*）──
            else if (!strcmp(buf, "s3rbridge")) s3rBridgeMode();
            else if (!strcmp(buf, "s3rping")) {
                gpsSerial.print("$S3R,PING\r\n");
                Serial.println("[S3R] PING 已发（应答见 [S3R] 行；GPS 直连时无应答）");
            }
            else if (!strcmp(buf, "s3rinfo")) {
                gpsSerial.print("$S3R,INFO\r\n");
                Serial.println("[S3R] INFO 已发（应答见 [S3R] 行；GPS 直连时无应答）");
            }
            else if (!strcmp(buf, "s3rimu")) {
                gpsSerial.print("$S3R,IMU\r\n");
                Serial.println("[S3R] IMU 查询已发（应答见 [S3R] 行）");
            }
            else if (!strcmp(buf, "s3rfuse on") || !strcmp(buf, "s3rfuse off")) {
                bool on = (buf[8] == 'o' && buf[9] == 'n');
                gpsSerial.printf("$S3R,FUSE,%s\r\n", on ? "ON" : "OFF");
                Serial.printf("[S3R] FUSE %s 已发（应答见 [S3R] 行）\n", on ? "ON" : "OFF");
            }
#endif
#if GNSS_TIMESHARE
            // ── 配置B 段日志台面实测命令（室内无 GPS，灌假点量上传耗时）──
            else if (!strncmp(buf, "flfill", 6)) {
                int n = atoi(buf + 6); if (n <= 0) n = 1;
                flashLogFillTest(n);
                uint16_t seg; uint32_t pts; flashLogCounts(&seg, &pts);
                Serial.printf("[FL] 灌入 %d 假点 → 积压 %lu 点 / %u 封段\n",
                              n, (unsigned long)pts, seg);
            }
            else if (!strcmp(buf, "flflush")) {
                uint16_t seg; uint32_t pts; flashLogCounts(&seg, &pts);
                Serial.printf("[FL] flflush 开始：积压 %lu 点 / %u 封段\n", (unsigned long)pts, seg);
                uint32_t t0 = millis();
                flashFlushViaLte(true);
                Serial.printf("[FL] flflush 完成：本次上传整体耗时 %lu ms\n",
                              (unsigned long)(millis() - t0));
            }
            else if (!strcmp(buf, "flstat")) {
                uint16_t seg; uint32_t pts; flashLogCounts(&seg, &pts);
                Serial.printf("[FL] 积压 %lu 点 / %u 封段（调度 %s）\n",
                              (unsigned long)pts, seg, flSchedHold ? "已挂起" : "自动");
            }
            else if (!strcmp(buf, "flclear")) { flashLogClear(); Serial.println("[FL] 段日志已清空"); }
            else if (!strcmp(buf, "flhold"))  { flSchedHold = !flSchedHold;
                Serial.printf("[FL] 自动 flush 调度 → %s\n", flSchedHold ? "挂起(只手动)" : "恢复自动"); }
            else if (!strcmp(buf, "help")) Serial.println(
                "[CMD] log|logclear|sendtest|at<cmd>|gnsstest|atscan | flfill<n>|flflush|flstat|flclear|flhold | help");
#else
            else if (!strcmp(buf, "help"))     Serial.println(
                "[CMD] log|logclear|sendtest|at<cmd>|gnsstest|atscan|pcon|pcoff|gpsin <nmea> | s3rbridge|s3rping|s3rinfo|s3rimu|s3rnav|s3rfuse on/off | help");
#endif
            else Serial.printf("[CMD] unknown: '%s' (try: help)\n", buf);
        } else if (len < sizeof(buf) - 1) {
            buf[len++] = c;
        } else {
            len = 0;   // overflow → discard the line
        }
    }
}
