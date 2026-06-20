// pwrlog.ino — RTC 电量日志环形缓冲 + GNSS 信号(GSV/TXT)自解析 + USB 串口命令台
// 配置A/B 共用。
#include "defs.h"

// ═══════════════════════════════════════════════════════════════════════════
// GNSS 信号诊断解析（解析一整行 NMEA）
// 只看 GSV（星座/CN0/可见星数）与 TXT（天线状态）；不碰 TinyGPS++（它另收同样字节）。
// ═══════════════════════════════════════════════════════════════════════════
static void gnssDiagLine(const char* s) {
    if (s[0] != '$') return;
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
    static char buf[80];
    static uint8_t len = 0;
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (len == 0) continue;
            buf[len] = 0; len = 0;
            // 原始 AT 透传：以 "at"/"AT" 开头的整行（保留大小写）直接转发给模组，
            // 打印应答。用于现场对锁死/异常的 SIM7080G 逐条试探与恢复实验。
            if ((buf[0] == 'a' || buf[0] == 'A') && (buf[1] == 't' || buf[1] == 'T')) {
                Serial.printf("[AT>] %s\n", buf);
                Serial.printf("[AT<] %s\n", catmCmd(String(buf), 16000).c_str());
                continue;
            }
            for (char *p = buf; *p; p++) if (*p >= 'A' && *p <= 'Z') *p += 32;
            if      (!strcmp(buf, "log"))      pwrlogDump();
            else if (!strcmp(buf, "logclear")) { pwrlogClear(); Serial.println("[PWRLOG] cleared"); }
            else if (!strcmp(buf, "sendtest"))  { forceSendReq = true; Serial.println("[CMD] 强制发包"); }
            else if (!strcmp(buf, "gnsstest")) gnssSwitchTest();
            else if (!strcmp(buf, "atscan"))   atScan();
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
            else if (!strcmp(buf, "help"))     Serial.println("[CMD] log | logclear | sendtest | at<cmd> | gnsstest | atscan | help");
#endif
            else Serial.printf("[CMD] unknown: '%s' (try: help)\n", buf);
        } else if (len < sizeof(buf) - 1) {
            buf[len++] = c;
        } else {
            len = 0;   // overflow → discard the line
        }
    }
}
