// imulog.ino — IMU/GNSS 双层环形日志（LittleFS，P2 重分区后 spiffs 分区 4.875MB）
//
// 两层环形（同一套段轮转机制，两个实例）：
//   全速率层 /il/ ：25Hz IMU + 5Hz GNSS + 融合事件/零偏/UTC 对齐——疑难段深挖，
//                   ILOG_SEG_MAX×96KB ≈ 最近 55 分钟。
//   摘要层  /ils/：1Hz GNSS(ILOG_SGNS) + 1Hz 设备自算特征块(ILOG_FEAT) + UTC 对齐
//                   ——全天回放调参的主粮，ILOG_SUM_SEG_MAX×96KB ≈ 19 小时。
// 记录 = [type u8][定长负载]（小端，布局见 defs.h ILOG_*）。
// 拉取：$S3R,IMUDUMP 分块 ACK 协议（两层一起拉，SEG 行带路径），host 端 imu_fetch.py；
// 解码 imu_decode.py（按类型自识别，摘要层出 _sgnss/_feat CSV）。
// 省流：深度停留（still≥zuptOn 持续 >2min）暂停全速率 IMU、GNSS 降 1Hz；摘要层不省
// ——它本来就是低速率，全天连续才有价值。
#include "defs.h"

// ── 环形段管理（两层共用一套实现；struct IlRing 定义在 defs.h，自动原型坑）─────────
static IlRing   ilR  = { "/il",  "ilseq", ILOG_SEG_MAX,     File(), {0}, 0, {0}, 0 };
static IlRing   isR  = { "/ils", "isseq", ILOG_SUM_SEG_MAX, File(), {0}, 0, {0}, 0 };
static bool     ilOK        = false;
// 写盘失败记账（0822 外场：bin 里出现"上一段数据写了两遍、第一遍半条截断"，疑底层写失败后
// 重试；先把失败次数/字节数记下来并进 INFO/EVT，下一批数据定根因）
static uint32_t ilWriteFail      = 0;
static uint32_t ilWriteFailBytes = 0;
static bool     ilWriteFailPend  = false;   // 有失败未落 EVT（写成功后由 tick 补记）
static uint32_t tIlFlush    = 0;
static uint32_t tIlTime     = 0;       // 'T' 时间对齐记录节流
static uint32_t tLatchStart = 0;       // 停留信念满(still≥zuptOn)起始——全速率省流判据
static uint32_t tGnssLast   = 0;       // 深度静止时全速率 GNSS 1Hz 节流
static uint32_t tSumGnssLast= 0;       // 摘要层 GNSS 抽取节流

// 列出某层现有段（升序，序号单调递增所以文件名字典序 = 时间序）
static int ilListSegs(const char* dir, String* names, int maxN) {
    File d = LittleFS.open(dir);
    if (!d) return 0;
    int n = 0;
    File e;
    while ((e = d.openNextFile()) && n < maxN) {
        names[n++] = String(dir) + "/" + e.name();
        e.close();
    }
    d.close();
    for (int i = 1; i < n; i++) {              // 插入排序（≤28 个）
        String x = names[i]; int j = i - 1;
        while (j >= 0 && names[j] > x) { names[j + 1] = names[j]; j--; }
        names[j + 1] = x;
    }
    return n;
}

static void ilDeleteOldestIfNeeded(IlRing& r) {
    String names[ILOG_SUM_SEG_MAX + 2];
    int n = ilListSegs(r.dir, names, r.segMax + 2);
    while (n >= r.segMax) {                    // 保持 ≤ MAX-1，给新段留位
        LittleFS.remove(names[0]);
        for (int i = 1; i < n; i++) names[i - 1] = names[i];
        n--;
    }
}

static void ilOpenNext(IlRing& r) {
    char name[28];
    uint32_t seq = prefs.getULong(r.seqKey, 0) + 1;
    prefs.putULong(r.seqKey, seq);
    snprintf(name, sizeof(name), "%s/s%08lu.bin", r.dir, (unsigned long)seq);
    strlcpy(r.name, name, sizeof(r.name));
    r.f = LittleFS.open(name, "w");
    r.curSize = 0;
    if (!r.f) { ilOK = false; Serial.printf("[IL] open %s FAIL\n", name); }
}

static void ilFlushBuf(IlRing& r) {
    if (!ilOK || !r.f || !r.bufLen) return;
    size_t w = r.f.write(r.buf, r.bufLen);
    if (w != r.bufLen) {                       // 短写/失败：记账（数据丢弃，不重试——重试正是
        ilWriteFail++;                         // 重复写的嫌疑路径；先拿到计数再定策略）
        ilWriteFailBytes += r.bufLen - w;
        ilWriteFailPend = true;
    }
    r.f.flush();
    r.curSize += w;
    r.bufLen = 0;
    if (r.curSize >= ILOG_SEG_BYTES) {         // 段满 → 轮转
        r.f.close();
        ilDeleteOldestIfNeeded(r);
        ilOpenNext(r);
    }
}

static void ilWrite(IlRing& r, const void* rec, uint16_t len) {
    if (!ilOK) return;
    if (r.bufLen + len > sizeof(r.buf)) ilFlushBuf(r);
    if (!ilOK) return;                         // flush 可能因轮转失败关掉
    memcpy(r.buf + r.bufLen, rec, len);
    r.bufLen += len;
}

static void imulogBegin() {
    if (!LittleFS.begin(true)) {               // true = 首次/换分区自动格式化
        Serial.println("[IL] LittleFS mount FAIL — 日志停用");
        ilOK = false;
        return;
    }
    LittleFS.mkdir("/il");
    LittleFS.mkdir("/ils");
    ilDeleteOldestIfNeeded(ilR);
    ilDeleteOldestIfNeeded(isR);
    ilOK = true;
    ilOpenNext(ilR);
    ilOpenNext(isR);
    uint32_t total; uint16_t segs, sumSegs;
    imulogStats(&total, &segs, &sumSegs);
    Serial.printf("[IL] ready  全速率 %u 段 + 摘要 %u 段 = %luKB  free=%luKB\n",
                  segs, sumSegs, (unsigned long)(total / 1024),
                  (unsigned long)((LittleFS.totalBytes() - LittleFS.usedBytes()) / 1024));
}

// ── 各类型记录 ───────────────────────────────────────────────────────────────

static void imulogImuRaw(uint32_t tms, const int16_t a[3], const int16_t g[3]) {
    // 深度静止（锁存 >2min）暂停全速率 IMU 记录省环形空间
    if (tLatchStart && tms - tLatchStart > NAV_STILL_SAVE_MS) return;
    struct __attribute__((packed)) { uint8_t t; uint32_t ms; int16_t v[6]; } r;
    r.t = ILOG_IMU; r.ms = tms;
    r.v[0]=a[0]; r.v[1]=a[1]; r.v[2]=a[2]; r.v[3]=g[0]; r.v[4]=g[1]; r.v[5]=g[2];
    ilWrite(ilR, &r, sizeof(r));
}

// GNSS：全速率层每候选一条；摘要层按 ILOG_SUM_GNSS_MS 抽取（同负载，type=SGNS）
static void imulogGnss(uint32_t tms, double lat, double lon, float altM,
                       float spdMps, float crsDeg, float hdop, uint8_t sats, bool valid) {
    struct __attribute__((packed)) {
        uint8_t t; uint32_t ms; int32_t lat, lon; int16_t alt;
        uint16_t spd, crs; uint8_t hdop, sats, flags;
    } r;
    r.t = ILOG_GNSS; r.ms = tms;
    r.lat = (int32_t)(lat * 1e7); r.lon = (int32_t)(lon * 1e7);
    r.alt = (int16_t)altM;
    r.spd = (uint16_t)(spdMps * 100.0f);
    r.crs = (uint16_t)(crsDeg * 10.0f);
    float h10 = hdop * 10.0f;
    r.hdop = (uint8_t)(h10 > 255 ? 255 : h10);
    r.sats = sats;
    r.flags = valid ? 1 : 0;
    if (tms - tSumGnssLast >= ILOG_SUM_GNSS_MS) {
        tSumGnssLast = tms;
        r.t = ILOG_SGNS;
        ilWrite(isR, &r, sizeof(r));
        r.t = ILOG_GNSS;
    }
    if (tLatchStart && tms - tLatchStart > NAV_STILL_SAVE_MS) {   // 深度停留全速率降 1Hz
        if (tms - tGnssLast < 1000) return;
    }
    tGnssLast = tms;
    ilWrite(ilR, &r, sizeof(r));
}

// 秒级特征块（imu.ino 每 4 个 0.25s 块聚合一次调用）→ 摘要层
static void imulogFeat(uint32_t tms, float accStdMean, float accStdMax,
                       float gyroMean, float headRateDps, bool stationary) {
    struct __attribute__((packed)) {
        uint8_t t; uint32_t ms; uint16_t asMean, asMax, gyr; int16_t hr; uint8_t flags;
    } r;
    r.t = ILOG_FEAT; r.ms = tms;
    float m = accStdMean * 1000.0f, x = accStdMax * 1000.0f, g = gyroMean * 100.0f;
    r.asMean = (uint16_t)(m > 65535 ? 65535 : m);          // mm/s²
    r.asMax  = (uint16_t)(x > 65535 ? 65535 : x);
    r.gyr    = (uint16_t)(g > 65535 ? 65535 : g);          // 0.01dps
    float h = headRateDps * 100.0f;
    r.hr     = (int16_t)(h > 32767 ? 32767 : h < -32768 ? -32768 : h);
    r.flags  = stationary ? 1 : 0;
    ilWrite(isR, &r, sizeof(r));
}

static void imulogFuse(uint32_t tms, double lat, double lon, uint8_t mode) {
    struct __attribute__((packed)) { uint8_t t; uint32_t ms; int32_t lat, lon; uint8_t m; } r;
    r.t = ILOG_FUSE; r.ms = tms;
    r.lat = (int32_t)(lat * 1e7); r.lon = (int32_t)(lon * 1e7); r.m = mode;
    ilWrite(ilR, &r, sizeof(r));
}

static void imulogEvent(uint8_t code, float val) {
    struct __attribute__((packed)) { uint8_t t; uint32_t ms; uint8_t c; float v; } r;
    r.t = ILOG_EVT; r.ms = millis(); r.c = code; r.v = val;
    ilWrite(ilR, &r, sizeof(r));
}

static void imulogBias(const float b[3]) {
    struct __attribute__((packed)) { uint8_t t; uint32_t ms; float v[3]; } r;
    r.t = ILOG_BIAS; r.ms = millis(); r.v[0]=b[0]; r.v[1]=b[1]; r.v[2]=b[2];
    ilWrite(ilR, &r, sizeof(r));
}

static void imulogTimeMark(uint32_t epoch) {
    struct __attribute__((packed)) { uint8_t t; uint32_t ms; uint32_t ep; } r;
    r.t = ILOG_TIME; r.ms = millis(); r.ep = epoch;
    ilWrite(ilR, &r, sizeof(r));               // 两层都写：各自独立可对齐 UTC
    ilWrite(isR, &r, sizeof(r));
}

// ── tick：周期落盘 + 锁存观察 + UTC 对齐记录 ────────────────────────────────────

// y/m/d h:m:s(UTC) → epoch 秒（无时区/闰秒，够对齐用）
static uint32_t ilEpochFrom(int y, int mo, int d, int h, int mi, int s) {
    static const uint16_t cum[12] = {0,31,59,90,120,151,181,212,243,273,304,334};
    uint32_t days = (uint32_t)(y - 1970) * 365 + (y - 1969) / 4 + cum[mo - 1] + (d - 1);
    if (mo > 2 && (y % 4 == 0)) days++;        // 2100 前有效
    return ((days * 24UL + h) * 60 + mi) * 60 + s;
}

static void imulogTick(uint32_t now) {
    if (!ilOK) return;
    // 停留信念满（still≥zuptOn）起始 → 全速率省流判据（>NAV_STILL_SAVE_MS 后 IMU 停记/GNSS 降 1Hz）
    tLatchStart = fuseStillLatched() ? fuseStillSince() : 0;
    // 写盘失败补记 EVT（在失败之后的第一次成功 tick 里写，避免在失败路径里递归写）
    if (ilWriteFailPend && ilR.bufLen + 10 <= sizeof(ilR.buf)) {
        ilWriteFailPend = false;
        imulogEvent(EV_IL_WFAIL, (float)ilWriteFail);
    }
    // 周期落盘（掉电最多丢 2s）
    if (now - tIlFlush >= ILOG_FLUSH_MS) {
        tIlFlush = now;
        ilFlushBuf(ilR);
        ilFlushBuf(isR);
    }
    // UTC 对齐记录：每分钟一条 millis↔UTC（离线绝对时间轴）。
    // TinyGPS 的 date 在无定位时 isValid 也可能给出空字段解析的垃圾（实测 month=0
    // 曾致 cum[-1] 越界读、写出 2046 年）——必须逐项范围校验。
    if (now - tIlTime >= ILOG_TIME_MS && gps.date.isValid() && gps.time.isValid()
            && gps.date.age() < 2000) {
        int y = gps.date.year(), mo = gps.date.month(), d = gps.date.day();
        if (y >= 2025 && y < 2100 && mo >= 1 && mo <= 12 && d >= 1 && d <= 31) {
            tIlTime = now;
            imulogTimeMark(ilEpochFrom(y, mo, d, gps.time.hour(), gps.time.minute(),
                                       gps.time.second()));
        }
    }
}

static uint32_t ilRingBytes(IlRing& r, int* segsOut) {
    String names[ILOG_SUM_SEG_MAX + 2];
    int n = ilListSegs(r.dir, names, r.segMax + 2);
    uint32_t total = r.bufLen;
    for (int i = 0; i < n; i++) {
        File f = LittleFS.open(names[i], "r");
        if (f) { total += f.size(); f.close(); }
    }
    if (segsOut) *segsOut = n;
    return total;
}

static void imulogStats(uint32_t* totalBytes, uint16_t* segs, uint16_t* sumSegs) {
    int n1, n2;
    uint32_t total = ilRingBytes(ilR, &n1) + ilRingBytes(isR, &n2);
    if (totalBytes) *totalBytes = total;
    if (segs)       *segs    = (uint16_t)n1;
    if (sumSegs)    *sumSegs = (uint16_t)n2;
}

static uint32_t imulogWriteFails() { return ilWriteFail; }
static uint32_t imulogFreeKB() {
    if (!ilOK) return 0;
    return (uint32_t)((LittleFS.totalBytes() - LittleFS.usedBytes()) / 1024);
}

static void imulogClear() {
    if (!ilOK) return;
    ilR.f.close(); ilR.bufLen = 0;
    isR.f.close(); isR.bufLen = 0;
    String names[ILOG_SUM_SEG_MAX + 2];
    for (IlRing* r : { &ilR, &isR }) {
        int n = ilListSegs(r->dir, names, r->segMax + 2);
        for (int i = 0; i < n; i++) LittleFS.remove(names[i]);
    }
    ilOpenNext(ilR);
    ilOpenNext(isR);
    Serial.println("[IL] cleared (both rings)");
}

// ── $S3R,IMUDUMP：分块 ACK 传输（两层一起；阻塞，期间透传暂停）────────────────────
// 活动段（正在写的文件）在 dump 期间必须先关写句柄：LittleFS 不支持同一文件双开，
// 0822 拉回的 bin 里活动段尾部混进了几十秒前的陈旧块就是这么来的。dump 后以追加重开。
static void ilCloseForDump(IlRing& r)  { ilFlushBuf(r); if (r.f) r.f.close(); }
static void ilReopenAfterDump(IlRing& r) {
    if (!ilOK) return;
    r.f = LittleFS.open(r.name, "a");
    if (!r.f) { Serial.printf("[IL] reopen %s FAIL → new seg\n", r.name); ilOpenNext(r); }
}

static void imulogDumpInner(Stream* io);
static void imulogDump(Stream* io) {
    if (!ilOK) { io->println("$S3R,IMUDUMP,ERR,nolog"); return; }
    ilCloseForDump(ilR);
    ilCloseForDump(isR);
    imulogDumpInner(io);
    ilReopenAfterDump(ilR);
    ilReopenAfterDump(isR);
}

static void imulogDumpInner(Stream* io) {
    String names[2 * (ILOG_SUM_SEG_MAX + 2)];
    int n = ilListSegs(ilR.dir, names, ILOG_SEG_MAX + 2);
    n += ilListSegs(isR.dir, names + n, ILOG_SUM_SEG_MAX + 2);
    uint32_t total = 0;
    for (int i = 0; i < n; i++) {
        File f = LittleFS.open(names[i], "r");
        if (f) { total += f.size(); f.close(); }
    }
    io->printf("$S3R,IMUDUMP,BEGIN,%d,%lu\r\n", n, (unsigned long)total);

    static uint8_t chunk[1024];
    char ackBuf[48], expect[32];
    uint32_t seq = 0;
    for (int i = 0; i < n; i++) {
        File f = LittleFS.open(names[i], "r");
        if (!f) continue;
        io->printf("$S3R,IMUDUMP,SEG,%s,%lu\r\n", names[i].c_str(), (unsigned long)f.size());
        while (f.available()) {
            size_t len = f.read(chunk, sizeof(chunk));
            uint32_t crc = crc32sw(chunk, len);
            snprintf(expect, sizeof(expect), "$S3R,IMUDUMP,ACK,%lu", (unsigned long)seq);
            bool acked = false;
            for (int attempt = 0; attempt < 4 && !acked; attempt++) {
                io->printf("$S3R,IMUDUMP,DATA,%lu,%u,%08lx\r\n",
                           (unsigned long)seq, (unsigned)len, (unsigned long)crc);
                io->write(chunk, len);
                uint32_t deadline = millis() + 3000;
                while (otaReadLine(io, ackBuf, sizeof(ackBuf), deadline)) {
                    if (!strcmp(ackBuf, expect)) { acked = true; break; }
                }
            }
            if (!acked) {
                f.close();
                io->println("$S3R,IMUDUMP,ERR,noack");
                Serial.println("[IL] dump abort: no ACK");
                return;
            }
            seq++;
        }
        f.close();
    }
    io->println("$S3R,IMUDUMP,END");
    Serial.printf("[IL] dump done: %d segs %luKB\n", n, (unsigned long)(total / 1024));
}
