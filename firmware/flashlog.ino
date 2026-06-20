// flashlog.ino — 配置B 的 LittleFS 存转日志（仅 GNSS_TIMESHARE 编译；配置A 不含）。
//
// 设计（见与用户的讨论）：
//   · GPS 点逐点追加到"分段文件"，满 FL_SEG_POINTS 点即封段、滚到下一段。
//   · 每点写完即 flush 落盘 → 普通重启/掉电都不丢（LittleFS 非易失、抗掉电）。
//   · 到"廉价时刻"（静止/无定位）才切 LTE，把封好的段最旧先发、每段服务器 200 即删。
//   · 崩溃/中途断网 → 整段保留、下次重发；服务端按时间戳去重保证幂等。
//   · 磁盘满 → 删最旧段（记录仪保最新）。
//   · RAM 里只缓存计数（O(1)），开机扫一次盘重建，之后全增量维护，绝不每轮扫盘。
//
// 段文件：/t/<8位序号>.s ，每条记录 = 一个 packed TrackPoint(20B)。
//
// 配置A/B 共用：B 每个 beacon 都记 + 静止/无定位才 flush；A 仅在实时发包失败(无信号)
// 时记，网络一恢复(下次发包成功)立刻全部上传——两套都靠这份持久化存储，彻底防断电。
#include "defs.h"
#include <LittleFS.h>
#include <ctype.h>

#define FL_REC ((int)sizeof(TrackPoint))   // 20 字节/点
#define FL_DIR "/t"

static bool     flReady       = false;
static uint32_t flCurSeq      = 0;   // 当前(正在追加)段序号
static uint16_t flCurPoints   = 0;   // 当前段已有点数
static uint32_t flOldestSeq   = 0;   // 最旧"已封段"序号（上传/丢弃从这里起）
static uint16_t flSealedCount = 0;   // 已封段数（满段 + 上传前临封的半段）
static uint32_t flTotalPoints = 0;   // 全部点数（含当前半段）

static void flSegPath(char* buf, size_t cap, uint32_t seq) {
    snprintf(buf, cap, "%s/%08lu.s", FL_DIR, (unsigned long)seq);
}

// 从文件名解析段序号（兼容 name() 带/不带目录前缀两种约定）。
static bool flParseSeq(const char* name, uint32_t* out) {
    const char* base = strrchr(name, '/');
    base = base ? base + 1 : name;
    if (!isdigit((unsigned char)base[0])) return false;
    *out = strtoul(base, nullptr, 10);
    return true;
}

// 开机扫一次盘，重建 RAM 计数（之后全增量维护，不再扫盘）。
static void flashLogBegin() {
    flReady = LittleFS.begin(true);          // true = 首次/损坏则格式化为 LittleFS
    if (!flReady) { Serial.println("[FL] LittleFS 挂载失败"); return; }
    if (!LittleFS.exists(FL_DIR)) LittleFS.mkdir(FL_DIR);

    uint32_t minSeq = 0, maxSeq = 0;
    uint16_t maxSeqPts = 0, fullCount = 0;
    uint32_t totalPts = 0;
    bool any = false;

    File dir = LittleFS.open(FL_DIR);
    if (dir) {
        for (File f = dir.openNextFile(); f; f = dir.openNextFile()) {
            uint32_t seq;
            if (flParseSeq(f.name(), &seq)) {
                uint16_t pts = (uint16_t)(f.size() / FL_REC);
                totalPts += pts;
                if (pts >= FL_SEG_POINTS) fullCount++;
                if (!any || seq < minSeq) minSeq = seq;
                if (!any || seq > maxSeq) { maxSeq = seq; maxSeqPts = pts; }
                any = true;
            }
            f.close();
        }
        dir.close();
    }

    if (!any) {
        flCurSeq = flOldestSeq = 0; flCurPoints = 0; flSealedCount = 0; flTotalPoints = 0;
    } else if (maxSeqPts < FL_SEG_POINTS) {
        flCurSeq = maxSeq; flCurPoints = maxSeqPts;     // 最高段没满 → 它就是当前段
        flOldestSeq = minSeq; flSealedCount = fullCount; flTotalPoints = totalPts;
    } else {
        flCurSeq = maxSeq + 1; flCurPoints = 0;         // 最高段也满 → 开新空当前段
        flOldestSeq = minSeq; flSealedCount = fullCount; flTotalPoints = totalPts;
    }
    Serial.printf("[FL] LittleFS OK  积压=%lu点/%u封段  current=#%lu(%u)  flash %lu/%lu KB\n",
                  (unsigned long)flTotalPoints, flSealedCount,
                  (unsigned long)flCurSeq, flCurPoints,
                  (unsigned long)(LittleFS.usedBytes() / 1024),
                  (unsigned long)(LittleFS.totalBytes() / 1024));
}

static void flashLogCounts(uint16_t* sealedSegs, uint32_t* totalPoints) {
    if (sealedSegs)  *sealedSegs  = flSealedCount;
    if (totalPoints) *totalPoints = flTotalPoints;
}

// 删最旧封段（磁盘满时保最新）。
static bool flDropOldest() {
    if (flOldestSeq >= flCurSeq) return false;          // 没有已封段可删
    char path[40]; flSegPath(path, sizeof(path), flOldestSeq);
    uint16_t pts = 0;
    File f = LittleFS.open(path, "r");
    if (f) { pts = (uint16_t)(f.size() / FL_REC); f.close(); }
    LittleFS.remove(path);
    if (flSealedCount) flSealedCount--;
    flTotalPoints = (flTotalPoints >= pts) ? (flTotalPoints - pts) : 0;
    Serial.printf("[FL] 磁盘满 → 删最旧段 #%lu(%u点)\n", (unsigned long)flOldestSeq, pts);
    flOldestSeq++;
    return true;
}

// 追加一个点：满段先封，空间不足先丢最旧，再写入并每点落盘。
static void flashLogAppend(const TrackPoint& p) {
    if (!flReady) return;
    if (flCurPoints >= FL_SEG_POINTS) {                 // 当前段满 → 封段、滚到新段
        flSealedCount++;
        flCurSeq++; flCurPoints = 0;
    }
    while (LittleFS.totalBytes() - LittleFS.usedBytes() < (size_t)(FL_REC * 64))
        if (!flDropOldest()) break;                     // 留 ~一个扇区余量

    char path[40]; flSegPath(path, sizeof(path), flCurSeq);
    File f = LittleFS.open(path, "a");                  // "a"：不存在则创建
    if (!f) { Serial.println("[FL] 追加打开失败"); return; }
    f.write((const uint8_t*)&p, FL_REC);
    f.flush();                                          // 每点落盘 → 抗重启/掉电
    f.close();
    flCurPoints++; flTotalPoints++;
}

// 上传前把当前半段临封成段（让"传完即彻底清空"）。
static void flSealCurrentForUpload() {
    if (flCurPoints == 0) return;
    flSealedCount++;
    flCurSeq++; flCurPoints = 0;                        // 新空当前段（下次 append 才建文件）
}

// 读一个段文件全部点，在【已打开的 SH 会话】上分批 catmSHReq（不重建 TLS）。
// 全部 200 → true 并回报点数；任一批失败 → false（保留整段，下次重发，服务端去重）。
static bool flReqSegment(const char* path, uint16_t* nOut) {
    File f = LittleFS.open(path, "r");
    if (!f) { *nOut = 0; return false; }
    uint16_t n = (uint16_t)(f.size() / FL_REC);
    if (n > FL_SEG_POINTS) n = FL_SEG_POINTS;
    static TrackPoint pts[FL_SEG_POINTS];
    f.read((uint8_t*)pts, (size_t)n * FL_REC);
    f.close();
    *nOut = n;

    static char body[FL_BODYLEN];        // 一个段(≤20点≈2.3KB)一次发完
    uint16_t i = 0;
    while (i < n) {
        uint16_t take = (uint16_t)((n - i) < FL_BATCH ? (n - i) : FL_BATCH);
        int pos = 0; body[pos++] = '[';
        for (uint16_t k = 0; k < take; k++) {
            char one[160];
            int m = fmtPoint(one, sizeof(one), pts[i + k]);
            if (k) body[pos++] = ',';
            memcpy(body + pos, one, m); pos += m;
        }
        body[pos++] = ']'; body[pos] = 0;
        int code = catmSHReq(body, pos);                // 复用已开会话，只发请求
        Serial.printf("[FL] SHREQ %u 点 -> HTTP %d\n", take, code);
        if (code != 200 && code != 201) return false;
        i += take;
    }
    return true;
}

// 上传所有封段（最旧先发、发完即删）。返回已上传点数。调用方须已切到 LTE。
// ★只开一次 SH 会话、连发所有段的所有批、最后关一次——把 TLS 握手从"每批一次"降到
//   "每次 flush 一次"。撞 SH 锁的自愈仍在 catmSHOpen 内（每次 flush 至多一次）。
static int flashLogUpload() {
    if (!flReady) return 0;
    flSealCurrentForUpload();                           // 半段也封进来，传完彻底空
    if (flSealedCount == 0) return 0;
    if (!catmSHOpen(FL_BODYLEN)) { Serial.println("[FL] SH 会话打开失败，保留积压"); return 0; }
    int uploaded = 0;
    while (flSealedCount > 0) {
        char path[40]; flSegPath(path, sizeof(path), flOldestSeq);
        if (!LittleFS.exists(path)) { flSealedCount--; flOldestSeq++; continue; }  // 空洞安全
        uint16_t n = 0;
        if (flReqSegment(path, &n)) {
            LittleFS.remove(path);
            flSealedCount--; flOldestSeq++;
            flTotalPoints = (flTotalPoints >= n) ? (flTotalPoints - n) : 0;
            uploaded += n;
        } else break;                                   // 某批失败 → 停，保留本段及其后
    }
    catmSHClose();                                      // 整批发完，关一次会话
    return uploaded;
}

// ── 台面实测辅助（仅配置B 的串口测试命令用；绕过记录路径直接灌点量上传耗时）────────
#if GNSS_TIMESHARE
// 注入 n 个测试假点。坐标 35.0x/139.0（明显的测试区；事后服务端按 lat<35.5 清理，
// 绝不碰真实轨迹 35.65/139.74）。每点唯一 ts，便于去重测试。
static void flashLogFillTest(int n) {
    static uint32_t testTs = 1735689600UL;              // 2025-01-01 起，每点 +1s
    for (int i = 0; i < n; i++) {
        TrackPoint p;
        p.ts      = testTs++;
        p.lat     = (int32_t)lround((35.0 + (testTs % 1000) * 0.0001) * 1e7);
        p.lon     = (int32_t)lround(139.0 * 1e7);
        p.alt     = 10; p.spd = 5; p.sat = 7; p.hdop = 12;
        p.bat_mv  = batMv; p.bat_pct = (uint8_t)batPct;
        flashLogAppend(p);
    }
}

// 清空整个段日志（删 /t 下所有段文件，重置计数）。段序号连续 [oldest, cur]。
static void flashLogClear() {
    if (!flReady) return;
    for (uint32_t s = flOldestSeq; s <= flCurSeq; s++) {
        char path[40]; flSegPath(path, sizeof(path), s);
        LittleFS.remove(path);                          // 缺失即忽略
    }
    flCurSeq = flOldestSeq = 0; flCurPoints = 0; flSealedCount = 0; flTotalPoints = 0;
}

#endif // GNSS_TIMESHARE
