// replay.cpp — 主机回放器：把 imulog 解码出的 _gnss.csv/_imu.csv 灌进 nav_core，
// 输出逐拍决策 CSV 供 metrics.py 评分。P1 的核心工具：估计器先在实测数据上跑赢
// 基线，才允许上机（0808 教训：桌面拍脑袋定参数直接上线 = 外场翻车）。
//
// 两种模式（--base 切换）：
//   nav  = v2 完整核心（停留收敛 + ZUPT + 断档推算 + 野点门 + NIS）
//   base = PowerHub track.ino 现行管线的忠实复刻（同一套方程/常数；停留收敛/ZUPT/
//          推算全部禁用）——作为"无 S3R 时"的对照基线
//
// 用法: g++ -O2 -o replay replay.cpp && ./replay <prefix> [--base] > out.csv
//   <prefix> 如 ../imulog_20260808_222804（自动接 _gnss.csv/_imu.csv）
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>
#include "../nav_core.h"

struct GnssRow { uint32_t ms; double lat, lon; float spd, crs, hdop; int sats, valid; };
struct ImuRow  { uint32_t ms; float a[3], g[3]; };
struct FeatRow { uint32_t ms; float stdMean, stdMax, gyro, headRate; int stat; };

static bool loadGnss(const char* path, std::vector<GnssRow>& v) {
    FILE* f = fopen(path, "r");
    if (!f) return false;
    char line[256];
    fgets(line, sizeof(line), f);                       // 表头
    while (fgets(line, sizeof(line), f)) {
        GnssRow r;
        double alt;
        if (sscanf(line, "%u,%lf,%lf,%lf,%f,%f,%f,%d,%d", &r.ms, &r.lat, &r.lon,
                   &alt, &r.spd, &r.crs, &r.hdop, &r.sats, &r.valid) != 9) continue;
        // 物理合法性 + 时间戳单调:环形日志 bad_sync 残渣可能漏进解码(0808 见 1 条
        // hdop=4813/sats=63/ms 乱序的损坏行),按通用判据丢弃,不针对特定值
        if (fabs(r.lat) > 90 || fabs(r.lon) > 180 || r.hdop < 0 || r.hdop > 100 ||
            r.sats < 0 || r.sats > 60 || r.spd < 0 || r.spd > 200) continue;
        if (!v.empty() && (r.ms <= v.back().ms || r.ms - v.back().ms > 600000)) continue;
        v.push_back(r);
    }
    fclose(f);
    return true;
}
static bool loadImu(const char* path, std::vector<ImuRow>& v) {
    FILE* f = fopen(path, "r");
    if (!f) return false;
    char line[256];
    fgets(line, sizeof(line), f);
    while (fgets(line, sizeof(line), f)) {
        ImuRow r;
        if (sscanf(line, "%u,%f,%f,%f,%f,%f,%f", &r.ms, &r.a[0], &r.a[1], &r.a[2],
                   &r.g[0], &r.g[1], &r.g[2]) != 7) continue;
        if (fabsf(r.a[0]) > 100 || fabsf(r.a[1]) > 100 || fabsf(r.a[2]) > 100) continue;
        if (!v.empty() && (r.ms <= v.back().ms || r.ms - v.back().ms > 600000)) continue;
        v.push_back(r);
    }
    fclose(f);
    return true;
}
// 摘要层设备自算特征块（P2 起的首选 IMU 源：窗口尺度与固件一致，无复算漂移）
static bool loadFeat(const char* path, std::vector<FeatRow>& v) {
    FILE* f = fopen(path, "r");
    if (!f) return false;
    char line[256];
    if (!fgets(line, sizeof(line), f)) { fclose(f); return false; }
    while (fgets(line, sizeof(line), f)) {
        FeatRow r;
        if (sscanf(line, "%u,%f,%f,%f,%f,%d", &r.ms, &r.stdMean, &r.stdMax,
                   &r.gyro, &r.headRate, &r.stat) != 6) continue;
        if (r.stdMax < 0 || r.stdMax > 60) continue;
        if (!v.empty() && (r.ms <= v.back().ms || r.ms - v.back().ms > 600000)) continue;
        v.push_back(r);
    }
    fclose(f);
    return true;
}

// ── IMU 特征复算（25Hz 日志 → 1s accStd 窗 + 航向角速率）───────────────────────
// 与 imu.ino 同算法，系数按 25Hz 换算。注：窗口尺度与固件 0.25s 块不同（25Hz 日志
// 撑不起 0.25s 统计），阈值语义近似；P2 起固件直接落"设备自算的特征块"消除该差异。
struct ImuFeat {
    float gravLp[3] = {0};
    bool  gravInit = false;
    float bias[3] = {0};
    bool  biasInit = false;
    float headRate = 0;
    float winSum = 0, winSumSq = 0;
    int   winN = 0;
    float accStd = 0;                              // 最近完成的 1s 窗
    bool  fresh = false;                           // 有新窗产出

    void feed(const ImuRow& r) {
        if (!gravInit) { gravInit = true; memcpy(gravLp, r.a, sizeof(gravLp)); }
        else for (int i = 0; i < 3; i++) gravLp[i] += 0.02f * (r.a[i] - gravLp[i]);
        // 零偏：准静态样本（|ω-bias|<3dps）慢 EMA；首样本直接置入
        float gd[3] = { r.g[0] - bias[0], r.g[1] - bias[1], r.g[2] - bias[2] };
        float gmag = sqrtf(gd[0]*gd[0] + gd[1]*gd[1] + gd[2]*gd[2]);
        if (!biasInit) { memcpy(bias, r.g, sizeof(bias)); biasInit = true; }
        else if (gmag < 3.0f)
            for (int i = 0; i < 3; i++) bias[i] += 0.002f * (r.g[i] - bias[i]);
        float gn = sqrtf(gravLp[0]*gravLp[0] + gravLp[1]*gravLp[1] + gravLp[2]*gravLp[2]);
        if (gn > 1.0f) {
            float hr = -(gd[0]*gravLp[0] + gd[1]*gravLp[1] + gd[2]*gravLp[2]) / gn;
            headRate += 0.2f * (hr - headRate);
        }
        float amag = sqrtf(r.a[0]*r.a[0] + r.a[1]*r.a[1] + r.a[2]*r.a[2]);
        winSum += amag; winSumSq += amag * amag;
        if (++winN >= 25) {                        // 1s @25Hz
            float mean = winSum / winN;
            float var  = winSumSq / winN - mean * mean;
            accStd = sqrtf(var > 0 ? var : 0);
            winSum = winSumSq = 0; winN = 0;
            fresh = true;
        }
    }
};

// --set 名=值 扫参表（P2 调参用；名字与 NavParams 字段一致）
struct PrmEntry { const char* name; float NavParams::*f; uint32_t NavParams::*u; };
static const PrmEntry kPrmTab[] = {
    {"glMaxMps", &NavParams::glMaxMps, nullptr},
    {"posSigmaM", &NavParams::posSigmaM, nullptr},
    {"accStdMove", &NavParams::accStdMove, nullptr},
    {"accStdStill", &NavParams::accStdStill, nullptr},
    {"stillSpdMax", &NavParams::stillSpdMax, nullptr},
    {"stillInnovMax", &NavParams::stillInnovMax, nullptr},
    {"stillEnterSec", &NavParams::stillEnterSec, nullptr},
    {"zuptOn", &NavParams::zuptOn, nullptr},
    {"zuptSigma", &NavParams::zuptSigma, nullptr},
    {"moveSpdReset", &NavParams::moveSpdReset, nullptr},
    {"accStdActive", &NavParams::accStdActive, nullptr},
    {"activeHoldSec", &NavParams::activeHoldSec, nullptr},
    {"coastDecayPerS", &NavParams::coastDecayPerS, nullptr},
    {"sigmaEmitMaxM", &NavParams::sigmaEmitMaxM, nullptr},
    {"crsMinMps", &NavParams::crsMinMps, nullptr},
    {"accStdCoastCarried", &NavParams::accStdCoastCarried, nullptr},
    {"coastHdopRef", &NavParams::coastHdopRef, nullptr},
    {"coastHdopMax", &NavParams::coastHdopMax, nullptr},
    {"reacqPedSpdMps", &NavParams::reacqPedSpdMps, nullptr},
    {"reacqWalkMps", &NavParams::reacqWalkMps, nullptr},
    {"reacqMarginM", &NavParams::reacqMarginM, nullptr},
    {"reacqGateMinGapMs", nullptr, &NavParams::reacqGateMinGapMs},
    {"reacqGateMaxGapMs", nullptr, &NavParams::reacqGateMaxGapMs},
    {"reacqVehEvidenceMs", nullptr, &NavParams::reacqVehEvidenceMs},
    {"glForceMs", nullptr, &NavParams::glForceMs},
    {"glResyncMs", nullptr, &NavParams::glResyncMs},
    {"coastStillCapMs", nullptr, &NavParams::coastStillCapMs},
};

int main(int argc, char** argv) {
    const char* prefix = nullptr;
    bool baseline = false;
    std::vector<const char*> sets;
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--base")) baseline = true;
        else if (!strcmp(argv[i], "--set") && i + 1 < argc) sets.push_back(argv[++i]);
        else prefix = argv[i];
    }
    if (!prefix) { fprintf(stderr, "usage: replay <prefix> [--base] [--set 名=值 ...]\n"); return 1; }
    char path[512];
    std::vector<GnssRow> gnss;
    std::vector<ImuRow>  imu;
    std::vector<FeatRow> feat;
    snprintf(path, sizeof(path), "%s_gnss.csv", prefix);
    if (!loadGnss(path, gnss) || gnss.empty()) {   // 全速率层缺失 → 摘要层 1Hz 全天回放
        snprintf(path, sizeof(path), "%s_sgnss.csv", prefix);
        if (!loadGnss(path, gnss)) { fprintf(stderr, "no gnss/sgnss csv\n"); return 1; }
    }
    snprintf(path, sizeof(path), "%s_feat.csv", prefix);
    bool haveFeat = loadFeat(path, feat) && !feat.empty();
    snprintf(path, sizeof(path), "%s_imu.csv", prefix);
    if (!loadImu(path, imu) && !haveFeat) { fprintf(stderr, "no imu/feat csv\n"); return 1; }
    fprintf(stderr, "[replay] gnss=%zu imu=%zu feat=%zu(%s) mode=%s\n",
            gnss.size(), imu.size(), feat.size(),
            haveFeat ? "device" : "recompute", baseline ? "base" : "nav");

    NavParams prm;
    navParamsDefault(&prm);
    if (baseline) {
        // 复刻 PowerHub 现行管线：过程噪声恒定（无停留收敛）、无 ZUPT、无推算输出。
        prm.accStdStill = prm.accStdMove;
        prm.zuptOn      = 2.0f;                   // still 永远到不了 → ZUPT 关
        prm.sigmaEmitMaxM = 0.0f;                 // coast 永不发点
    }
    for (const char* s : sets) {                   // --set 覆盖（在 base 预设之后）
        const char* eq = strchr(s, '=');
        if (!eq) { fprintf(stderr, "bad --set %s\n", s); return 1; }
        bool hit = false;
        for (const auto& e : kPrmTab) {
            if (strncmp(s, e.name, eq - s) || strlen(e.name) != (size_t)(eq - s)) continue;
            if (e.f) prm.*(e.f) = (float)atof(eq + 1);
            else     prm.*(e.u) = (uint32_t)atol(eq + 1);
            hit = true; break;
        }
        if (!hit) { fprintf(stderr, "unknown param %.*s\n", (int)(eq - s), s); return 1; }
        fprintf(stderr, "[replay] set %s\n", s);
    }
    NavCore nav;
    navInit(&nav, &prm);

    ImuFeat fc;                                    // 无 feat.csv 时的 25Hz 复算兜底
    size_t gi = 0, ii = 0, fi = 0;
    // 输出：ms,kind(F=实测拍/C=推算拍),emit,est,lat,lon,spd,crs,crsv,sigma,still,
    //       rlat,rlon,rvalid,rhdop,rspd(原始参考)
    printf("ms,kind,emit,est,lat,lon,spd_mps,crs_deg,crs_valid,sigma_m,still,"
           "raw_lat,raw_lon,raw_valid,raw_hdop,raw_spd\n");
    while (gi < gnss.size() || (haveFeat ? fi < feat.size() : ii < imu.size())) {
        if (haveFeat) {                            // 设备自算特征（用 stdMax：秒内单个
            bool takeFeat = fi < feat.size() &&    // 响块即活动，贴近固件按块喂的语义）
                            (gi >= gnss.size() || feat[fi].ms <= gnss[gi].ms);
            if (takeFeat) {
                if (!baseline)
                    navImu(&nav, feat[fi].ms, feat[fi].stdMax, feat[fi].headRate);
                fi++;
                continue;
            }
        } else {
            bool takeImu = ii < imu.size() &&
                           (gi >= gnss.size() || imu[ii].ms <= gnss[gi].ms);
            if (takeImu) {
                fc.feed(imu[ii]);
                if (fc.fresh && !baseline) {       // 基线不吃 IMU（PowerHub 没有 IMU）
                    fc.fresh = false;
                    navImu(&nav, imu[ii].ms, fc.accStd, fc.headRate);
                }
                ii++;
                continue;
            }
        }
        const GnssRow& r = gnss[gi++];
        NavOut o;
        bool got = false;
        char kind = 'F';
        if (r.valid && r.lat != 0) {
            got = navGnss(&nav, r.ms, r.lat, r.lon, r.hdop,
                          r.spd, r.crs > 0 ? r.crs : -1, &o);
            if (!got) { kind = 'C'; got = navCoast(&nav, r.ms, &o); }  // 野点拍→推算
        } else {
            kind = 'C';
            got = navCoast(&nav, r.ms, &o);
        }
        printf("%u,%c,%d,%d,%.7f,%.7f,%.2f,%.1f,%d,%.1f,%.2f,%.7f,%.7f,%d,%.1f,%.2f\n",
               r.ms, kind, got && o.emit ? 1 : 0, got && o.est ? 1 : 0,
               got ? o.lat : 0.0, got ? o.lon : 0.0,
               got ? o.spdMps : 0.0f, got ? o.crsDeg : 0.0f,
               got ? (o.crsValid ? 1 : 0) : 0,
               got ? o.sigmaM : 0.0f, got ? o.still : 0.0f,
               r.lat, r.lon, r.valid, r.hdop, r.spd);
    }
    return 0;
}
