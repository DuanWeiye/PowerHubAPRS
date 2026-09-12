// nav_core_test.c — nav_core.h 主机侧场景单测（g++ 编译，无 Arduino 依赖）
//   cd s3r/test && g++ -O1 -Wall -o nav_core_test nav_core_test.c && ./nav_core_test
// 场景全部取自真实携带情形（0912/0816 外场），坐标用东京站地标（公开），不碰真实轨迹。
#include <stdio.h>
#include <stdlib.h>
#include "../nav_core.h"

static int fails = 0, checks = 0;
#define CHECK(cond, ...) do { checks++; if (!(cond)) { fails++; printf("  FAIL L%d: ", __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

static const double LAT0 = 35.681236, LON0 = 139.767125;   // 东京站
static double kx;                                           // 1 度经度的米数

typedef struct { int emits, ests, rejects; float maxSpd; double lastLat, lastLon; float lastSigma; } Stat;
static void statReset(Stat* s) { memset(s, 0, sizeof(*s)); }
static void record(Stat* s, const NavOut* o, bool got) {
    if (!got || !o->emit) return;
    s->emits++; if (o->est) s->ests++;
    if (o->spdMps > s->maxSpd) s->maxSpd = o->spdMps;
    s->lastLat = o->lat; s->lastLon = o->lon; s->lastSigma = o->sigmaM;
}
// 一拍：valid 定位（含 ~1m 噪声）→ 返回"量测是否被接受"（被拒时按 coast 处理，输出照记）
static bool fix(NavCore* n, Stat* s, uint32_t ms, double lat, double lon, float hdop, float spd, float crs) {
    NavOut o; bool acc = navGnss(n, ms, lat + 0.7e-5 * sin(ms * 0.37), lon + 0.7e-5 * cos(ms * 0.53), hdop, spd, crs, &o);
    bool got = acc;
    if (!acc) { s->rejects++; got = navCoast(n, ms, &o); }
    record(s, &o, got); return acc;
}
static bool nofix(NavCore* n, Stat* s, uint32_t ms) {
    NavOut o; bool got = navCoast(n, ms, &o); record(s, &o, got); return got && o.emit;
}
// 沿东向匀速走一段（5Hz），并喂 IMU 步行特征；返回终点经度
static double walk(NavCore* n, Stat* s, uint32_t* ms, double lat, double lon, float spd, int sec, float accStd) {
    for (int i = 0; i < sec * 5; i++) {
        *ms += 200; lon += spd * 0.2 / kx;
        if (i % 5 == 0) navImu(n, *ms, accStd, 0);
        fix(n, s, *ms, lat, lon, 0.9f, spd, 90.0f);
    }
    return lon;
}
static void gap(NavCore* n, Stat* s, uint32_t* ms, int sec, float accStd) {
    for (int i = 0; i < sec * 5; i++) { *ms += 200; if (i % 5 == 0) navImu(n, *ms, accStd, 0); nofix(n, s, *ms); }
}
static void setup(NavCore* n) { NavParams p; navParamsDefault(&p); navInit(n, &p); }

int main() {
    kx = 111320.0 * cos(LAT0 * M_PI / 180.0);
    NavCore n; Stat s; uint32_t ms;

    printf("场景1 行人进楼：步行 60s → 断档 63s → 424m 外单拍伪定位 → 4s 后 350m 处 3 拍（0912 复现）\n");
    setup(&n); statReset(&s); ms = 1000;
    double lon = walk(&n, &s, &ms, LAT0, LON0, 1.3f, 60, 1.5f);
    double lastLon = s.lastLon;
    statReset(&s);
    gap(&n, &s, &ms, 63, 0.4f);                        // 楼内：IMU 安静（电梯/静立，0912 实测 0.4）
    CHECK(s.emits < 250 && !nofix(&n, &s, ms + 200), "63s 断档内 coast 应在 σ 越界后停发（emits=%d）", s.emits);
    statReset(&s);
    ms += 200; bool e1 = fix(&n, &s, ms, LAT0 + 424.0 / 111320.0, lon, 3.8f, 1.9f, 10.0f);
    CHECK(!e1 && s.rejects == 1, "424m 伪点应被可达性门拒收（emit=%d rej=%d）", e1, s.rejects);
    ms += 3800;
    for (int i = 0; i < 3; i++) { ms += 200; fix(&n, &s, ms, LAT0 + 350.0 / 111320.0, lon, 2.3f, 1.0f, 10.0f); }
    CHECK(s.emits == 0 && s.rejects == 4, "350m 处 3 拍也应拒收（emits=%d rej=%d）", s.emits, s.rejects);
    CHECK(s.maxSpd < 5.0f, "全程不得出现飞点速度（max %.1f m/s）", s.maxSpd);
    (void)lastLon;

    printf("场景2 行人真走远：步行 → 断档 40s → 120m 前方重捕获（可达）→ 接受且速度合理\n");
    setup(&n); statReset(&s); ms = 1000;
    lon = walk(&n, &s, &ms, LAT0, LON0, 1.3f, 60, 1.5f);
    gap(&n, &s, &ms, 40, 1.5f); statReset(&s);
    double lon2 = lon + 120.0 / kx;
    for (int i = 0; i < 10; i++) { ms += 200; fix(&n, &s, ms, LAT0, lon2, 0.9f, 1.3f, 90.0f); lon2 += 1.3 * 0.2 / kx; }
    CHECK(s.rejects == 0 && s.emits == 10, "可达的重捕获应立即接受（rej=%d emits=%d）", s.rejects, s.emits);
    CHECK(s.maxSpd < 4.0f, "重建后速度应≈步行（max %.1f m/s）", s.maxSpd);
    CHECK(fabs((s.lastLon - lon2) * kx) < 15, "输出应贴住新位置（差 %.0fm）", fabs((s.lastLon - lon2) * kx));

    printf("场景3 电车隧道：20m/s → 断档 60s → 1200m 前方重捕获 → 车辆语境不受门限影响\n");
    setup(&n); statReset(&s); ms = 1000;
    lon = walk(&n, &s, &ms, LAT0, LON0, 20.0f, 30, 0.2f);
    gap(&n, &s, &ms, 60, 0.2f); statReset(&s);
    lon2 = lon + 1200.0 / kx;
    for (int i = 0; i < 10; i++) { ms += 200; fix(&n, &s, ms, LAT0, lon2, 0.9f, 20.0f, 90.0f); lon2 += 20.0 * 0.2 / kx; }
    CHECK(s.rejects == 0 && s.emits == 10, "车辆语境重捕获应直接接受（rej=%d emits=%d）", s.rejects, s.emits);
    CHECK(s.maxSpd > 15.0f && s.maxSpd < 30.0f, "重建后速度应≈自报 20m/s（max %.1f）", s.maxSpd);

    printf("场景4 长断档：停留 → 断档 400s → 3km 外重捕获（进店后打车）→ 门关闭，接受\n");
    setup(&n); statReset(&s); ms = 1000;
    for (int i = 0; i < 300; i++) { ms += 200; if (i % 5 == 0) navImu(&n, ms, 0.05f, 0); fix(&n, &s, ms, LAT0, LON0, 0.9f, 0.0f, -1.0f); }
    CHECK(n.still > 0.8f, "60s 停留后 still 应满（%.2f）", n.still);
    gap(&n, &s, &ms, 400, 0.05f); statReset(&s);
    ms += 200; bool e4 = fix(&n, &s, ms, LAT0, LON0 + 3000.0 / kx, 1.0f, 8.0f, 90.0f);
    CHECK(e4 && s.rejects == 0, "超过 3 分钟的断档不设门（emit=%d rej=%d）", e4, s.rejects);

    printf("场景5 行人短断档 8s + 80m 多径跳点：先拒；回到原地立刻接受；断档 10s 后持续 100m 偏移 → 界限追上时放行\n");
    setup(&n); statReset(&s); ms = 1000;
    lon = walk(&n, &s, &ms, LAT0, LON0, 1.3f, 30, 1.5f);
    gap(&n, &s, &ms, 8, 1.5f); statReset(&s);
    ms += 200; bool e5 = fix(&n, &s, ms, LAT0 + 80.0 / 111320.0, lon, 1.5f, 1.0f, 90.0f);
    CHECK(!e5 && s.rejects == 1, "8.2s 断档界=62.8m，80m 多径跳点应拒（acc=%d）", e5);
    CHECK(s.emits == 0 || fabs((s.lastLat - LAT0) * 111320.0) < 30, "拒收后若有 coast 点也应在原路附近（偏 %.0fm）", fabs((s.lastLat - LAT0) * 111320.0));
    ms += 200; bool e5b = fix(&n, &s, ms, LAT0, lon + 1.3 * 8.6 / kx, 0.9f, 1.3f, 90.0f);
    CHECK(e5b, "回到真实位置的下一拍应立即接受（emit=%d）", e5b);
    lon = s.lastLon;
    gap(&n, &s, &ms, 10, 1.5f); statReset(&s);
    // 断档 10s 后持续 100m 偏移：界限 = age×4+30，age 17.5s 时追上 → 约第 37 拍放行
    int acceptedAt = -1;
    for (int i = 0; i < 100; i++) { ms += 200; if (fix(&n, &s, ms, LAT0 + 100.0 / 111320.0, lon, 1.0f, 0.0f, -1.0f) && acceptedAt < 0) acceptedAt = i; }
    CHECK(acceptedAt > 30 && acceptedAt < 45, "持续 100m 偏移应在界限追上时放行（第 %d 拍）", acceptedAt);

    printf("场景6 0,0 坏句：valid 但坐标 0,0 → 拒收且不动状态\n");
    setup(&n); statReset(&s); ms = 1000;
    lon = walk(&n, &s, &ms, LAT0, LON0, 1.3f, 10, 1.5f); statReset(&s);
    ms += 200; bool e6 = fix(&n, &s, ms, 0.0, 0.0, 25.5f, 1.15f, 0.0f);
    CHECK(!e6 && s.rejects == 1 && fabs(n.glLat - LAT0) < 0.001, "0,0 应拒收（acc=%d glLat=%.4f）", e6, n.glLat);
    ms += 200; bool e6b = fix(&n, &s, ms, LAT0, lon + 1.3 * 0.4 / kx, 0.9f, 1.3f, 90.0f);
    CHECK(e6b, "下一拍正常定位应照常接受");

    printf("场景7 正常步行 5Hz 不受门影响：600 拍零拒绝\n");
    setup(&n); statReset(&s); ms = 1000;
    walk(&n, &s, &ms, LAT0, LON0, 1.3f, 120, 1.5f);
    CHECK(s.rejects == 0 && s.emits == 600, "rej=%d emits=%d", s.rejects, s.emits);

    printf("场景8 电车出站：站内停 30s → 断档 20s → 365m 外重捕获、自报 18m/s（0912 08:47 复现）→ 2s 车速证据后放行\n");
    setup(&n); statReset(&s); ms = 1000;
    for (int i = 0; i < 150; i++) { ms += 200; if (i % 5 == 0) navImu(&n, ms, 0.15f, 0); fix(&n, &s, ms, LAT0, LON0, 0.9f, 0.0f, -1.0f); }
    gap(&n, &s, &ms, 20, 0.15f); statReset(&s);
    lon2 = LON0 + 365.0 / kx; acceptedAt = -1;
    for (int i = 0; i < 30; i++) { ms += 200; if (fix(&n, &s, ms, LAT0, lon2, 0.9f, 18.0f, 90.0f) && acceptedAt < 0) acceptedAt = i; lon2 += 18.0 * 0.2 / kx; }
    CHECK(acceptedAt >= 8 && acceptedAt <= 11, "车速证据 2s 后应放行（第 %d 拍）", acceptedAt);
    CHECK(s.maxSpd > 12.0f && s.maxSpd < 30.0f, "放行后速度≈车速（max %.1f）", s.maxSpd);

    printf("场景9 行人进楼 + 单拍伪多普勒：断档 60s 后 400m 外一拍自报 20m/s、随后候选自报 1m/s → 不放行\n");
    setup(&n); statReset(&s); ms = 1000;
    lon = walk(&n, &s, &ms, LAT0, LON0, 1.3f, 60, 1.5f);
    gap(&n, &s, &ms, 60, 0.4f); statReset(&s);
    ms += 200; fix(&n, &s, ms, LAT0 + 400.0 / 111320.0, lon, 2.0f, 20.0f, 0.0f);
    for (int i = 0; i < 20; i++) { ms += 200; fix(&n, &s, ms, LAT0 + 400.0 / 111320.0, lon, 2.0f, 1.0f, 0.0f); }
    CHECK(s.rejects == 21 && s.emits == 0, "单拍伪车速不构成证据（rej=%d emits=%d）", s.rejects, s.emits);

    printf("%s：%d 项检查，%d 失败\n", fails ? "FAILED" : "PASSED", checks, fails);
    return fails ? 1 : 0;
}
