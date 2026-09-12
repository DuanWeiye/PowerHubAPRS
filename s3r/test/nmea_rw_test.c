// nmea_rw_test.c — nmea_rw.h 主机侧单元测试（g++ 编译，无 Arduino 依赖）
//
//   cd s3r/test && g++ -O1 -Wall -o nmea_rw_test nmea_rw_test.c && ./nmea_rw_test
//
// 用例含 2026-08-08 真机（ATGM336H 屋内）抓到的原始语句（校验和是真的），
// 以及模拟 fuse.ino 锁存/DR 改写路径的字段替换 → 重建 → 反解回读。
#include "../nmea_rw.h"

static int nfail = 0;
#define CHECK(cond, msg) do { \
    if (!(cond)) { printf("FAIL: %s\n", msg); nfail++; } \
    else         { printf("ok:   %s\n", msg); } \
} while (0)

// 造一条带正确校验和的行（模拟"真实有效定位"输入）
static void mkline(char* out, size_t cap, const char* payload) {
    snprintf(out, cap, "$%s*%02X", payload, nmeaChecksum(payload));
}

int main() {
    // ── 1. 校验和：真机抓到的原始语句 ──────────────────────────────────────────
    CHECK(nmeaChecksumOk("$GNRMC,,V,,,,,,,,,,N,V*37"), "checksum real invalid RMC");
    CHECK(nmeaChecksumOk("$GNGGA,,,,,,0,00,25.5,,,,,,*64"), "checksum real invalid GGA");
    CHECK(nmeaChecksumOk("$GPGSV,1,1,00,1*64"), "checksum real GSV");
    CHECK(nmeaChecksumOk("$GPTXT,01,01,01,ANTENNA OPEN*25"), "checksum real TXT");
    CHECK(!nmeaChecksumOk("$GNRMC,,V,,,,,,,,,,N,V*38"), "corrupted checksum rejected");
    CHECK(!nmeaChecksumOk("GNRMC,,V*37"), "no-$ rejected");
    CHECK(!nmeaChecksumOk("$GNRMC,,V"), "no-star rejected");

    // ── 2. split + rebuild 恒等（原样重建 = 原行 + CRLF）─────────────────────────
    {
        const char* orig = "$GNGGA,,,,,,0,00,25.5,,,,,,*64";
        char work[128], out[140];
        strcpy(work, orig);
        char* f[NMEA_MAX_FIELDS];
        int n = nmeaSplit(work, f, NMEA_MAX_FIELDS);
        CHECK(n == 15, "GGA split field count 15");
        int len = nmeaRebuild(out, sizeof(out), f, n);
        CHECK(len > 0 && strncmp(out, orig, strlen(orig)) == 0
              && strcmp(out + strlen(orig), "\r\n") == 0, "rebuild identity + CRLF");
    }
    {
        char work[128];
        strcpy(work, "$GNRMC,,V,,,,,,,,,,N,V*37");
        char* f[NMEA_MAX_FIELDS];
        int n = nmeaSplit(work, f, NMEA_MAX_FIELDS);
        CHECK(n == 14, "RMC(4.1) split field count 14");
        CHECK(strcmp(f[2], "V") == 0 && strcmp(f[12], "N") == 0, "RMC status/mode fields");
    }

    // ── 3. 经纬度 解析↔格式化 往返 ──────────────────────────────────────────────
    {
        double d;
        CHECK(nmeaLatLonParse("3540.87416", "N", &d) && fabs(d - 35.681236) < 1e-8,
              "lat parse 3540.87416N");
        CHECK(nmeaLatLonParse("13946.02750", "E", &d) && fabs(d - 139.767125) < 1e-6,
              "lon parse 13946.02750E");
        CHECK(nmeaLatLonParse("3540.87416", "S", &d) && d < 0, "S hemisphere negative");
        CHECK(!nmeaLatLonParse("", "N", &d), "empty latlon rejected");

        char dm[NMEA_NUM_BUF]; char h;
        nmeaLatLonFmt(35.681236, false, dm, sizeof(dm), &h);
        CHECK(strcmp(dm, "3540.87416") == 0 && h == 'N', "lat fmt round-trip");
        nmeaLatLonFmt(139.767125, true, dm, sizeof(dm), &h);
        CHECK(strcmp(dm, "13946.02750") == 0 && h == 'E', "lon fmt round-trip");
        nmeaLatLonFmt(-35.1000005, false, dm, sizeof(dm), &h);
        CHECK(h == 'S', "S hemisphere fmt");
        nmeaLatLonFmt(35.9999999, false, dm, sizeof(dm), &h);   // 59.999994' → 进位保护
        double back;
        nmeaLatLonParse(dm, &h, &back);
        CHECK(fabs(back - 35.9999999) < 5e-7, "carry guard near 60'");
        // 低分(mm<10)带前导零
        nmeaLatLonFmt(35.1, false, dm, sizeof(dm), &h);
        CHECK(strcmp(dm, "3506.00000") == 0, "leading-zero minutes");
    }

    // ── 4. 锁存改写：有效 GGA 坐标替换（模拟 fuseHandleGGA latch 路径）───────────
    {
        char line[140], work[140], out[160];
        mkline(line, sizeof(line),
               "GNGGA,062735.000,3540.87416,N,13946.02750,E,1,08,1.2,45.6,M,39.0,M,,");
        CHECK(nmeaChecksumOk(line), "constructed valid GGA checksum");
        strcpy(work, line);
        char* f[NMEA_MAX_FIELDS];
        int n = nmeaSplit(work, f, NMEA_MAX_FIELDS);
        double latchLat = 35.6812400, latchLon = 139.7671300;
        char dmLat[NMEA_NUM_BUF], dmLon[NMEA_NUM_BUF], hLat[2] = {0}, hLon[2] = {0};
        nmeaLatLonFmt(latchLat, false, dmLat, sizeof(dmLat), &hLat[0]);
        nmeaLatLonFmt(latchLon, true,  dmLon, sizeof(dmLon), &hLon[0]);
        f[2] = dmLat; f[3] = hLat; f[4] = dmLon; f[5] = hLon;
        CHECK(nmeaRebuild(out, sizeof(out), f, n) > 0, "latch GGA rebuild");
        out[strcspn(out, "\r\n")] = 0;
        CHECK(nmeaChecksumOk(out), "latch GGA output checksum valid");
        // 反解回读
        char w2[160]; strcpy(w2, out);
        char* g[NMEA_MAX_FIELDS];
        nmeaSplit(w2, g, NMEA_MAX_FIELDS);
        double rl, rn;
        CHECK(nmeaLatLonParse(g[2], g[3], &rl) && fabs(rl - latchLat) < 2e-7,
              "latch GGA lat round-trip");
        CHECK(nmeaLatLonParse(g[4], g[5], &rn) && fabs(rn - latchLon) < 2e-7,
              "latch GGA lon round-trip");
        CHECK(strcmp(g[6], "1") == 0 && strcmp(g[8], "1.2") == 0,
              "latch GGA untouched fields kept");
    }

    // ── 5. DR 改写：真机无效 GGA → 填坐标/quality=6/hdop/alt ─────────────────────
    {
        char work[140], out[160];
        strcpy(work, "$GNGGA,,,,,,0,00,25.5,,,,,,*64");   // 真机抓的
        char* f[NMEA_MAX_FIELDS];
        int n = nmeaSplit(work, f, NMEA_MAX_FIELDS);
        double drLat = 35.6815000, drLon = 139.7675000;
        char dmLat[NMEA_NUM_BUF], dmLon[NMEA_NUM_BUF], hLat[2] = {0}, hLon[2] = {0};
        char qual[2] = "6", altBuf[NMEA_NUM_BUF];
        nmeaLatLonFmt(drLat, false, dmLat, sizeof(dmLat), &hLat[0]);
        nmeaLatLonFmt(drLon, true,  dmLon, sizeof(dmLon), &hLon[0]);
        snprintf(altBuf, sizeof(altBuf), "%.1f", 45.6);
        f[2] = dmLat; f[3] = hLat; f[4] = dmLon; f[5] = hLon; f[6] = qual;
        if (!f[9][0]) f[9] = altBuf;
        CHECK(nmeaRebuild(out, sizeof(out), f, n) > 0, "DR GGA rebuild");
        out[strcspn(out, "\r\n")] = 0;
        CHECK(nmeaChecksumOk(out), "DR GGA output checksum valid");
        char w2[160]; strcpy(w2, out);
        char* g[NMEA_MAX_FIELDS];
        int n2 = nmeaSplit(w2, g, NMEA_MAX_FIELDS);
        double rl, rn;
        CHECK(n2 == 15, "DR GGA field count preserved");
        CHECK(strcmp(g[6], "6") == 0, "DR GGA quality=6 (estimated)");
        CHECK(nmeaLatLonParse(g[2], g[3], &rl) && fabs(rl - drLat) < 2e-7 &&
              nmeaLatLonParse(g[4], g[5], &rn) && fabs(rn - drLon) < 2e-7,
              "DR GGA coords round-trip");
        CHECK(strcmp(g[9], "45.6") == 0 && strcmp(g[8], "25.5") == 0,
              "DR GGA alt filled, hdop kept");
    }

    // ── 6. DR 改写：真机无效 RMC → status=A/坐标/速度/航向/mode=E ────────────────
    {
        char work[140], out[160];
        strcpy(work, "$GNRMC,,V,,,,,,,,,,N,V*37");        // 真机抓的
        char* f[NMEA_MAX_FIELDS];
        int n = nmeaSplit(work, f, NMEA_MAX_FIELDS);
        double drLat = 35.6815000, drLon = 139.7675000;
        char dmLat[NMEA_NUM_BUF], dmLon[NMEA_NUM_BUF], hLat[2] = {0}, hLon[2] = {0};
        char st[2] = "A", spd[NMEA_NUM_BUF], crs[NMEA_NUM_BUF];
        nmeaLatLonFmt(drLat, false, dmLat, sizeof(dmLat), &hLat[0]);
        nmeaLatLonFmt(drLon, true,  dmLon, sizeof(dmLon), &hLon[0]);
        snprintf(spd, sizeof(spd), "%.2f", 1.4 / 0.514444);   // 步速 1.4m/s → 节
        snprintf(crs, sizeof(crs), "%.1f", 123.4);
        f[2] = st; f[3] = dmLat; f[4] = hLat; f[5] = dmLon; f[6] = hLon;
        f[7] = spd; f[8] = crs;
        if (n >= 13) f[12] = (char*)"E";
        CHECK(nmeaRebuild(out, sizeof(out), f, n) > 0, "DR RMC rebuild");
        out[strcspn(out, "\r\n")] = 0;
        CHECK(nmeaChecksumOk(out), "DR RMC output checksum valid");
        char w2[160]; strcpy(w2, out);
        char* g[NMEA_MAX_FIELDS];
        int n2 = nmeaSplit(w2, g, NMEA_MAX_FIELDS);
        CHECK(n2 == 14 && strcmp(g[2], "A") == 0 && strcmp(g[12], "E") == 0,
              "DR RMC status=A mode=E");
        CHECK(strcmp(g[7], "2.72") == 0 && strcmp(g[8], "123.4") == 0,
              "DR RMC speed/course");
        printf("      sample DR RMC: %s\n", out);
    }

    // ── 6b. v2 改写 API（fuse.ino v0.3 实际调用路径）：真机语句 → apply/invalidate → 回读 ──
    {
        // 有效 GGA（东京站，真实格式）：跟踪点只换坐标，quality/HDOP/星数原样
        char line[140], work[140], out[160];
        mkline(line, sizeof(line),
               "GNGGA,014530.00,3540.87416,N,13946.02750,E,1,12,0.9,25.3,M,39.1,M,,");
        strcpy(work, line);
        char* f[NMEA_MAX_FIELDS];
        int n = nmeaSplit(work, f, NMEA_MAX_FIELDS);
        NmeaEdit e;
        double kfLat = 35.681300, kfLon = 139.767200;      // KF 输出（离原点 ~8m）
        CHECK(nmeaGgaApply(f, n, kfLat, kfLon, false, 5.0f, 25.3f, &e), "v2 GGA apply (fix)");
        CHECK(nmeaRebuild(out, sizeof(out), f, n) > 0, "v2 GGA rebuild");
        out[strcspn(out, "\r\n")] = 0;
        CHECK(nmeaChecksumOk(out), "v2 GGA checksum");
        char w2[160]; strcpy(w2, out);
        char* g[NMEA_MAX_FIELDS];
        int n2 = nmeaSplit(w2, g, NMEA_MAX_FIELDS);
        double rl, rn;
        CHECK(n2 == 15 && nmeaLatLonParse(g[2], g[3], &rl) && fabs(rl - kfLat) < 2e-7
              && nmeaLatLonParse(g[4], g[5], &rn) && fabs(rn - kfLon) < 2e-7, "v2 GGA coords");
        CHECK(strcmp(g[1], "014530.00") == 0 && strcmp(g[6], "1") == 0 && strcmp(g[7], "12") == 0
              && strcmp(g[8], "0.9") == 0 && strcmp(g[9], "25.3") == 0, "v2 GGA other fields kept");

        // 无效 GGA（真机抓的）→ 推算点：quality=6、HDOP/海拔补值
        strcpy(work, "$GNGGA,104759.80,,,,,0,00,9.6,,,,,,*71");
        n = nmeaSplit(work, f, NMEA_MAX_FIELDS);
        CHECK(nmeaGgaApply(f, n, kfLat, kfLon, true, 5.0f, 25.3f, &e), "v2 GGA apply (est)");
        nmeaRebuild(out, sizeof(out), f, n); out[strcspn(out, "\r\n")] = 0;
        strcpy(w2, out); n2 = nmeaSplit(w2, g, NMEA_MAX_FIELDS);
        CHECK(nmeaChecksumOk(out) && strcmp(g[6], "6") == 0 && strcmp(g[8], "9.6") == 0
              && strcmp(g[9], "25.3") == 0, "v2 GGA est: q=6, hdop kept(9.6), alt filled");

        // 有效 GGA 但估计器拒绝（野点）→ quality=0，坐标保留
        mkline(line, sizeof(line),
               "GNGGA,014530.00,3540.87416,N,13946.02750,E,1,12,0.9,25.3,M,39.1,M,,");
        strcpy(work, line); n = nmeaSplit(work, f, NMEA_MAX_FIELDS);
        CHECK(nmeaGgaInvalidate(f, n, &e), "v2 GGA invalidate");
        nmeaRebuild(out, sizeof(out), f, n); out[strcspn(out, "\r\n")] = 0;
        strcpy(w2, out); n2 = nmeaSplit(w2, g, NMEA_MAX_FIELDS);
        CHECK(nmeaChecksumOk(out) && strcmp(g[6], "0") == 0 && strcmp(g[2], "3540.87416") == 0,
              "v2 GGA invalidated: q=0 coords kept");

        // 有效 RMC → 跟踪点：status A、坐标/速度/航向换估计、模式保持 A
        mkline(line, sizeof(line),
               "GNRMC,014530.00,A,3540.87416,N,13946.02750,E,2.72,123.4,220826,,,A,V");
        strcpy(work, line); n = nmeaSplit(work, f, NMEA_MAX_FIELDS);
        CHECK(nmeaRmcApply(f, n, kfLat, kfLon, 15.0f, 271.5f, true, false, &e), "v2 RMC apply (fix)");
        nmeaRebuild(out, sizeof(out), f, n); out[strcspn(out, "\r\n")] = 0;
        strcpy(w2, out); n2 = nmeaSplit(w2, g, NMEA_MAX_FIELDS);
        CHECK(nmeaChecksumOk(out) && n2 == 14 && strcmp(g[2], "A") == 0 && strcmp(g[12], "A") == 0,
              "v2 RMC fix: status A, mode kept A");
        CHECK(nmeaLatLonParse(g[3], g[4], &rl) && fabs(rl - kfLat) < 2e-7
              && strcmp(g[7], "29.16") == 0 && strcmp(g[8], "271.5") == 0 && strcmp(g[9], "220826") == 0,
              "v2 RMC fix: coords/15m/s=29.16kn/course/date");

        // 无效 RMC（真机）→ 推算点：A + E，航向未知留空
        strcpy(work, "$GNRMC,104759.80,V,,,,,,,220826,,,N,V*13");
        n = nmeaSplit(work, f, NMEA_MAX_FIELDS);
        CHECK(nmeaRmcApply(f, n, kfLat, kfLon, 0.3f, 0, false, true, &e), "v2 RMC apply (est)");
        nmeaRebuild(out, sizeof(out), f, n); out[strcspn(out, "\r\n")] = 0;
        strcpy(w2, out); n2 = nmeaSplit(w2, g, NMEA_MAX_FIELDS);
        CHECK(nmeaChecksumOk(out) && strcmp(g[2], "A") == 0 && strcmp(g[12], "E") == 0
              && g[8][0] == 0 && strcmp(g[7], "0.58") == 0, "v2 RMC est: A/E, course empty");
        printf("      sample v2 est RMC: %s\n", out);

        // 有效 RMC 被拒 → V/N
        mkline(line, sizeof(line),
               "GNRMC,014530.00,A,3540.87416,N,13946.02750,E,2.72,123.4,220826,,,A,V");
        strcpy(work, line); n = nmeaSplit(work, f, NMEA_MAX_FIELDS);
        CHECK(nmeaRmcInvalidate(f, n, &e), "v2 RMC invalidate");
        nmeaRebuild(out, sizeof(out), f, n); out[strcspn(out, "\r\n")] = 0;
        strcpy(w2, out); n2 = nmeaSplit(w2, g, NMEA_MAX_FIELDS);
        CHECK(nmeaChecksumOk(out) && strcmp(g[2], "V") == 0 && strcmp(g[12], "N") == 0,
              "v2 RMC invalidated: V/N");
        // 西/南半球坐标也走一遍（改写格式化的半球字符）
        strcpy(work, line); n = nmeaSplit(work, f, NMEA_MAX_FIELDS);
        nmeaRmcApply(f, n, -33.8688, -70.6693, 1.0f, 0, false, false, &e);   // 圣地亚哥
        nmeaRebuild(out, sizeof(out), f, n); out[strcspn(out, "\r\n")] = 0;
        strcpy(w2, out); n2 = nmeaSplit(w2, g, NMEA_MAX_FIELDS);
        CHECK(strcmp(g[4], "S") == 0 && strcmp(g[6], "W") == 0
              && nmeaLatLonParse(g[3], g[4], &rl) && fabs(rl + 33.8688) < 2e-7, "v2 RMC S/W hemisphere");
    }

    // ── 7. 类型判断 ────────────────────────────────────────────────────────────
    CHECK(nmeaIsType("$GNGGA", "GGA") && nmeaIsType("$GPRMC", "RMC")
          && !nmeaIsType("$GNGSV", "GGA") && !nmeaIsType("$PFUSE", "USE") == false,
          "type check");   // $PFUSE: f0+3="USE" —— 恰不与 GGA/RMC 冲突即可

    printf("\n%s (%d failures)\n", nfail ? "FAILED" : "ALL PASS", nfail);
    return nfail ? 1 : 0;
}
