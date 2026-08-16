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

    // ── 7. 类型判断 ────────────────────────────────────────────────────────────
    CHECK(nmeaIsType("$GNGGA", "GGA") && nmeaIsType("$GPRMC", "RMC")
          && !nmeaIsType("$GNGSV", "GGA") && !nmeaIsType("$PFUSE", "USE") == false,
          "type check");   // $PFUSE: f0+3="USE" —— 恰不与 GGA/RMC 冲突即可

    printf("\n%s (%d failures)\n", nfail ? "FAILED" : "ALL PASS", nfail);
    return nfail ? 1 : 0;
}
