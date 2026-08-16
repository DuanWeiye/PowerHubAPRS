// parse.ino — NMEA 旁路解析：GSV/TXT 星座·CN0·天线诊断 + 定位状态机
// gnssDiagLine 移植自主固件 pwrlog.ino（逻辑一致，勿双向漂移）。
#include "defs.h"

// 解析一整行 NMEA。只看 GSV（星座/CN0/可见星数）与 TXT（天线状态：ATGM336H 会在
// 开机及状态变化时报 $GPTXT,...,ANTENNA OK/OPEN/SHORT）；定位类语句由 TinyGPS++ 另收。
static void gnssDiagLine(const char* s) {
    if (s[0] != '$') return;
    const char* ty = s + 3;                       // 句型在 talker(2) 之后
    if (ty[0]=='T' && ty[1]=='X' && ty[2]=='T') {
        if      (strstr(s, "ANTENNA OK"))    gnssAnt = 1;
        else if (strstr(s, "ANTENNA OPEN"))  gnssAnt = 2;
        else if (strstr(s, "ANTENNA SHORT")) gnssAnt = 3;
        return;
    }
    if (!(ty[0]=='G' && ty[1]=='S' && ty[2]=='V')) return;
    const char* tk = s + 1;                       // talker 两字符
    int slot;
    if      (tk[0]=='G' && tk[1]=='P') slot = 0;  // GPS（QZSS 旧固件可能混在此，按 PRN 另判）
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
    gnssInView[slot] = (uint8_t)numSV;
    for (int b = 4; b + 3 < nf; b += 4) {
        int prn = atoi(f[b]);
        int snr = atoi(f[b + 3]);                 // 空字段 atoi=0
        if (snr > gnssAccCN0[slot]) gnssAccCN0[slot] = (uint8_t)snr;
        if (slot == 0 && prn >= 193 && prn <= 202 && snr > 0) {  // QZSS 混在 GPGSV 的兜底
            if (gnssInView[4] == 0) gnssInView[4] = 1;
            if (snr > gnssCN0[4]) gnssCN0[4] = (uint8_t)snr;
        }
    }
    if (msgNum >= totalMsgs) gnssCN0[slot] = gnssAccCN0[slot];   // 周期结束，提交最强 CN0
}

// 可见星总数（各星座求和）
static uint16_t gnssTotalInView() {
    uint16_t n = 0;
    for (int i = 0; i < 6; i++) n += gnssInView[i];
    return n;
}

// 全星座最强 CN0
static uint8_t gnssMaxCN0() {
    uint8_t m = 0;
    for (int i = 0; i < 6; i++) if (gnssCN0[i] > m) m = gnssCN0[i];
    return m;
}

// 定位状态机（只为屏显服务；透传数据流不经过它）
static void updateGpsState(uint32_t now) {
    if (gpsState == GS_DETECTING || gpsState == GS_NO_MODULE) {
        if (gpsChars > 20) gpsState = GS_SEARCHING;
        return;
    }
    // 有效且新鲜的定位 → FIX
    bool good = gps.location.isValid() && gps.location.age() < GPS_STALE_MS;
    gpsState = good ? GS_FIX_GOOD : GS_SEARCHING;

    // ANT? 判据：持续 0 可见星计时（真天线状态优先用 TXT 报告的 gnssAnt）
    if (gnssTotalInView() > 0) tSatsZeroSince = 0;
    else if (!tSatsZeroSince)  tSatsZeroSince = now;
}
