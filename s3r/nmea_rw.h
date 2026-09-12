// nmea_rw.h — NMEA 行解析/改写/重建的纯 C 核心（无 Arduino 依赖）
//
// 供 fuse.ino 的融合改写用；同一份头被 test/nmea_rw_test.c 在主机上 g++ 编译单测
// （改写坐标错一位就是把人送进海里，必须能在主机上用真实语句验证后再上机）。
// 全部 static inline，无全局状态。
#pragma once
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#define NMEA_MAX_FIELDS 24
#define NMEA_NUM_BUF    16    // 单个替换字段的格式化缓冲

// ── 校验和：'$' 与 '*' 之间所有字节异或 ─────────────────────────────────────────
static inline uint8_t nmeaChecksum(const char* s) {
    uint8_t cs = 0;
    if (*s == '$') s++;
    while (*s && *s != '*' && *s != '\r' && *s != '\n') cs ^= (uint8_t)*s++;
    return cs;
}

// 行是否带合法校验和（"$....*hh"）。改写只对校验通过的行做，坏行原样放行。
static inline bool nmeaChecksumOk(const char* s) {
    if (*s != '$') return false;
    const char* star = strchr(s, '*');
    if (!star || strlen(star) < 3) return false;
    unsigned given = (unsigned)strtoul(star + 1, NULL, 16);
    return nmeaChecksum(s) == (uint8_t)given;
}

// ── 字段切分（在 work 副本上原地切，'*' 前的逗号分段；返回字段数）───────────────
// f[0]="$GNGGA"，之后每个逗号后为一字段。'*' 与其后校验和被截掉。
static inline int nmeaSplit(char* work, char* f[], int maxf) {
    int n = 0;
    char* star = strchr(work, '*');
    if (star) *star = 0;
    char* p = work;
    f[n++] = p;
    while (*p && n < maxf) {
        if (*p == ',') { *p = 0; f[n++] = p + 1; }
        p++;
    }
    return n;
}

// ── 重建："$f0,f1,...*hh\r\n"（返回长度，容量不足返回 -1）─────────────────────
static inline int nmeaRebuild(char* out, size_t cap, char* const f[], int n) {
    size_t pos = 0;
    for (int i = 0; i < n; i++) {
        size_t l = strlen(f[i]);
        if (pos + l + 2 >= cap) return -1;
        if (i) out[pos++] = ',';
        memcpy(out + pos, f[i], l);
        pos += l;
    }
    out[pos] = 0;
    uint8_t cs = nmeaChecksum(out);
    if (pos + 6 >= cap) return -1;
    pos += snprintf(out + pos, cap - pos, "*%02X\r\n", cs);
    return (int)pos;
}

// ── 坐标解析：ddmm.mmmm + 半球 → 十进制度（空字段返回 false）────────────────────
static inline bool nmeaLatLonParse(const char* dm, const char* hemi, double* deg) {
    if (!dm || !*dm || !hemi || !*hemi) return false;
    double v = atof(dm);
    double dd = floor(v / 100.0);
    double mm = v - dd * 100.0;
    double d = dd + mm / 60.0;
    if (*hemi == 'S' || *hemi == 'W') d = -d;
    *deg = d;
    return true;
}

// ── 坐标格式化：十进制度 → "ddmm.mmmmm"/"dddmm.mmmmm" + 半球字符 ────────────────
// 5 位小数分 ≈ 1.8cm 分辨率。isLon 控制度位宽（2/3 位）。
static inline void nmeaLatLonFmt(double deg, bool isLon, char* dmOut, size_t cap, char* hemiOut) {
    *hemiOut = isLon ? (deg < 0 ? 'W' : 'E') : (deg < 0 ? 'S' : 'N');
    double a = fabs(deg);
    int dd = (int)a;
    double mm = (a - dd) * 60.0;
    if (mm >= 59.999995) { dd++; mm = 0.0; }   // 进位保护：59.99999x 四舍五入到 60 会出 "xx60.00000"
    snprintf(dmOut, cap, isLon ? "%03d%08.5f" : "%02d%08.5f", dd, mm);
}

// ── 语句类型判断（跳过 2 字符 talker）────────────────────────────────────────────
static inline bool nmeaIsType(const char* f0, const char* ty3) {
    return f0[0] == '$' && strlen(f0) >= 6 && strncmp(f0 + 3, ty3, 3) == 0;
}

// ═══════════════════════════════════════════════════════════════════════════
// 融合改写（fuse.ino v2 用；nmeaSplit 之后、nmeaRebuild 之前，对字段指针数组原地替换）
// 全部只改"该改的字段"：其余字段（UTC 时间、星数、大地水准面差、DGPS 龄期…）原样保留，
// PowerHub 侧 TinyGPS++ 看到的仍是一条合法 GGA/RMC。
// ═══════════════════════════════════════════════════════════════════════════

// 替换字段的格式化缓冲（生命周期须覆盖到 nmeaRebuild 调用结束）
typedef struct {
    char dmLat[NMEA_NUM_BUF], dmLon[NMEA_NUM_BUF], hemiLat[2], hemiLon[2];
    char q[2], hdop[NMEA_NUM_BUF], alt[NMEA_NUM_BUF];    // GGA: quality / hdop 补值 / alt 补值
    char st[2], spd[NMEA_NUM_BUF], crs[NMEA_NUM_BUF], mode[2];   // RMC: status / 速度 / 航向 / 模式
} NmeaEdit;

static inline void nmeaEditLatLon(double lat, double lon, NmeaEdit* e, char** fLat, char** fLatH,
                                  char** fLon, char** fLonH) {
    nmeaLatLonFmt(lat, false, e->dmLat, sizeof(e->dmLat), &e->hemiLat[0]); e->hemiLat[1] = 0;
    nmeaLatLonFmt(lon, true,  e->dmLon, sizeof(e->dmLon), &e->hemiLon[0]); e->hemiLon[1] = 0;
    *fLat = e->dmLat; *fLatH = e->hemiLat; *fLon = e->dmLon; *fLonH = e->hemiLon;
}

// GGA（字段：1=UTC 2/3=纬度 4/5=经度 6=quality 7=星数 8=HDOP 9=海拔 10=M …）
//   est=false：只换坐标（quality/HDOP/星数保持原样——原句本来就是有效定位）。
//   est=true ：推算点。quality=6（NMEA 标准 "estimated/DR"）；原句无定位时 HDOP/海拔
//              字段是空的，补 hdopFill / altFill 让下游解析器别拿到空值。
//   返回 false = 字段数不足（调用方原样转发）。
static inline bool nmeaGgaApply(char* f[], int n, double lat, double lon, bool est,
                                float hdopFill, float altFill, NmeaEdit* e) {
    if (n < 10) return false;
    nmeaEditLatLon(lat, lon, e, &f[2], &f[3], &f[4], &f[5]);
    if (est) {
        e->q[0] = '6'; e->q[1] = 0; f[6] = e->q;
        if (!f[8][0]) { snprintf(e->hdop, sizeof(e->hdop), "%.1f", hdopFill); f[8] = e->hdop; }
        if (!f[9][0]) { snprintf(e->alt,  sizeof(e->alt),  "%.1f", altFill);  f[9] = e->alt;  }
    }
    return true;
}

// GGA：标为无定位（quality=0）。坐标等其它字段原样保留——下游 TinyGPS++ 以 quality>0
// 为"有定位"判据，置 0 即不采信本拍；保留原坐标是为了诚实（看得出模组报了什么）。
static inline bool nmeaGgaInvalidate(char* f[], int n, NmeaEdit* e) {
    if (n < 7) return false;
    e->q[0] = '0'; e->q[1] = 0; f[6] = e->q;
    return true;
}

// RMC（字段：1=UTC 2=状态A/V 3/4=纬度 5/6=经度 7=速度(节) 8=航向 9=日期 10/11=磁偏
//      12=模式指示(NMEA 2.3+：A 自主 / D 差分 / E 估算 / N 无效)）
//   坐标/速度/航向一律换成估计器输出（速度/航向来自 KF 速度矢量，与坐标自洽；
//   crsValid=false → 航向字段留空=未知，不伪造）。
//   est=true → 状态 A + 模式 E（原句可能是 V：推算点对下游是"有定位"，但模式字段诚实标注）。
//   est=false → 状态 A，模式字段保持原样。
static inline bool nmeaRmcApply(char* f[], int n, double lat, double lon, float spdMps,
                                float crsDeg, bool crsValid, bool est, NmeaEdit* e) {
    if (n < 9) return false;
    e->st[0] = 'A'; e->st[1] = 0; f[2] = e->st;
    nmeaEditLatLon(lat, lon, e, &f[3], &f[4], &f[5], &f[6]);
    snprintf(e->spd, sizeof(e->spd), "%.2f", spdMps / 0.514444f);   // m/s → 节
    f[7] = e->spd;
    if (crsValid) { snprintf(e->crs, sizeof(e->crs), "%.1f", crsDeg); f[8] = e->crs; }
    else          { e->crs[0] = 0; f[8] = e->crs; }
    if (est && n >= 13) { e->mode[0] = 'E'; e->mode[1] = 0; f[12] = e->mode; }
    return true;
}

// RMC：标为无效（状态 V；模式字段 N）。坐标等原样保留，理由同 nmeaGgaInvalidate。
static inline bool nmeaRmcInvalidate(char* f[], int n, NmeaEdit* e) {
    if (n < 3) return false;
    e->st[0] = 'V'; e->st[1] = 0; f[2] = e->st;
    if (n >= 13) { e->mode[0] = 'N'; e->mode[1] = 0; f[12] = e->mode; }
    return true;
}
