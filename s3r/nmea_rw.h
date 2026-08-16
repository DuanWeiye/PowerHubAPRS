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
