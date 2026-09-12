#!/usr/bin/env python3
# imu_decode.py — 解码 imu_fetch.py 拉回的 .bin → CSV（imu/gnss/fuse/events/feat/sgnss）
#
# 记录布局与 s3r/defs.h ILOG_* 一致（小端定长）。输出到 <in>_imu.csv 等四个文件。
# 单位换算：accel raw×8g/32768→m/s²；gyro raw×2000/32768→dps（与固件 IMU_*_RES 一致）。

import struct
import sys

ACC_RES = 8.0 / 32768.0 * 9.80665
GYR_RES = 2000.0 / 32768.0

EVT = {1: "LATCH_ON", 2: "LATCH_OFF", 3: "LATCH_ESC", 4: "DR_ON",
       5: "DR_OFF", 6: "DR_ABORT", 7: "BIAS_SET", 8: "IMU_FAIL", 9: "IL_WFAIL"}

# type -> (负载字节数, struct fmt)
LAYOUT = {
    0x01: (16, "<Ihhhhhh"),      # IMU: ms, a3, g3
    0x02: (21, "<IiihHHBBB"),    # GNSS: ms, lat, lon, alt, spd, crs, hdop, sats, flags
    0x03: (13, "<IiiB"),         # FUSE: ms, lat, lon, mode(1=FIX 2=COAST)
    0x04: (9,  "<IBf"),          # EVT: ms, code, val
    0x05: (16, "<Ifff"),         # BIAS: ms, b3
    0x06: (8,  "<II"),           # TIME: ms, epoch
    0x07: (13, "<IHHHhB"),       # FEAT: ms, accStdMean/Max mm/s2, gyro 0.01dps,
                                 #       headRate 0.01dps, flags(b0=stationary)
    0x08: (21, "<IiihHHBBB"),    # SGNS: 摘要层 1Hz GNSS，布局同 GNSS
}


def main():
    if len(sys.argv) < 2:
        sys.exit("usage: imu_decode.py <imulog.bin>")
    path = sys.argv[1]
    data = open(path, "rb").read()
    base = path.rsplit(".", 1)[0]

    fimu = open(base + "_imu.csv", "w")
    fgns = open(base + "_gnss.csv", "w")
    ffus = open(base + "_fuse.csv", "w")
    fevt = open(base + "_events.csv", "w")
    ffea = open(base + "_feat.csv", "w")
    fsgn = open(base + "_sgnss.csv", "w")
    fimu.write("ms,ax_ms2,ay_ms2,az_ms2,gx_dps,gy_dps,gz_dps\n")
    fgns.write("ms,lat,lon,alt_m,spd_mps,crs_deg,hdop,sats,valid\n")
    ffus.write("ms,lat,lon,mode\n")
    fevt.write("ms,event,val\n")
    ffea.write("ms,acc_std_mean,acc_std_max,gyro_dps,headrate_dps,stat\n")
    fsgn.write("ms,lat,lon,alt_m,spd_mps,crs_deg,hdop,sats,valid\n")

    # ── 流式解析 + 失步对齐 + 重复去除 ──────────────────────────────────────
    # 记录无魔数/CRC，仅靠类型字节定界。两类真实损坏（0822 外场实证）：
    #  ① 固件写盘偶发失败后重试 → 同一段数据（2s flush 周期或更长，最长见过 65s）
    #     被写两遍，第一遍在半条记录处截断 → 紧跟的几字节把解析带偏；
    #  ② 掉电/换段截断。
    # 老做法"未知字节跳 1B"会在 ① 处把错位字节误读成合法记录（IMU 0x01 很容易被
    # 命中），整段数据变垃圾。现在：每条记录要求与后继记录 tms 连贯（±1h）才接受，
    # 否则在 64B 窗口内搜"≥4 条连贯记录"的最小偏移重新对齐；重复写入的记录按
    # (字节完全相同 & tms 在最近 120s 窗内) 去重。
    MS_MAX = 200_000_000            # tms 理智上限（55h）
    COH_MS = 3_600_000              # 相邻记录 tms 连贯界（±1h；开机段切换另算）
    WIN_MS = 120_000                # 去重窗口

    def rec_at(p):
        """p 处若是合法记录返回 (type, size, tms)，否则 None"""
        if p >= len(data): return None
        t = data[p]
        if t not in LAYOUT: return None
        size, fmt = LAYOUT[t]
        if p + 1 + size > len(data): return None
        tms = struct.unpack_from("<I", data, p + 1)[0]
        if tms > MS_MAX: return None
        return t, size, tms

    def run_len(p, cap=8):
        """从 p 起连续连贯记录数（最多 cap 条）"""
        k, prev = 0, None
        while k < cap:
            r = rec_at(p)
            if r is None: break
            if prev is not None and abs(r[2] - prev) > COH_MS: break
            prev = r[2]; p += 1 + r[1]; k += 1
        return k

    pos, n, bad, dup, realign = 0, 0, 0, 0, 0
    counts = {}
    seen, seen_q = set(), []      # 去重：最近窗口内的原始记录字节
    last_tms = None
    while pos < len(data):
        r = rec_at(pos)
        ok = r is not None and run_len(pos, 2) >= (2 if pos + 1 + r[1] < len(data) else 1)
        if not ok:
            # 失步：64B 内找第一个 ≥4 条连贯记录的偏移（末尾不足 4 条时放宽）
            best = None
            for k in range(1, 65):
                if pos + k >= len(data): break
                rl = run_len(pos + k, 4)
                if rl >= 4 or (rl >= 1 and pos + k + 64 >= len(data)):
                    best = k; break
            if best is None:
                bad += 1; pos += 1; continue
            bad += best; pos += best; realign += 1
            continue
        t, size, tms = r
        raw = data[pos:pos + 1 + size]
        # 去重窗口维护（tms 大跳=开机段切换 → 清窗）
        if last_tms is not None and abs(tms - last_tms) > COH_MS:
            seen.clear(); seen_q.clear()
        last_tms = tms
        while seen_q and tms - seen_q[0][0] > WIN_MS:
            seen.discard(seen_q.pop(0)[1])
        if raw in seen:
            dup += 1; pos += 1 + size; continue
        seen.add(raw); seen_q.append((tms, raw))
        v = struct.unpack(LAYOUT[t][1], raw[1:])
        pos += 1 + size
        n += 1
        counts[t] = counts.get(t, 0) + 1
        if t == 0x01:
            fimu.write(f"{v[0]},{v[1]*ACC_RES:.4f},{v[2]*ACC_RES:.4f},{v[3]*ACC_RES:.4f},"
                       f"{v[4]*GYR_RES:.3f},{v[5]*GYR_RES:.3f},{v[6]*GYR_RES:.3f}\n")
        elif t == 0x02:
            fgns.write(f"{v[0]},{v[1]/1e7:.7f},{v[2]/1e7:.7f},{v[3]},{v[4]/100:.2f},"
                       f"{v[5]/10:.1f},{v[6]/10:.1f},{v[7]},{v[8]&1}\n")
        elif t == 0x03:
            ffus.write(f"{v[0]},{v[1]/1e7:.7f},{v[2]/1e7:.7f},"
                       f"{'FIX' if v[3]==1 else 'COAST'}\n")   # v0.3 起 1=FIX 2=COAST（v1 旧义 LATCH/DR）
        elif t == 0x04:
            fevt.write(f"{v[0]},{EVT.get(v[1], v[1])},{v[2]:.2f}\n")
        elif t == 0x05:
            fevt.write(f"{v[0]},BIAS,{v[1]:.3f}/{v[2]:.3f}/{v[3]:.3f}\n")
        elif t == 0x06:
            fevt.write(f"{v[0]},TIMEMARK,{v[1]}\n")
        elif t == 0x07:
            ffea.write(f"{v[0]},{v[1]/1000:.3f},{v[2]/1000:.3f},{v[3]/100:.2f},"
                       f"{v[4]/100:.2f},{v[5]&1}\n")
        elif t == 0x08:
            fsgn.write(f"{v[0]},{v[1]/1e7:.7f},{v[2]/1e7:.7f},{v[3]},{v[4]/100:.2f},"
                       f"{v[5]/10:.1f},{v[6]/10:.1f},{v[7]},{v[8]&1}\n")

    for f in (fimu, fgns, ffus, fevt, ffea, fsgn):
        f.close()
    names = {1: "IMU", 2: "GNSS", 3: "FUSE", 4: "EVT", 5: "BIAS", 6: "TIME",
             7: "FEAT", 8: "SGNS"}
    stat = "  ".join(f"{names[k]}={v}" for k, v in sorted(counts.items()))
    print(f"[DEC] {n} records ({stat})  bad_bytes={bad} realign={realign} dup_removed={dup}")
    print(f"[DEC] -> {base}_imu/_gnss/_fuse/_events/_feat/_sgnss .csv")


if __name__ == "__main__":
    main()
