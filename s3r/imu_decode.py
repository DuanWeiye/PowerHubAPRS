#!/usr/bin/env python3
# imu_decode.py — 解码 imu_fetch.py 拉回的 .bin → CSV（imu/gnss/fuse/events）
#
# 记录布局与 s3r/defs.h ILOG_* 一致（小端定长）。输出到 <in>_imu.csv 等四个文件。
# 单位换算：accel raw×8g/32768→m/s²；gyro raw×2000/32768→dps（与固件 IMU_*_RES 一致）。

import struct
import sys

ACC_RES = 8.0 / 32768.0 * 9.80665
GYR_RES = 2000.0 / 32768.0

EVT = {1: "LATCH_ON", 2: "LATCH_OFF", 3: "LATCH_ESC", 4: "DR_ON",
       5: "DR_OFF", 6: "DR_ABORT", 7: "BIAS_SET", 8: "IMU_FAIL"}

# type -> (负载字节数, struct fmt)
LAYOUT = {
    0x01: (16, "<Ihhhhhh"),      # IMU: ms, a3, g3
    0x02: (21, "<IiihHHBBB"),    # GNSS: ms, lat, lon, alt, spd, crs, hdop, sats, flags
    0x03: (13, "<IiiB"),         # FUSE: ms, lat, lon, mode
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

    pos, n, bad = 0, 0, 0
    counts = {}
    while pos < len(data):
        t = data[pos]
        pos += 1
        if t not in LAYOUT:
            bad += 1
            continue                       # 未知字节：跳 1B 重新同步（掉电截断段）
        size, fmt = LAYOUT[t]
        if pos + size > len(data):
            break
        v = struct.unpack(fmt, data[pos:pos + size])
        pos += size
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
                       f"{'LATCH' if v[3]==1 else 'DR'}\n")
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
    print(f"[DEC] {n} records ({stat})  bad_sync={bad}")
    print(f"[DEC] -> {base}_imu/_gnss/_fuse/_events/_feat/_sgnss .csv")


if __name__ == "__main__":
    main()
