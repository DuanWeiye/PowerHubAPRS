#!/usr/bin/env python3
# imu_fetch.py — 拉取 S3R 的 IMU/GNSS 环形日志（DGX 侧）
#
# 与 s3r/imulog.ino 的 $S3R,IMUDUMP 协议配对（分块 CRC32 + ACK，镜像 OTA 方向反转）。
# 链路两种（同 ota_flash.py）：
#   装机后：--port 指 PowerHub USB 口，脚本自动先发 "s3rbridge" 进透传桥
#   台面：  --port 指 S3R 自身 USB 口（bridge 命令是未知命令，无害）
# 输出：拼接所有段为单个 .bin（默认 imulog_<时间戳>.bin），用 imu_decode.py 解码。
# --clear：拉取成功后清空设备上的日志。

import argparse
import sys
import time
import zlib
from datetime import datetime

import serial

DEF_PORT = "/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_1C:DB:D4:A8:27:C4-if00"


def rd_s3r_line(ser, timeout):
    end = time.time() + timeout
    while time.time() < end:
        raw = ser.readline()
        if not raw:
            continue
        s = raw.decode(errors="ignore").strip()
        if s.startswith("$S3R"):
            return s
    return None


def read_exact(ser, n, timeout):
    buf = b""
    end = time.time() + timeout
    while len(buf) < n and time.time() < end:
        buf += ser.read(n - len(buf))
    return buf


def main():
    ap = argparse.ArgumentParser(description="S3R IMU log fetcher")
    ap.add_argument("--port", default=DEF_PORT, help="PowerHub 口(默认)或 S3R USB 口")
    ap.add_argument("--out", default=None)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--clear", action="store_true", help="拉取成功后清空设备日志")
    args = ap.parse_args()

    out = args.out or f"imulog_{datetime.now():%Y%m%d_%H%M%S}.bin"
    ser = serial.Serial(args.port, args.baud, timeout=0.5)
    time.sleep(0.3)
    ser.write(b"s3rbridge\n")          # PowerHub 进桥；直连 S3R 时无害
    time.sleep(0.5)
    ser.reset_input_buffer()

    ser.write(b"$S3R,IMUDUMP\r\n")
    r = rd_s3r_line(ser, 35)           # PowerHub 可能正阻塞在发包里，进桥会晚
    if not r or not r.startswith("$S3R,IMUDUMP,BEGIN,"):
        sys.exit(f"[IL] no BEGIN (got: {r})")
    nsegs, total = r.split(",")[3], int(r.split(",")[4])
    print(f"[IL] {nsegs} segs, {total} bytes total")

    data = b""
    t0 = time.time()
    while True:
        r = rd_s3r_line(ser, 10)
        if r is None:
            sys.exit("[IL] timeout mid-transfer")
        if r.startswith("$S3R,IMUDUMP,SEG,"):
            print(f"[IL] {r.split(',',3)[3]}")
            continue
        if r.startswith("$S3R,IMUDUMP,DATA,"):
            p = r.split(",")
            seq, ln, crc = int(p[3]), int(p[4]), int(p[5], 16)
            body = read_exact(ser, ln, 5)
            if len(body) == ln and (zlib.crc32(body) & 0xFFFFFFFF) == crc:
                ser.write(f"$S3R,IMUDUMP,ACK,{seq}\r\n".encode())
                data += body
                if total:
                    pct = len(data) * 100 // total
                    rate = len(data) / max(time.time() - t0, 0.001) / 1024
                    print(f"\r[IL] {pct:3d}%  {len(data)}/{total}  {rate:.1f} KB/s",
                          end="", flush=True)
            else:
                # 不 ACK → 设备端重发同一块
                print(f"\n[IL] seq {seq} bad ({len(body)}/{ln} bytes), waiting resend")
            continue
        if r.startswith("$S3R,IMUDUMP,END"):
            print()
            break
        if r.startswith("$S3R,IMUDUMP,ERR"):
            sys.exit(f"\n[IL] device error: {r}")
        # 其它 $S3R 行（心跳 PONG 等）忽略

    with open(out, "wb") as fp:
        fp.write(data)
    print(f"[IL] saved {len(data)} bytes -> {out}")

    if args.clear:
        # 桥上应答偶发丢行：明确等 CLEARED，收不到就重发（CLEAR 幂等）
        r = None
        for _ in range(3):
            ser.write(b"$S3R,IMULOG,CLEAR\r\n")
            end = time.time() + 10
            while time.time() < end:
                s = rd_s3r_line(ser, 2)
                if s and s.startswith("$S3R,IMULOG,CLEARED"):
                    r = s
                    break
            if r:
                break
        print(f"[IL] clear: {r if r else '未确认(3次重试无应答,建议手动重试)'}")
    ser.close()
    print("[IL] SUCCESS  (decode: python3 imu_decode.py " + out + ")")


if __name__ == "__main__":
    main()
