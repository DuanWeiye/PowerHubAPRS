#!/usr/bin/env python3
# ota_flash.py — S3R UART-OTA 刷写端（DGX 侧）
#
# 与 s3r/ota.ino 的接收器配对。链路两种：
#   台面直测：--port 指 S3R 自身 USB 口（默认值）
#   装机之后：--port 指 PowerHub 的 USB 口。本脚本开口后先发一行 "s3rbridge"（PowerHub
#             控制台命令）把它带进透传桥，之后字节原样进 Grove 链路，本协议不变。
#             桥在刷完后 60s 无 USB 数据自动退出，PowerHub 恢复正常运行。
#
# 协议（行式 ASCII 头 + 原始二进制块，逐块 ACK，MD5 端到端校验）：
#   → $S3R,OTA,BEGIN,<size>,<md5hex>     ← $S3R,OTA,READY
#   → $S3R,OTA,DATA,<seq>,<len>,<crc32hex>\n + <len> 字节   ← $S3R,OTA,ACK,<seq> / NAK
#   → $S3R,OTA,END                       ← $S3R,OTA,OK → 设备重启
#   →（重连）$S3R,PING                    ← $S3R,CONFIRMED + $S3R,PONG,<ver>,<part>
# 刷完必须 PING 确认：新固件不被确认，重启 3 次后自动回滚旧槽（见 ota.ino 守卫）。

import argparse
import hashlib
import sys
import time
import zlib

import serial

DEF_PORT = "/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_14:C1:9F:D5:B8:64-if00"
CHUNK = 1024


def rd_s3r_line(ser, timeout):
    """读一行 $S3R 应答；透传桥模式下链路里混着 NMEA 等噪声，非 $S3R 行一律丢弃。"""
    end = time.time() + timeout
    while time.time() < end:
        raw = ser.readline()
        if not raw:
            continue
        s = raw.decode(errors="ignore").strip()
        if s.startswith("$S3R"):
            return s
    return None


def main():
    ap = argparse.ArgumentParser(description="S3R UART-OTA flasher")
    ap.add_argument("--port", default=DEF_PORT)
    ap.add_argument("--bin", default="s3r.bin")
    ap.add_argument("--baud", type=int, default=115200)
    args = ap.parse_args()

    data = open(args.bin, "rb").read()
    md5 = hashlib.md5(data).hexdigest()
    size = len(data)
    print(f"[OTA] {args.bin}: {size} bytes, md5={md5}")
    print(f"[OTA] port: {args.port}")

    ser = serial.Serial(args.port, args.baud, timeout=0.5)
    time.sleep(0.3)
    # 若对面是 PowerHub：控制台命令进透传桥。直连 S3R 时是未知命令，应答（含
    # "$S3R,ERR,unknown:…"）随下面的 reset_input_buffer 一并丢弃，无害。
    ser.write(b"s3rbridge\n")
    time.sleep(0.5)
    ser.reset_input_buffer()

    # ── BEGIN ──
    # 等待放宽到 35s：PowerHub 可能正阻塞在发包/AT 重试里（15-25s），进桥会晚。
    ser.write(f"$S3R,OTA,BEGIN,{size},{md5}\r\n".encode())
    r = rd_s3r_line(ser, 35)
    if r != "$S3R,OTA,READY":
        sys.exit(f"[OTA] no READY (got: {r})")
    print("[OTA] device ready, sending...")

    # ── DATA：逐块 stop-and-wait，NAK/超时重发 ──
    seq, pos, t0, last_pct = 0, 0, time.time(), -1
    while pos < size:
        chunk = data[pos:pos + CHUNK]
        crc = zlib.crc32(chunk) & 0xFFFFFFFF
        hdr = f"$S3R,OTA,DATA,{seq},{len(chunk)},{crc:08x}\n".encode()
        for attempt in range(4):
            ser.write(hdr)
            ser.write(chunk)
            ser.flush()
            r = rd_s3r_line(ser, 5)
            if r == f"$S3R,OTA,ACK,{seq}":
                break
            print(f"\n[OTA] seq {seq} attempt {attempt + 1}: {r or 'timeout'}, retrying")
        else:
            ser.write(b"$S3R,OTA,ABORT\r\n")
            sys.exit(f"[OTA] chunk {seq} failed after retries")
        pos += len(chunk)
        seq += 1
        pct = pos * 100 // size
        if pct != last_pct:                      # 百分比变化才刷一行，避免日志刷屏
            last_pct = pct
            rate = pos / max(time.time() - t0, 0.001) / 1024
            print(f"\r[OTA] {pct:3d}%  {pos}/{size}  {rate:.1f} KB/s", end="", flush=True)
    print()

    # ── END ──
    ser.write(b"$S3R,OTA,END\r\n")
    r = rd_s3r_line(ser, 20)
    if r != "$S3R,OTA,OK":
        sys.exit(f"[OTA] END failed (got: {r})")
    print("[OTA] flash OK, device rebooting...")
    ser.close()

    # ── 重连 + PING 确认（不确认则设备 3 次重启后回滚）──
    time.sleep(2)
    for _ in range(30):
        try:
            ser = serial.Serial(args.port, args.baud, timeout=0.5)
        except (serial.SerialException, OSError):
            time.sleep(1)
            continue
        time.sleep(0.5)
        ser.reset_input_buffer()
        ser.write(b"$S3R,PING\r\n")
        deadline = time.time() + 3
        confirmed, pong = False, None
        while time.time() < deadline:
            r = rd_s3r_line(ser, max(deadline - time.time(), 0.1))
            if r == "$S3R,CONFIRMED":
                confirmed = True
            elif r and r.startswith("$S3R,PONG"):
                pong = r
                break
        ser.close()
        if pong:
            print(f"[OTA] {pong}" + ("  (confirmed)" if confirmed else "  (already confirmed)"))
            print("[OTA] SUCCESS")
            return
        time.sleep(1)
    sys.exit("[OTA] device did not answer PING after reboot — firmware will roll back")


if __name__ == "__main__":
    main()
