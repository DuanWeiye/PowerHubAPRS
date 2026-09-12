#!/usr/bin/env python3
# inject_test.py — 台面端到端验证融合链（不出门、不需要 GPS 信号）
#
# 链路：DGX USB → PowerHub 控制台 `gpsin $S3R,NMEA,<句>` → PORT.C 链路 → S3R ota.ino 注入钩子
#       → fuse.ino 估计器改写 → 回到 PowerHub 被 TinyGPS++/PFUSE 正常解析 → liveFix/beacon。
# 脚本合成一段 5Hz 轨迹（坐标用 35.0x/139.0x 测试区，和 flashlog 假点约定一致，绝不碰真实轨迹）：
#   A 静止 40s → B 步行 1.4m/s 60s → C 断档 20s（RMC V / GGA q=0）→ D 步行续 30s → E 静止 25s
# 期间每个阶段末用 `s3rnav` 读 PowerHub 侧状态（旁路在线 / liveFix est / 坐标），最后可选
# `sendtest` 强发一包看服务端遥测字段。预期（P3 验收项）：
#   · A/E：still 上升到 ≥0.8，liveFix 速度≈0、坐标钉住；
#   · C：est=1 的推算点持续 ≤ ~20s（σ 到 40m 停发），之后 PowerHub "Fix lost"；
#   · D：恢复后无传送（KF 从断档前状态续上）；全程旁路保持 ONLINE，不抖动。
# 用法：python3 inject_test.py [--port <PowerHub口>] [--send]   （--send 末尾强发一包到服务器）
#       python3 inject_test.py --reacq   断档重捕获场景（0912 进楼飞点复现 + 电车出站放行）：
#   A 静止 30s → B 步行 30s → C 断档 63s → D 424m 外伪定位 1s（自报 1.9m/s）→ E 断档 3s →
#   F 350m 外伪定位 3s（自报 1.0）→ G 断档 20s → H 原地真定位续走 10s → I 断档 20s →
#   J "电车"：365m 外起、北向 18m/s 15s（自报 18）。预期：D/F 期间 liveFix 不动（PowerHub 不
#   见新点，"Fix lost" 保持）；H 立即接受；J 约 2s 后 liveFix 跟上车。
import argparse, math, sys, time, re
from datetime import datetime, timezone
import serial

DEF_PORT = "/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_1C:DB:D4:A8:27:C4-if00"
LAT0, LON0 = 35.0520000, 139.0510000        # 测试区（非真实位置）


def cs(payload):
    c = 0
    for ch in payload.encode():
        c ^= ch
    return f"{payload}*{c:02X}"


def dm(deg, is_lon):
    a = abs(deg); d = int(a); m = (a - d) * 60
    return (f"{d:03d}{m:08.5f}" if is_lon else f"{d:02d}{m:08.5f}"), \
           ("E" if deg >= 0 else "W") if is_lon else ("N" if deg >= 0 else "S")


def epoch_sentences(t, lat, lon, valid, spd_mps, crs):
    utc = datetime.fromtimestamp(t, timezone.utc)
    hms = utc.strftime("%H%M%S") + f".{int((t % 1) * 100):02d}"
    dmy = utc.strftime("%d%m%y")
    if valid:
        la, lah = dm(lat, False); lo, loh = dm(lon, True)
        rmc = f"GNRMC,{hms},A,{la},{lah},{lo},{loh},{spd_mps / 0.514444:.2f},{crs:.1f},{dmy},,,A,V"
        gga = f"GNGGA,{hms},{la},{lah},{lo},{loh},1,12,0.9,25.3,M,39.1,M,,"
    else:
        rmc = f"GNRMC,{hms},V,,,,,,,{dmy},,,N,V"
        gga = f"GNGGA,{hms},,,,,0,00,9.6,,,,,,"
    return cs(rmc), cs(gga)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=DEF_PORT)
    ap.add_argument("--send", action="store_true", help="末尾 sendtest 强发一包（服务端会多一个测试区点）")
    ap.add_argument("--fast", action="store_true", help="各阶段时长减半")
    ap.add_argument("--reacq", action="store_true", help="断档重捕获场景（见文件头）")
    a = ap.parse_args()
    ser = serial.Serial(a.port, 115200, timeout=0.05)
    time.sleep(0.5)
    ser.reset_input_buffer()

    log = open("inject_test.log", "w")
    keep = re.compile(r"\[S3R\]|\[GPS\] (Fix|searching)|\[CM\] (body|POST|beacon)")

    def pump(sec):
        end = time.time() + sec
        while time.time() < end:
            l = ser.readline()
            if not l: continue
            l = l.decode(errors="replace").rstrip()
            log.write(l + "\n")
            if keep.search(l): print("   ", l)

    def cmd(c, wait=2.5):
        ser.write((c + "\n").encode()); pump(wait)

    k = 0.5 if a.fast else 1.0
    # 阶段元组：(名, 秒, 真实东向速度 m/s, 有定位?, 发出位置北向偏移 m, 偏移北向增速 m/s, 自报速度覆盖(None=真实), 航向)
    if a.reacq:
        phases = [("A 静止", 30 * k, 0.0, True, 0, 0, None, 90.0), ("B 步行→东", 30 * k, 1.4, True, 0, 0, None, 90.0),
                  ("C 断档", 63 * k, 0.0, False, 0, 0, None, 90.0), ("D 伪定位424m", 1, 0.0, True, 424, 0, 1.9, 10.0),
                  ("E 断档", 3, 0.0, False, 0, 0, None, 90.0), ("F 伪定位350m", 3, 0.0, True, 350, 0, 1.0, 10.0),
                  ("G 断档", 20 * k, 0.0, False, 0, 0, None, 90.0), ("H 真定位续走", 10, 1.4, True, 0, 0, None, 90.0),
                  ("I 断档", 20 * k, 0.0, False, 0, 0, None, 90.0), ("J 电车北向18m/s", 15, 0.0, True, 365, 18.0, 18.0, 0.0)]
    else:
        phases = [("A 静止", 40 * k, 0.0, True, 0, 0, None, 90.0), ("B 步行→东", 60 * k, 1.4, True, 0, 0, None, 90.0),
                  ("C 断档", 20 * k, 1.4, False, 0, 0, None, 90.0), ("D 步行续", 30 * k, 1.4, True, 0, 0, None, 90.0),
                  ("E 静止", 25 * k, 0.0, True, 0, 0, None, 90.0)]
    lat, lon = LAT0, LON0
    kx = 111320.0 * math.cos(math.radians(lat))
    t0 = time.time()
    print(f"== 注入开始 {datetime.now():%H:%M:%S}  测试区 {LAT0},{LON0} ==")
    # 控制台在线检查：OTA/imu_fetch 刚用过透传桥时桥可能还没空闲退出（桥内 USB 输入直通
    # S3R，控制台命令全被吞）——静默 65s 等它超时退出后再试，最多两轮
    for attempt in range(3):
        ser.reset_input_buffer()
        ser.write(b"s3rnav\n")
        end = time.time() + 3.0; ok = False
        while time.time() < end:
            l = ser.readline().decode(errors="replace")
            if "[S3R] nav" in l: ok = True; break
        if ok: break
        if attempt == 2: sys.exit("PowerHub 控制台无应答（仍在透传桥或未开机），放弃")
        print("   控制台无应答（疑似仍在透传桥），静默 65s 等桥退出…")
        time.sleep(65)
    cmd("s3rnav", 1.5)
    for name, dur, spd, valid, offN, moveN, repSpd, crs in phases:
        print(f"-- {name} {dur:.0f}s spd={spd} valid={valid} offN={offN} --")
        tp = time.time(); n = 0
        while time.time() - tp < dur:
            t = time.time()
            dt = 0.2
            lon += spd * dt / kx                      # 真实位置始终推进（含断档期）
            offN += moveN * dt
            # 量测噪声 ~1m（有定位时）
            nl = lat + offN / 111320.0 + (0.8 / 111320.0) * math.sin(n * 1.7)
            no = lon + (0.8 / kx) * math.cos(n * 2.3)
            rmc, gga = epoch_sentences(t, nl, no, valid, spd if repSpd is None else repSpd, crs)
            ser.write(f"gpsin $S3R,NMEA,{rmc}\n".encode())
            ser.write(f"gpsin $S3R,NMEA,{gga}\n".encode())
            n += 1
            pump(0.2 - (time.time() - t) if time.time() - t < 0.2 else 0.01)
        cmd("s3rnav", 1.5)
    print("== 注入结束 ==")
    pump(8)                                           # 等 PowerHub 侧"Fix lost"
    cmd("s3rnav", 1.5)
    if a.send:
        print("-- sendtest（强发当前点）--")
        cmd("sendtest", 40)
    ser.close()
    print("完整日志: inject_test.log")


if __name__ == "__main__":
    main()
