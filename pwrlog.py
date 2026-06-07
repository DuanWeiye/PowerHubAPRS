#!/usr/bin/env python3
"""M5Power power-log tool — pull the on-device RTC power log over USB serial.

The firmware buffers battery voltage/current samples in RTC memory (see PWRLOG_*
in firmware.ino) and exposes `log` / `logclear` commands on the USB console.
This script sends those commands, saves the dump as CSV, prints a draw summary,
and (if matplotlib is present) plots the discharge curve.

Usage:
  ./pwrlog.py                       # dump → CSV (+ PNG) + summary
  ./pwrlog.py --clear               # erase the on-device log (do this before a run)
  ./pwrlog.py --port /dev/ttyACM0 --mah 2200 --outdir ~/m5logs

Notes:
  * Sending a command opens the port for writing, which resets the board ONCE.
    The log lives in RTC memory and survives the reset, so the dump is complete.
  * iotService holds /dev/ttyUSB0 (the M5Paper), not ACM0 — no contention.
"""
import argparse, datetime, os, sys, time

try:
    import serial
except ImportError:
    sys.exit("pyserial missing — install with:  pip install pyserial")


def capture(port, cmd, done_markers, timeout, resend=3.0):
    """Open port, (re)send cmd until all done_markers seen or timeout. Returns text.

    Resending covers the case where the open reset the board and the command
    parser isn't live yet (it only runs after setup() finishes booting)."""
    try:
        p = serial.Serial(port, 115200, timeout=0.4)
    except serial.SerialException as e:
        sys.exit(f"cannot open {port}: {e}")
    out = b""
    last = 0.0
    deadline = time.time() + timeout
    try:
        while time.time() < deadline:
            d = p.read(2048)
            if d:
                out += d
                if all(m.encode() in out for m in done_markers):
                    break
            if time.time() - last > resend:
                p.write(cmd)
                p.flush()
                last = time.time()
    finally:
        p.close()
    return out.decode(errors="replace").replace("\r", "")


def parse_block(txt):
    """Extract the first complete CSV dump (idx,ts,... → [PWRLOG] end)."""
    lines = txt.splitlines()
    start = end = None
    for i, l in enumerate(lines):
        if start is None and l.startswith("idx,ts,"):
            start = i
        elif start is not None and l.startswith("[PWRLOG] end"):
            end = i
            break
    if start is None or end is None:
        return None
    rows = []
    for l in lines[start + 1:end]:
        parts = l.split(",")
        if len(parts) == 9 and parts[0].isdigit():
            rows.append([int(x) for x in parts])
    return rows


def main():
    ap = argparse.ArgumentParser(description="Pull M5Power RTC power log.")
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--clear", action="store_true", help="erase the on-device log")
    ap.add_argument("--outdir", default=".")
    ap.add_argument("--mah", type=float, default=2200.0,
                    help="battery capacity for the endurance estimate (NP-F550 ≈ 2200)")
    ap.add_argument("--timeout", type=float, default=120.0,
                    help="seconds to wait (allow for a boot after the open-reset)")
    args = ap.parse_args()

    if args.clear:
        txt = capture(args.port, b"logclear\r\n", ["[PWRLOG] cleared"], args.timeout)
        print("cleared." if "[PWRLOG] cleared" in txt
              else "clear NOT confirmed — try again (device may still be booting).")
        return

    txt = capture(args.port, b"log\r\n", ["idx,ts,", "[PWRLOG] end"], args.timeout)
    rows = parse_block(txt)
    if rows is None:
        sys.stderr.write(txt[-600:] + "\n")
        sys.exit("ERROR: no complete dump captured — is the device connected & running?")
    if not rows:
        print("Log is empty (no samples buffered yet).")
        return

    os.makedirs(os.path.expanduser(args.outdir), exist_ok=True)
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = os.path.join(os.path.expanduser(args.outdir), f"pwrlog_{stamp}.csv")
    with open(csv_path, "w") as f:
        f.write("idx,epoch,iso_utc,mv,ma,pct,ext,chg,fix,catm\n")
        for r in rows:
            iso = (datetime.datetime.fromtimestamp(r[1], datetime.timezone.utc)
                   .strftime("%Y-%m-%dT%H:%M:%SZ") if r[1] > 0 else "")
            f.write(f"{r[0]},{r[1]},{iso},{r[2]},{r[3]},{r[4]},{r[5]},{r[6]},{r[7]},{r[8]}\n")

    n = len(rows)
    span = rows[-1][1] - rows[0][1] if rows[0][1] and rows[-1][1] else 0
    print(f"records : {n}")
    print(f"saved   : {csv_path}")
    if span:
        print(f"span    : {span/3600:.2f} h")
    print(f"voltage : {rows[0][2]} -> {rows[-1][2]} mV   ({rows[0][4]}% -> {rows[-1][4]}%)")

    batt = [r for r in rows if r[5] == 0]          # ext==0 → battery-only
    if batt:
        mas = [r[3] for r in batt]                 # + = draw, - = charging
        avg = sum(mas) / len(mas)
        print(f"battery : {len(batt)} samples, avg draw {avg:.0f} mA "
              f"(min {min(mas)}, max {max(mas)})")
        if avg > 0:
            print(f"endurance≈ {args.mah/avg:.1f} h on {args.mah:.0f} mAh (at this draw)")
        zeros = [r for r in batt if abs(r[3]) < 5]
        if zeros:
            print(f"NOTE    : {len(zeros)} battery samples near 0 mA — possible sleep/shutdown")
    else:
        print("battery : none (was on external power the whole time — run on battery to measure draw)")

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        x = [(r[1] - rows[0][1]) / 60.0 for r in rows]
        fig, ax1 = plt.subplots(figsize=(11, 4))
        ax1.plot(x, [r[2] for r in rows], "b-")
        ax1.set_xlabel("minutes"); ax1.set_ylabel("battery mV", color="b")
        ax2 = ax1.twinx()
        ax2.plot(x, [r[3] for r in rows], "r-", alpha=0.55)
        ax2.set_ylabel("mA  (− charge / + draw)", color="r")
        ax2.axhline(0, color="grey", lw=0.5)
        plt.title(f"M5Power battery log ({n} pts, {span/3600:.1f} h)")
        png = os.path.join(os.path.expanduser(args.outdir), f"pwrlog_{stamp}.png")
        fig.tight_layout(); fig.savefig(png, dpi=110)
        print(f"plot    : {png}")
    except ImportError:
        print("plot    : skipped (no matplotlib; CSV saved)")


if __name__ == "__main__":
    main()
