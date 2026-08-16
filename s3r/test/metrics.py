#!/usr/bin/env python3
# metrics.py — 回放输出评分：v2(nav) vs 基线(base) vs 原始 GNSS
# 指标全部对着 0808 翻车症状与 0801 真实痛点设计,定版验收也用同一把尺子:
#   A 良好段跟踪偏差   —— 估计器不许"跑离"健康的 GNSS(v1 翻车主症状)
#   B 停留段散布       —— 停留漂移云是否收敛(做这个功能的初衷)
#   C 输出连续性       —— 相邻发点隐含速度无非物理跳变(v1 阶梯/飞点症状)
#   D 断档桥接         —— 无定位窗里诚实桥了多少(0801 的 22 断档痛点)
#   E v1 失败签名      —— still 信念高企时原始速度>3m/s 的拍数(必须≈0)
# 用法: python3 metrics.py nav.csv base.csv [--plot out.png]
import csv, math, sys

def dist_m(la1, lo1, la2, lo2):
    kx = 111320.0 * math.cos(math.radians(la1))
    return math.hypot((la2 - la1) * 111320.0, (lo2 - lo1) * kx)

def load(path):
    rows = []
    with open(path) as f:
        for r in csv.DictReader(f):
            rows.append({k: (v if k == "kind" else float(v)) for k, v in r.items()})
    return rows

def q(v, p):
    if not v: return float("nan")
    s = sorted(v)
    return s[min(int(len(s) * p), len(s) - 1)]

def stop_segments(rows, min_dur_ms=60000):
    """原始数据定义的停留段: raw_valid=1 且 raw_spd<0.5 连续超 min_dur"""
    segs, start, last = [], None, None
    for r in rows:
        ok = r["raw_valid"] == 1 and r["raw_spd"] < 0.5
        if ok:
            if start is None: start = r["ms"]
            last = r["ms"]
        else:
            if start is not None and last - start >= min_dur_ms:
                segs.append((start, last))
            start = None
    if start is not None and last - start >= min_dur_ms:
        segs.append((start, last))
    return segs

def analyze(rows, name):
    out = {"name": name}
    emit = [r for r in rows if r["emit"] == 1]
    # A 良好段跟踪偏差(raw 健康: valid, hdop<2, spd>3)
    devs = [dist_m(r["lat"], r["lon"], r["raw_lat"], r["raw_lon"])
            for r in emit if r["raw_valid"] == 1 and r["raw_hdop"] < 2 and r["raw_spd"] > 3]
    out["A_dev_med"], out["A_dev_p90"], out["A_dev_max"] = q(devs, .5), q(devs, .9), q(devs, 1)
    out["A_runaway150"] = sum(1 for d in devs if d > 150)
    # B 停留段散布(段内发点相对段中位点的 p95 半径)
    segs = stop_segments(rows)
    radii = []
    for (t0, t1) in segs:
        pts = [(r["lat"], r["lon"]) for r in emit if t0 <= r["ms"] <= t1]
        if len(pts) < 10: continue
        mla = sorted(p[0] for p in pts)[len(pts) // 2]
        mlo = sorted(p[1] for p in pts)[len(pts) // 2]
        radii.append(q([dist_m(la, lo, mla, mlo) for la, lo in pts], .95))
    out["B_stop_segs"] = len(radii)
    out["B_stop_r95"] = q(radii, .5) if radii else float("nan")
    # C 输出连续性: 相邻发点隐含速度>60m/s 且单步距离>50m 才算"地图可见的传送"
    # (0.2s 拍距上 十几米的 KF 收敛步会虚高成>60m/s,但地图上不可见,不计)
    jumps, vmax = 0, []
    for i in range(1, len(emit)):
        dt = (emit[i]["ms"] - emit[i-1]["ms"]) / 1000.0
        if not (0 < dt < 10): continue
        d = dist_m(emit[i-1]["lat"], emit[i-1]["lon"], emit[i]["lat"], emit[i]["lon"])
        v = d / max(dt, 0.2)
        vmax.append(v)
        if v > 60 and d > 50: jumps += 1
    out["C_v_p99"], out["C_jumps"] = q(vmax, .99), jumps
    # D 断档桥接: raw invalid 的拍里发了多少推算点/最长桥接时长
    gap_est = [r for r in rows if r["raw_valid"] == 0 and r["emit"] == 1 and r["est"] == 1]
    out["D_gap_epochs"] = sum(1 for r in rows if r["raw_valid"] == 0)
    out["D_bridged"] = len(gap_est)
    lg, cur, prev = 0, 0, None
    for r in rows:
        if r["raw_valid"] == 0 and r["emit"] == 1:
            cur = cur + (r["ms"] - prev if prev else 0)
            lg = max(lg, cur)
            prev = r["ms"]
        else:
            cur, prev = 0, None
    out["D_longest_bridge_s"] = lg / 1000.0
    out["D_sigma_max"] = q([r["sigma_m"] for r in gap_est], 1) if gap_est else 0
    # D2 桥接终点修正: 每段 coast 发点串结束后 3s 内首个实测点与最后推算点的距离
    # (= 桥错了多远;调 coastDecay/sigmaEmitMax 的直接依据,对应 v1 的 DR_OFF val)
    corr, last_coast = [], None
    for r in emit:
        if r["est"] == 1: last_coast = r
        elif last_coast is not None:
            if 0 < r["ms"] - last_coast["ms"] <= 3000:
                corr.append(dist_m(last_coast["lat"], last_coast["lon"], r["lat"], r["lon"]))
            last_coast = None
    out["D2_n"], out["D2_med"], out["D2_max"] = len(corr), q(corr, .5), q(corr, 1)
    # E v1 失败签名: still>0.8 且原始速度>3m/s。限 hdop<2 的健康拍——v1 翻车是户外
    # 好信号下把行进电车判静止;室内多径垃圾拍(0816 全天回放:店内 hdop≥2 的 260 拍
    # spd 虚报>3,信念反而是对的)不算失败签名,基线同样中招证明是 raw 现象。
    out["E_still_moving"] = sum(1 for r in rows
                                if r["still"] > 0.8 and r["raw_valid"] == 1
                                and r["raw_spd"] > 3 and r["raw_hdop"] < 2)
    return out

def main():
    nav = analyze(load(sys.argv[1]), "nav(v2)")
    base = analyze(load(sys.argv[2]), "base(PowerHub复刻)")
    keys = [("A_dev_med",  "A 良好段偏差中位(m)"), ("A_dev_p90", "A 良好段偏差p90(m)"),
            ("A_dev_max",  "A 良好段偏差max(m)"), ("A_runaway150", "A 偏差>150m拍数"),
            ("B_stop_segs","B 停留段数(≥60s)"),   ("B_stop_r95", "B 停留散布r95(m)"),
            ("C_v_p99",    "C 隐含速度p99(m/s)"), ("C_jumps",    "C 地图可见传送数"),
            ("D_gap_epochs","D 无定位拍总数"),    ("D_bridged",  "D 桥接发点数"),
            ("D_longest_bridge_s", "D 最长桥接(s)"), ("D_sigma_max", "D 桥接σmax(m)"),
            ("D2_n", "D2 桥接段数"), ("D2_med", "D2 终点修正中位(m)"), ("D2_max", "D2 终点修正max(m)"),
            ("E_still_moving", "E v1失败签名拍数")]
    print(f"{'指标':<28}{nav['name']:>16}{base['name']:>22}")
    for k, label in keys:
        def fmt(v):
            return f"{v:.1f}" if isinstance(v, float) and not math.isnan(v) else str(v)
        print(f"{label:<28}{fmt(nav[k]):>16}{fmt(base[k]):>22}")

    if "--plot" in sys.argv:
        png = sys.argv[sys.argv.index("--plot") + 1]
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        rows_n, rows_b = load(sys.argv[1]), load(sys.argv[2])
        fig, ax = plt.subplots(figsize=(11, 11))
        raw = [(r["raw_lon"], r["raw_lat"]) for r in rows_n if r["raw_valid"] == 1]
        ax.plot([p[0] for p in raw], [p[1] for p in raw], ".", ms=1.5, color="#999", label="raw GNSS")
        eb = [(r["lon"], r["lat"]) for r in rows_b if r["emit"] == 1]
        ax.plot([p[0] for p in eb], [p[1] for p in eb], "-", lw=0.7, color="#1f77b4", label="base(KF only)")
        en = [(r["lon"], r["lat"]) for r in rows_n if r["emit"] == 1 and r["est"] == 0]
        ax.plot([p[0] for p in en], [p[1] for p in en], "-", lw=0.9, color="#d62728", label="nav v2")
        ec = [(r["lon"], r["lat"]) for r in rows_n if r["emit"] == 1 and r["est"] == 1]
        ax.plot([p[0] for p in ec], [p[1] for p in ec], "x", ms=4, color="#ff7f0e", label="nav v2 coast(q=6)")
        ax.set_aspect(1.0 / math.cos(math.radians(raw[0][1] if raw else 35)))
        ax.legend(); ax.set_title("replay: raw vs base vs nav-v2")
        fig.savefig(png, dpi=140, bbox_inches="tight")
        print(f"[plot] {png}")

if __name__ == "__main__":
    main()
