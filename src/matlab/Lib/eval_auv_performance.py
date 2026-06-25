#!/usr/bin/env python
"""AUV 性能评估: 基于0616水池实验数据"""
import csv, numpy as np, os

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
DATA_DIR = os.path.join(PROJECT_ROOT, "data", "csv")

# 加载
with open(os.path.join(DATA_DIR, "auv_data_merged.csv"), "r", encoding="utf-8") as f:
    reader = csv.DictReader(f)
    raw = {c: [] for c in reader.fieldnames}
    for row in reader:
        for c in reader.fieldnames:
            raw[c].append(row[c])

t_all = np.array([float(v) for v in raw["stamp_time"]])
motor_TX = np.array([int(float(v)) for v in raw["motor_TX"]])
motor_TY = np.array([int(float(v)) for v in raw["motor_TY"]])
motor_MZ = np.array([int(float(v)) for v in raw["motor_MZ"]])
lvx = np.array([float(v) for v in raw["linear_vx"]])
lvy = np.array([float(v) for v in raw["linear_vy"]])
awz = np.array([float(v) for v in raw["angular_wz"]])
prop_pwr = np.array([float(v) for v in raw["prop_pwr_power"]])
ctrl_pwr = np.array([float(v) for v in raw["ctrl_pwr_power"]])
depth = np.array([float(v) for v in raw["depth"]])
yaw_all = np.array([float(v) for v in raw["yaw"]])

with open(os.path.join(DATA_DIR, "auv_segments_final.csv"), "r", encoding="utf-8") as f:
    segs = list(csv.DictReader(f))

print("=" * 70)
print("AUV 性能评估 (0616 水池实验)")
print("=" * 70)

# === 1. 最大速度 ===
print("\n--- 1. 实测速度范围 ---")
for ch, v_arr, unit in [("TX", lvx, "m/s"), ("TY", lvy, "m/s"), ("MZ", awz, "°/s")]:
    for dn in sorted(set(s["direction_name"] for s in segs if s["channel"] == ch)):
        ch_segs = [s for s in segs if s["channel"] == ch and s["direction_name"] == dn]
        if not ch_segs:
            continue
        v_list = []
        for s in ch_segs:
            idx_s, idx_e = int(s["idx_start"]), int(s["idx_end"])
            n3 = max((idx_e - idx_s) // 3, 5)
            v_ss = np.mean(np.abs(v_arr[idx_s + 2 * n3 : idx_e + 1]))
            v_list.append((s["amp_pct"], v_ss, float(s["pp_seg_mean"])))
        if v_list:
            v_max = max(v[1] for v in v_list)
            v_80 = [v for v in v_list if v[0] == "80%"]
            v80_str = f", 80%={v_80[0][1]:.3f}{unit}" if v_80 else ""
            print(f"  {ch} {dn}: 峰值 {v_max:.3f}{unit}{v80_str}")

# === 2. 速域-功率 ===
print("\n--- 2. 速域-功率关系 (仅功率有效段) ---")
for ch_name, v_arr, unit, label in [("Surge", lvx, "m/s", "TX"), ("Sway", lvy, "m/s", "TY")]:
    valid_pts = []
    for s in segs:
        if s["channel"] != ("TX" if ch_name == "Surge" else "TY"):
            continue
        pp_m = float(s["pp_seg_mean"])
        if pp_m < 30:
            continue
        idx_s, idx_e = int(s["idx_start"]), int(s["idx_end"])
        n3 = max((idx_e - idx_s) // 3, 5)
        v_ss = np.mean(np.abs(v_arr[idx_s + 2 * n3 : idx_e + 1]))
        valid_pts.append({"amp": s["amp_pct"], "v": v_ss, "P": pp_m, "dn": s["direction_name"]})
    if valid_pts:
        valid_pts.sort(key=lambda x: x["v"])
        print(f"  {ch_name}:")
        for pt in valid_pts:
            net_P = pt["P"] - 29  # 减去待机
            cost = net_P / pt["v"] if pt["v"] > 0.01 else 0
            print(f"    {pt['amp']:>5s} {pt['dn']}: v={pt['v']:.3f} m/s, P_total={pt['P']:.0f}W, 推进P≈{net_P:.0f}W, 等效阻力={cost:.1f}N")

# === 3. 加速性能 ===
print("\n--- 3. 加速性能 (10%->90% 稳态上升时间) ---")
for ch_name, v_arr, v_scale in [("TX", lvx, 1.0), ("TY", lvy, 1.0)]:
    ch_segs = [s for s in segs if s["channel"] == ch_name and s["amp_pct"] != "100%"]
    rise_times = []
    for s in ch_segs:
        idx_s, idx_e = int(s["idx_start"]), int(s["idx_end"])
        v_seg = v_arr[idx_s : idx_e + 1] * v_scale
        t_rel = t_all[idx_s : idx_e + 1] - t_all[idx_s]
        v0 = np.mean(v_seg[:5])
        v_ss = np.mean(v_seg[-max(len(v_seg) // 3, 5) :])
        if abs(v_ss - v0) < 0.01:
            continue
        v10 = v0 + 0.1 * (v_ss - v0)
        v90 = v0 + 0.9 * (v_ss - v0)
        i10 = np.where(np.abs(v_seg) >= np.abs(v10))[0]
        i90 = np.where(np.abs(v_seg) >= np.abs(v90))[0]
        if len(i10) > 0 and len(i90) > 0:
            t_rise = t_rel[i90[0]] - t_rel[i10[0]]
            if 0.1 < t_rise < 60:
                rise_times.append((s["amp_pct"], t_rise))
    if rise_times:
        rt = [r[1] for r in rise_times]
        print(f"  {ch_name}: Tr(10-90%) = {np.mean(rt):.1f}s, [{np.min(rt):.1f}, {np.max(rt):.1f}]s, n={len(rt)}")

# === 4. 定深精度 ===
print("\n--- 4. 定深精度 ---")
for ch_name in ["TX", "TY", "MZ"]:
    ch_segs = [s for s in segs if s["channel"] == ch_name]
    if not ch_segs:
        continue
    d_stds = [float(s["depth_std"]) for s in ch_segs]
    print(f"  {ch_name}: depth std = {np.mean(d_stds):.3f}m [{np.min(d_stds):.3f}, {np.max(d_stds):.3f}]m")

# === 5. 航向保持 (直航段) ===
print("\n--- 5. 航向保持精度 (TX直航) ---")
tx_segs = [s for s in segs if s["channel"] == "TX"]
y_valid = [float(s["yaw_std"]) for s in tx_segs if float(s["yaw_std"]) < 180]
y_bad = [float(s["yaw_std"]) for s in tx_segs if float(s["yaw_std"]) >= 180]
print(f"  有效段 ({len(y_valid)}): yaw std = {np.mean(y_valid):.1f}° [{np.min(y_valid):.1f}, {np.max(y_valid):.1f}]°")
if y_bad:
    print(f"  异常段 ({len(y_bad)}): 偏航角跨±180°跳变, yaw std > 180°")

# === 6. 系统功耗 ===
print("\n--- 6. 系统功耗 ---")
idle = (np.abs(motor_TX) < 100) & (np.abs(motor_TY) < 100) & (np.abs(motor_MZ) < 100)
p_idle = prop_pwr[idle]
c_idle = ctrl_pwr[idle]
print(f"  待机: 动力={np.median(p_idle):.1f}W, 控制={np.median(c_idle):.1f}W, 总={np.median(p_idle + c_idle):.1f}W")

for label, m1, m2 in [
    ("TX 前进 100%", (np.abs(motor_TX) >= 9000) & (motor_TX > 0), None),
    ("TX 后退 100%", (np.abs(motor_TX) >= 9000) & (motor_TX < 0), None),
    ("TY 侧移 80%", (np.abs(motor_TY) >= 7500), None),
]:
    mask = m1
    ok = (prop_pwr > 30)
    m_final = mask & ok
    if np.sum(m_final) > 5:
        p_m = np.mean(prop_pwr[m_final])
        c_m = np.mean(ctrl_pwr[m_final])
        print(f"  {label}: 动力={p_m:.0f}W, 控制={c_m:.0f}W, 总={p_m + c_m:.0f}W")

# === 7. 转弯半径 ===
print("\n--- 7. 转弯性能 ---")
mz_segs = [s for s in segs if s["channel"] == "MZ" and s["direction_name"] == "右转"]
for s in mz_segs[:4]:
    idx_s, idx_e = int(s["idx_start"]), int(s["idx_end"])
    n3 = max((idx_e - idx_s) // 3, 5)
    wz_ss = np.mean(np.abs(awz[idx_s + 2 * n3 : idx_e + 1]))
    vx_m = np.mean(np.abs(lvx[idx_s + 2 * n3 : idx_e + 1]))
    if wz_ss > 0.5 and vx_m > 0.02:
        R = vx_m / (wz_ss * np.pi / 180)
        print(f"  {s['amp_pct']}: wz={wz_ss:.1f}°/s, vx={vx_m:.3f}m/s, R≈{R:.1f}m")

# === 8. 续航估算 ===
print("\n--- 8. 续航估算 ---")
# 电池: 假设 24V 20Ah = 480Wh
BATTERY_WH = 480
for label, p_watts, v_ms in [
    ("待机/低速巡游", 35, 0.25),
    ("中速巡游 (40%)", 35, 0.55),
    ("高速巡游 (80%)", 400, 0.77),
    ("全速 (100%, 饱和)", 500, 0.85),
]:
    hours = BATTERY_WH / p_watts if p_watts > 0 else float("inf")
    dist_km = v_ms * hours * 3.6
    print(f"  {label}: P={p_watts}W, 续航={hours:.1f}h, 航程={dist_km:.1f}km")

print(f"\n  电池容量假设: {BATTERY_WH}Wh (24V 20Ah), 实际可用 ~80%")

print("\n" + "=" * 70)
