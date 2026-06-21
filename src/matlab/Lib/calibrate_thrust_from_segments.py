#!/usr/bin/env python
"""
推进器标定: CFD阻力 → 反推 k 和 KT
- 使用 CFD 阻力/附加质量参数 (xhy_drag_cfd.m, xhy.m)
- 仅标定推进器系数: k (链路2), KT (链路1)
- 输入: 0616 阶跃段 + 0601 水池最大速度参考
"""

import csv, numpy as np, os, json

# ---------------------------------------------------------------------------
# 配置
# ---------------------------------------------------------------------------
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
DATA_DIR = os.path.join(PROJECT_ROOT, "data", "csv")
OUT_DIR = os.path.join(PROJECT_ROOT, "data", "calibration")
os.makedirs(OUT_DIR, exist_ok=True)

RHO = 1000.0; MAIN_D = 0.10; AUX_D = 0.06
N_MAX = 2500; CAN_FS = 9500  # 固件满量程 CAN-g
X_SF = 0.424; X_SR = -0.376

# CFD 阻力系数 (xhy_drag_cfd.m, 300W网格)
CFD = {
    'surge':  {'d1': 0.65,  'd2': 10.85},   # Fx = d1*u + d2*u*|u|
    'sway':   {'d1': 1.67,  'd2': 178.59},  # Fy
    'yaw_m':  {'d1': 0.2046, 'd2': 18.200}, # Nz = d1*r + d2*r*|r| [N·m]
}

# 0601 水池参考数据
JUNE1 = {
    'surge_fwd':  {'v_max': 0.675, 'cmd': 9500},   # 前进 max
    'sway_right': {'v_max': 0.330, 'cmd': 9500},   # 右移 max
    'sway_left':  {'v_max': 0.200, 'cmd': 9500},   # 左移 max
}

# ---------------------------------------------------------------------------
# 1. 加载
# ---------------------------------------------------------------------------
print("[1/3] 加载数据...")
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
awz = np.array([float(v) for v in raw["angular_wz"]])  # deg/s
prop_pwr = np.array([float(v) for v in raw["prop_pwr_power"]])

with open(os.path.join(DATA_DIR, "auv_segments_final.csv"), "r", encoding="utf-8") as f:
    segs = list(csv.DictReader(f))

# ---------------------------------------------------------------------------
# 2. 标定计算
# ---------------------------------------------------------------------------
print("[2/3] CFD阻力反推 k 和 KT...")

def calibrate_from_segments(channel, vel_array, cmd_array, drag_cfg, m_eff,
                            diam, arm_eff=None, k_can_ratio=1.0, vel_scale=1.0):
    """
    对指定通道的所有段进行标定
    - drag_cfg: {'d1':..., 'd2':...}
    - k_can_ratio: CAN-g 到单个推进器的比例 (TX→1.0, TY→0.5, MZ→0.83)
    - arm_eff: 力臂 (仅MZ)
    - vel_scale: 速度缩放 (MZ: deg/s→rad/s)
    """
    results = []
    ch_segs = [s for s in segs if s['channel'] == channel
               and s['amp_pct'] != '100%']  # 100%饱和，排除

    for seg in ch_segs:
        idx_s = int(seg['idx_start']); idx_e = int(seg['idx_end'])
        if idx_e <= idx_s + 5: continue

        # 后 1/3 稳态
        n3 = max((idx_e - idx_s) // 3, 5)
        v_seg = vel_array[idx_s + 2*n3 : idx_e + 1] * vel_scale
        cmd_seg = cmd_array[idx_s + 2*n3 : idx_e + 1]
        pp_seg = prop_pwr[idx_s + 2*n3 : idx_e + 1]

        v_ss = np.mean(v_seg)
        cmd_m = np.median(np.abs(cmd_seg))
        pp_m = np.mean(pp_seg)

        if cmd_m < 100 or abs(v_ss) < 1e-4:
            continue

        # CFD 阻力/力矩
        d1, d2 = drag_cfg['d1'], drag_cfg['d2']
        v_abs = abs(v_ss)
        F_drag = d1 * v_abs + d2 * v_abs**2

        # 链路2: k = F_drag / cmd
        k = F_drag / cmd_m

        # 链路1: KT
        n_can_per = k_can_ratio * cmd_m  # 单个推进器 CAN-g
        n_rpm = min(n_can_per / CAN_FS * N_MAX, N_MAX)
        n_rps = n_rpm / 60.0

        if arm_eff is not None:
            # MZ: 力矩 → 单个推进器推力
            T_per = F_drag / (2 * arm_eff)
        elif channel == 'TY':
            T_per = F_drag / 2.0  # 两桨分担
        else:
            T_per = F_drag  # TX: 单桨

        KT = T_per / (RHO * diam**4 * n_rps**2) if n_rps > 0 else 0

        # 功率效率
        eff = F_drag * v_abs / pp_m if pp_m > 20 else 0

        results.append({
            'channel': channel, 'seg_id': seg['seg_id'],
            'direction_name': seg['direction_name'],
            'direction': int(seg['direction']),
            'amp_pct': seg['amp_pct'],
            'cmd_median': round(cmd_m),
            'v_ss': round(v_abs, 4),
            'F_drag': round(F_drag, 4),
            'k': round(k, 6),
            'KT': round(KT, 4),
            'T_per_thruster': round(T_per, 4),
            'n_rpm': round(n_rpm, 1),
            'n_can_per': round(n_can_per),
            'pwr_mean': round(pp_m, 1),
            'pwr_eff': round(eff, 4),
            'tau_est': round(m_eff / d1 if d1 > 0 else 0, 2),
        })
    return results

# Surge (TX): T5 主推, K[5,TX]=1.0
tx_res = calibrate_from_segments('TX', lvx, motor_TX, CFD['surge'],
                                 m_eff=42.0, diam=MAIN_D, k_can_ratio=1.0)

# Sway (TY): T3+T4 侧推同向, each K=-0.5
ty_res = calibrate_from_segments('TY', lvy, motor_TY, CFD['sway'],
                                 m_eff=55.0, diam=AUX_D, k_can_ratio=0.5)

# Yaw (MZ): T3/T4 差动, each |K|=0.83, 力臂 0.4m
arm_yaw = (abs(X_SF) + abs(X_SR)) / 2.0  # 0.4m
mz_res = calibrate_from_segments('MZ', awz, motor_MZ, CFD['yaw_m'],
                                 m_eff=12.0, diam=AUX_D, k_can_ratio=0.83,
                                 arm_eff=arm_yaw, vel_scale=np.pi/180.0)

all_res = tx_res + ty_res + mz_res

# 0601 参考
june1_refs = []
for key, cfg in JUNE1.items():
    ch = 'TX' if 'surge' in key else 'TY'
    if 'surge' in key:
        d1, d2 = CFD['surge']['d1'], CFD['surge']['d2']
        diam = MAIN_D; k_ratio = 1.0; arm = None; label = '前进' if 'fwd' in key else '后退'
    else:
        d1, d2 = CFD['sway']['d1'], CFD['sway']['d2']
        diam = AUX_D; k_ratio = 0.5; arm = None
        label = '右移' if 'right' in key else '左移'

    v = cfg['v_max']; cmd = cfg['cmd']
    F = d1*v + d2*v**2
    k = F / cmd
    n_rpm = min(k_ratio * cmd / CAN_FS * N_MAX, N_MAX)
    n_rps = n_rpm / 60.0
    T_per = F if ch == 'TX' else F/2.0
    KT = T_per / (RHO * diam**4 * n_rps**2) if n_rps > 0 else 0
    june1_refs.append({
        'channel': ch, 'source': '0601', 'direction_name': label,
        'amp_pct': '100%', 'cmd_median': cmd, 'v_ss': v, 'F_drag': round(F,4),
        'k': round(k,6), 'KT': round(KT,4), 'T_per_thruster': round(T_per,4),
        'n_rpm': round(n_rpm,1),
    })

# ---------------------------------------------------------------------------
# 3. 汇总输出
# ---------------------------------------------------------------------------
print("[3/3] 输出...")

# 按通道+方向分组统计
from collections import defaultdict
groups = defaultdict(list)
for r in all_res:
    groups[f"{r['channel']}_{r['direction_name']}"].append(r)

print(f"\n{'='*70}")
print("推进器标定结果 (CFD阻力反推)")
print(f"{'='*70}")

final_cal = {}
for grp_key in sorted(groups.keys()):
    rs = groups[grp_key]
    ks = [r['k'] for r in rs]
    KTs = [r['KT'] for r in rs]

    # 找对应0601参考
    j1 = [j for j in june1_refs if f"{j['channel']}_{j['direction_name']}" == grp_key]

    print(f"\n--- {grp_key} ({len(rs)} 段) ---")
    for r in rs:
        print(f"  {r['amp_pct']:>5s}: v_ss={r['v_ss']:.3f}, F={r['F_drag']:.2f}N, "
              f"k={r['k']:.6f}, KT={r['KT']:.4f}, RPM={r['n_rpm']:.0f}, "
              f"τ_cfd={r['tau_est']:.1f}s, P={r['pwr_mean']:.0f}W, eff={r['pwr_eff']:.3f}")

    print(f"  k 均值: {np.mean(ks):.6f} ± {np.std(ks):.6f}")
    print(f"  KT 均值: {np.mean(KTs):.4f} ± {np.std(KTs):.4f}")

    if j1:
        print(f"  0601 参考: k={j1[0]['k']:.6f}, KT={j1[0]['KT']:.4f}, v={j1[0]['v_ss']}m/s")

    ch, dn = grp_key.split('_', 1)
    final_cal[grp_key] = {
        'k_mean': round(np.mean(ks), 6), 'k_std': round(np.std(ks), 6),
        'KT_mean': round(np.mean(KTs), 4), 'KT_std': round(np.std(KTs), 4),
        'v_ss_range': f"{min(r['v_ss'] for r in rs):.3f}~{max(r['v_ss'] for r in rs):.3f}",
    }

# 0601 参考单独打印
if june1_refs:
    print(f"\n{'='*70}")
    print("0601 水池参考 (最大速度映射)")
    print(f"{'='*70}")
    for j in june1_refs:
        print(f"  {j['channel']} {j['direction_name']}: k={j['k']:.6f}, KT={j['KT']:.4f}, v={j['v_ss']}m/s")

# 保存 CSV
all_out = all_res + june1_refs
keys = ['channel','source','seg_id','direction_name','direction','amp_pct',
        'cmd_median','v_ss','F_drag','k','KT','T_per_thruster','n_rpm',
        'n_can_per','pwr_mean','pwr_eff','tau_est']
with open(os.path.join(OUT_DIR, "thrust_calibration_final.csv"), "w", newline="", encoding="utf-8") as f:
    writer = csv.DictWriter(f, fieldnames=keys, extrasaction='ignore')
    writer.writeheader()
    for r in all_out:
        r.setdefault('source', '0616')
        writer.writerow(r)

with open(os.path.join(OUT_DIR, "thrust_calibration_final.json"), "w", encoding="utf-8") as f:
    json.dump({"segments": all_out, "summary": final_cal, "june1_ref": june1_refs},
              f, indent=2, ensure_ascii=False)

print(f"\n输出: {OUT_DIR}")
