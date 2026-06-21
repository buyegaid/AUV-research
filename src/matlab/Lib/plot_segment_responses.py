#!/usr/bin/env python
"""
按通道绘制各推力等级的阶跃响应时序图
- 每个通道一张大图: 正向/负向 各子图(速度+指令+功率)
- 汇总图: 稳态速度 vs 推力等级
"""

import csv
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator
import os

# ---------------------------------------------------------------------------
# 配置
# ---------------------------------------------------------------------------
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
DATA_DIR = os.path.join(PROJECT_ROOT, "data", "csv")
OUT_DIR = os.path.join(PROJECT_ROOT, "data", "figures")
os.makedirs(OUT_DIR, exist_ok=True)

# 中文字体
plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False

# ---------------------------------------------------------------------------
# 1. 加载数据
# ---------------------------------------------------------------------------
print("[1/3] 加载数据...")

# 主数据
with open(os.path.join(DATA_DIR, "auv_data_merged.csv"), "r", encoding="utf-8") as f:
    reader = csv.DictReader(f)
    raw = {c: [] for c in reader.fieldnames}
    for row in reader:
        for c in reader.fieldnames:
            raw[c].append(row[c])

t = np.array([float(v) for v in raw["stamp_time"]])
motor_TX = np.array([int(float(v)) for v in raw["motor_TX"]])
motor_TY = np.array([int(float(v)) for v in raw["motor_TY"]])
motor_TZ = np.array([int(float(v)) for v in raw["motor_TZ"]])
motor_MZ = np.array([int(float(v)) for v in raw["motor_MZ"]])
linear_vx = np.array([float(v) for v in raw["linear_vx"]])
linear_vy = np.array([float(v) for v in raw["linear_vy"]])
angular_wz = np.array([float(v) for v in raw["angular_wz"]])
prop_pwr = np.array([float(v) for v in raw["prop_pwr_power"]])
depth = np.array([float(v) for v in raw["depth"]])
yaw_arr = np.array([float(v) for v in raw["yaw"]])

# 段索引
with open(os.path.join(DATA_DIR, "auv_segments_final.csv"), "r", encoding="utf-8") as f:
    segs = list(csv.DictReader(f))

print(f"  数据点: {len(t)}, 段数: {len(segs)}")

# ---------------------------------------------------------------------------
# 2. 绘图配置
# ---------------------------------------------------------------------------
print("[2/3] 绘制通道响应图...")

channels = [
    {"name": "TX", "label": "前进/后退 (Surge)", "cmd": motor_TX, "vel": linear_vx,
     "vlabel": "v_x (m/s)", "pos_name": "前进", "neg_name": "后退"},
    {"name": "TY", "label": "右移/左移 (Sway)", "cmd": motor_TY, "vel": linear_vy,
     "vlabel": "v_y (m/s)", "pos_name": "右移", "neg_name": "左移"},
    {"name": "MZ", "label": "右转/左转 (Yaw)", "cmd": motor_MZ, "vel": angular_wz,
     "vlabel": "ω_z (°/s)", "pos_name": "右转", "neg_name": "左转"},
]

THRUST_LEVELS = [20, 40, 60, 80, 100]
COLORS = plt.cm.viridis(np.linspace(0.1, 0.9, len(THRUST_LEVELS)))
LEVEL_COLOR = dict(zip(THRUST_LEVELS, COLORS))

for ch in channels:
    ch_name = ch["name"]
    ch_segs = [s for s in segs if s["channel"] == ch_name]
    if not ch_segs:
        print(f"  {ch_name}: 无数据，跳过")
        continue

    pos_segs = [s for s in ch_segs if int(s["direction"]) == 1]
    neg_segs = [s for s in ch_segs if int(s["direction"]) == -1]

    fig, axes = plt.subplots(3, 2, figsize=(18, 12), sharex='col')
    fig.suptitle(f'{ch["label"]} — 阶跃响应时序', fontsize=14, fontweight='bold')

    # ---- 正向 ----
    for i, seg in enumerate(pos_segs):
        idx_s = int(seg["idx_pad_start"])
        idx_e = int(seg["idx_pad_end"])
        seg_t = t[idx_s:idx_e] - t[int(seg["idx_start"])]  # 相对时间
        seg_vel = ch["vel"][idx_s:idx_e]
        seg_cmd = ch["cmd"][idx_s:idx_e]
        seg_pwr = prop_pwr[idx_s:idx_e]
        level = int(float(seg["amplitude"])) // 100  # 20, 40, ...
        clr = LEVEL_COLOR.get(level, 'gray')

        axes[0, 0].plot(seg_t, seg_vel, color=clr, linewidth=1.2)
        axes[1, 0].plot(seg_t, seg_cmd, color=clr, linewidth=1.2)
        axes[2, 0].plot(seg_t, seg_pwr, color=clr, linewidth=1.0)

        # 标出段区间
        t_ss = t[int(seg["idx_start"])] - t[int(seg["idx_start"])]
        t_se = t[int(seg["idx_end"])] - t[int(seg["idx_start"])]
        for ax_row in [0, 1, 2]:
            axes[ax_row, 0].axvline(t_ss, color='k', linestyle='--', alpha=0.25, linewidth=0.8)
            axes[ax_row, 0].axvline(t_se, color='k', linestyle='--', alpha=0.25, linewidth=0.8)

    axes[0, 0].set_title(ch["pos_name"], fontsize=12)
    axes[0, 0].set_ylabel(ch["vlabel"])
    axes[0, 0].grid(True, alpha=0.3)
    axes[1, 0].set_ylabel('指令')
    axes[1, 0].grid(True, alpha=0.3)
    axes[2, 0].set_ylabel('动力功率 (W)')
    axes[2, 0].set_xlabel('时间 (s)')
    axes[2, 0].grid(True, alpha=0.3)

    # ---- 负向 ----
    for i, seg in enumerate(neg_segs):
        idx_s = int(seg["idx_pad_start"])
        idx_e = int(seg["idx_pad_end"])
        seg_t = t[idx_s:idx_e] - t[int(seg["idx_start"])]
        seg_vel = ch["vel"][idx_s:idx_e]
        seg_cmd = ch["cmd"][idx_s:idx_e]
        seg_pwr = prop_pwr[idx_s:idx_e]
        level = int(float(seg["amplitude"])) // 100
        clr = LEVEL_COLOR.get(level, 'gray')

        axes[0, 1].plot(seg_t, seg_vel, color=clr, linewidth=1.2)
        axes[1, 1].plot(seg_t, seg_cmd, color=clr, linewidth=1.2)
        axes[2, 1].plot(seg_t, seg_pwr, color=clr, linewidth=1.0)

        t_ss = t[int(seg["idx_start"])] - t[int(seg["idx_start"])]
        t_se = t[int(seg["idx_end"])] - t[int(seg["idx_start"])]
        for ax_row in [0, 1, 2]:
            axes[ax_row, 1].axvline(t_ss, color='k', linestyle='--', alpha=0.25, linewidth=0.8)
            axes[ax_row, 1].axvline(t_se, color='k', linestyle='--', alpha=0.25, linewidth=0.8)

    axes[0, 1].set_title(ch["neg_name"], fontsize=12)
    axes[0, 1].grid(True, alpha=0.3)
    axes[1, 1].grid(True, alpha=0.3)
    axes[2, 1].set_xlabel('时间 (s)')
    axes[2, 1].grid(True, alpha=0.3)

    # Y 轴对齐
    axes[0, 0].sharey(axes[0, 1])
    axes[1, 0].sharey(axes[1, 1])

    # 图例
    legend_labels = [f'{lv}%' for lv in THRUST_LEVELS]
    axes[0, 0].legend(legend_labels, loc='best', fontsize=8, ncol=5)

    plt.tight_layout()
    out_path = os.path.join(OUT_DIR, f'response_{ch_name}.png')
    fig.savefig(out_path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f"  已保存: response_{ch_name}.png")

# ---------------------------------------------------------------------------
# 3. 稳态汇总图
# ---------------------------------------------------------------------------
print("[3/3] 稳态汇总...")

fig, axes = plt.subplots(1, 3, figsize=(16, 5))
fig.suptitle('稳态速度 vs 推力等级', fontsize=14, fontweight='bold')

for ch_idx, ch in enumerate(channels):
    ax = axes[ch_idx]
    ch_segs = [s for s in segs if s["channel"] == ch["name"]]
    if not ch_segs:
        continue

    for d, marker, label_suffix in [(-1, 'v', '(-)'), (1, '^', '(+)')]:
        dir_segs = [s for s in ch_segs if int(s["direction"]) == d]
        if not dir_segs:
            continue

        levels = sorted(set(int(float(s["amplitude"])) // 100 for s in dir_segs))
        for lv in levels:
            lv_segs = [s for s in dir_segs if int(float(s["amplitude"])) // 100 == lv]
            if ch_idx == 0:
                vals = [float(s["steady_vx"]) for s in lv_segs]
            elif ch_idx == 1:
                vals = [float(s["steady_vy"]) for s in lv_segs]
            else:
                vals = [float(s["steady_wz"]) for s in lv_segs]

            # 标注功率有效性
            has_pwr = [float(s["pp_seg_mean"]) > 30 for s in lv_segs]
            for j, (v, hp) in enumerate(zip(vals, has_pwr)):
                edge_clr = 'green' if hp else 'gray'
                ax.scatter(lv, abs(v), marker=marker, s=80,
                          facecolors='none' if not hp else COLORS[THRUST_LEVELS.index(lv)],
                          edgecolors=edge_clr, linewidths=1.5, zorder=3)

    ax.set_xlabel('推力等级 (%)')
    ax.set_ylabel(ch["vlabel"])
    ax.set_title(ch["label"])
    ax.grid(True, alpha=0.3)
    ax.set_xticks(THRUST_LEVELS)

# 图例
from matplotlib.lines import Line2D
legend_elements = [
    Line2D([0], [0], marker='^', color='w', markerfacecolor='gray', markersize=10, label='正向'),
    Line2D([0], [0], marker='v', color='w', markerfacecolor='gray', markersize=10, label='负向'),
    Line2D([0], [0], marker='o', color='w', markerfacecolor='gray', markersize=10,
           markeredgecolor='green', markeredgewidth=1.5, label='功率有效'),
    Line2D([0], [0], marker='o', color='w', markerfacecolor='gray', markersize=10,
           markeredgecolor='gray', markeredgewidth=1.5, label='功率无效'),
]
fig.legend(handles=legend_elements, loc='lower center', ncol=4, fontsize=9)

plt.tight_layout(rect=[0, 0.06, 1, 1])
out_path = os.path.join(OUT_DIR, 'steady_state_summary.png')
fig.savefig(out_path, dpi=150, bbox_inches='tight')
plt.close(fig)
print(f"  已保存: steady_state_summary.png")

print(f"\n完成! 图片保存在: {OUT_DIR}")
