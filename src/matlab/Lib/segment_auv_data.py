#!/usr/bin/env python
"""
对 auv_data_merged.csv 进行数据段划分。
- 阶跃测试: 前进/后退(TX), 右移/左移(TY), 右转/左转(MZ)
- 推力等级: 2000(20%) / 4000(40%) / 6000(60%) / 8000(80%) / 10000(100%)
- MZ 仅在 TX=TY=0 时才视为旋转测试, 否则为航向控制
- 输出段索引 CSV，后续 MATLAB 按 idx 范围提取
"""

import csv
import numpy as np
import os

# ---------------------------------------------------------------------------
# 配置
# ---------------------------------------------------------------------------
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
CSV_IN = os.path.join(PROJECT_ROOT, "data", "csv", "auv_data_merged.csv")
CSV_OUT = os.path.join(PROJECT_ROOT, "data", "csv", "auv_segments.csv")

STEP_THRESHOLD = 1500       # 阶跃检测阈值 (>1500 确保是 2000+ 的测试指令)
CTRL_THRESHOLD = 500        # 控制量判定阈值 (MZ 航向控制 < 500)
MIN_SEGMENT_DURATION = 2.0  # 最小有效段时长（秒）
MIN_MZ_DURATION = 4.0       # MZ 旋转测试最小时长（秒），过滤航向修正
MERGE_GAP = 3.0             # 同幅值段合并间隔（秒）
STANDARD_AMPS = [2000, 4000, 6000, 8000, 10000]  # 仅标准测试等级 20/40/60/80/100%
VALID_MODES = [1, 2, 3]     # 有效控制模式 (02=定深, 03=定深定向, 排除04定点)

# ---------------------------------------------------------------------------
# 1. 加载数据
# ---------------------------------------------------------------------------
print("[1/4] 加载 CSV...")
times, tx, ty, tz, mz = [], [], [], [], []
depths, yaws, modes = [], [], []
linear_vx, linear_vy, angular_wz = [], [], []

with open(CSV_IN, "r", encoding="utf-8") as f:
    reader = csv.DictReader(f)
    for row in reader:
        times.append(float(row["stamp_time"]))
        tx.append(int(float(row["motor_TX"])))
        ty.append(int(float(row["motor_TY"])))
        tz.append(int(float(row["motor_TZ"])))
        mz.append(int(float(row["motor_MZ"])))
        depths.append(float(row["depth"]))
        yaws.append(float(row["yaw"]))
        modes.append(int(row["control_mode"]))
        linear_vx.append(float(row["linear_vx"]))
        linear_vy.append(float(row["linear_vy"]))
        angular_wz.append(float(row["angular_wz"]))

times = np.array(times)
tx = np.array(tx); ty = np.array(ty); tz = np.array(tz); mz = np.array(mz)
depths = np.array(depths); yaws = np.array(yaws); modes = np.array(modes)
linear_vx = np.array(linear_vx); linear_vy = np.array(linear_vy)
angular_wz = np.array(angular_wz)
N = len(times)

t0 = times[0]
print(f"  总行数: {N}, 时间跨度: {times[0]:.1f} → {times[-1]:.1f}")

# ---------------------------------------------------------------------------
# 2. 构建各通道的"测试活动"掩码
# ---------------------------------------------------------------------------
print("[2/4] 构建测试活动掩码...")

def cluster_to_standard_amp(values):
    """将信号值映射到最接近的标准幅值, 偏离超过 500 则归为 -1 (非标准)"""
    vals = np.array(values, dtype=float)
    result = np.full_like(vals, -1, dtype=int)
    for amp in STANDARD_AMPS:
        mask_pos = np.abs(vals - amp) <= 500
        result[mask_pos] = amp
        mask_neg = np.abs(vals + amp) <= 500
        result[mask_neg] = -amp
    return result

tx_clustered = cluster_to_standard_amp(tx)
ty_clustered = cluster_to_standard_amp(ty)
mz_clustered = cluster_to_standard_amp(mz)

# 检测各通道是否在"测试模式" (|cmd| >= 2000, 且为标准等级)
tx_testing_raw = np.abs(tx_clustered) >= 2000
ty_testing_raw = np.abs(ty_clustered) >= 2000
mz_testing_raw = np.abs(mz_clustered) >= 2000

# 过滤: 仅保留有效控制模式 (02=定深, 03=定深定向, 排除04定点)
valid_mask = np.isin(modes, VALID_MODES)

tx_testing = tx_testing_raw & valid_mask
ty_testing = ty_testing_raw & valid_mask
mz_testing = mz_testing_raw & valid_mask

print(f"  排除定点模式(mode 4)后:")
print(f"    TX 测试点: {np.sum(tx_testing)} (原 {np.sum(tx_testing_raw)})")
print(f"    TY 测试点: {np.sum(ty_testing)} (原 {np.sum(ty_testing_raw)})")
print(f"    MZ 测试点: {np.sum(mz_testing)} (原 {np.sum(mz_testing_raw)})")

# 解决 TX/TY 重叠: 同时激活时, 归给 |cmd| 更大的通道
overlap = tx_testing & ty_testing
if np.any(overlap):
    print(f"  发现 TX/TY 重叠点: {np.sum(overlap)} 个, 按较大指令分配...")
    tx_bigger = np.abs(tx_clustered) >= np.abs(ty_clustered)
    tx_testing[overlap] = tx_bigger[overlap]
    ty_testing[overlap] = ~tx_bigger[overlap]

# MZ 在 TX 或 TY 测试期间的活动 -> 航向控制, 不是旋转测试
mz_as_rotation = mz_testing & (~tx_testing) & (~ty_testing)

print(f"  TX 测试点:   {np.sum(tx_testing)} ({np.sum(tx_testing)/N*100:.1f}%)")
print(f"  TY 测试点:   {np.sum(ty_testing)} ({np.sum(ty_testing)/N*100:.1f}%)")
print(f"  MZ 测试点:   {np.sum(mz_testing)} ({np.sum(mz_testing)/N*100:.1f}%)")
print(f"  MZ 纯旋转:   {np.sum(mz_as_rotation)} ({np.sum(mz_as_rotation)/N*100:.1f}%)")

# ---------------------------------------------------------------------------
# 3. 找连续段
# ---------------------------------------------------------------------------
print("[3/4] 提取连续段...")

def extract_segments(testing_mask, clustered_signal, times, min_dur=MIN_SEGMENT_DURATION):
    """
    从 testing_mask 中找连续 True 段, 按幅值变化切分。
    返回 [(start_idx, end_idx, amplitude, direction)]。
    """
    if not np.any(testing_mask):
        return []

    # 找 testing_mask 的边界
    mask_int = testing_mask.astype(int)
    changes = np.diff(mask_int, prepend=0)
    starts = np.where(changes == 1)[0]
    ends = np.where(changes == -1)[0]

    if len(starts) == 0:
        return []
    if len(ends) < len(starts):
        ends = np.append(ends, N)

    raw_segs = []
    for s, e in zip(starts, ends):
        dur = times[min(e - 1, N - 1)] - times[s]
        if dur < min_dur:
            continue
        seg_clustered = clustered_signal[s:e]
        # 取中值作为段幅值
        median_amp = np.median(seg_clustered[seg_clustered != -1])
        if np.isnan(median_amp):
            continue
        amp = int(np.round(median_amp / 1000) * 1000)
        if amp == 0:
            continue
        direction = 1 if median_amp > 0 else -1
        raw_segs.append((int(s), int(e), direction, abs(amp)))

    return raw_segs

tx_segs = extract_segments(tx_testing, tx_clustered, times)
ty_segs = extract_segments(ty_testing, ty_clustered, times)
mz_segs = extract_segments(mz_as_rotation, mz_clustered, times, min_dur=MIN_MZ_DURATION)

# 合并相邻同幅值段
def merge_segments(segments, times, gap=MERGE_GAP):
    if len(segments) <= 1:
        return segments
    merged = []
    cur = list(segments[0])
    for seg in segments[1:]:
        s, e, d, a = seg
        gap_t = times[s] - times[cur[1] - 1]
        if d == cur[2] and a == cur[3] and gap_t < gap:
            cur[1] = e
        else:
            merged.append(tuple(cur))
            cur = list(seg)
    merged.append(tuple(cur))
    return merged

tx_segs = merge_segments(tx_segs, times)
ty_segs = merge_segments(ty_segs, times)
mz_segs = merge_segments(mz_segs, times)

print(f"  TX 段: {len(tx_segs)}")
print(f"  TY 段: {len(ty_segs)}")
print(f"  MZ 段: {len(mz_segs)}")

# ---------------------------------------------------------------------------
# 4. 标注 & 输出
# ---------------------------------------------------------------------------
print("[4/4] 标注并输出...")

def label_segment(s, e, channel, direction, amp):
    seg_t = times[s:e]
    seg_tx = tx[s:e]; seg_ty = ty[s:e]; seg_tz = tz[s:e]; seg_mz = mz[s:e]
    seg_depth = depths[s:e]; seg_yaw = yaws[s:e]
    seg_vx = linear_vx[s:e]; seg_vy = linear_vy[s:e]; seg_wz = angular_wz[s:e]

    # 方向名称
    if channel == "TX":
        test_name = "前进" if direction > 0 else "后退"
        motion_type = "直航"
        holding = "定深定向"
    elif channel == "TY":
        test_name = "右移" if direction > 0 else "左移"
        motion_type = "侧移"
        holding = "定深"
    elif channel == "MZ":
        test_name = "右转" if direction > 0 else "左转"
        motion_type = "旋转"
        holding = "定深"

    # 深度统计
    depth_mean = np.mean(seg_depth)
    depth_std = np.std(seg_depth)

    # 航向统计
    yaw_mean = np.mean(seg_yaw)
    yaw_std = np.std(seg_yaw)

    # 后 50% 段作为稳态
    half = len(seg_t) // 2
    steady_vx = np.mean(seg_vx[half:])
    steady_vy = np.mean(seg_vy[half:])
    steady_wz = np.mean(seg_wz[half:])

    # 总速度
    total_speed = np.mean(np.sqrt(seg_vx[half:]**2 + seg_vy[half:]**2))

    # 垂推活动 (深度控制)
    tz_active = np.any(np.abs(seg_tz) > 100)

    duration = seg_t[-1] - seg_t[0]

    return {
        "seg_id": 0,
        "idx_start": int(s),
        "idx_end": int(e - 1),
        "t_start": seg_t[0],
        "t_end": seg_t[-1],
        "duration": round(duration, 2),
        "channel": channel,
        "direction": direction,
        "direction_name": test_name,
        "motion_type": motion_type,
        "holding": holding,
        "amplitude": amp,
        "amp_pct": f"{amp // 100}%",
        "motor_median": int(np.median(seg_tx)) if channel == "TX"
                        else int(np.median(seg_ty)) if channel == "TY"
                        else int(np.median(seg_mz)),
        "depth_mean": round(depth_mean, 4),
        "depth_std": round(depth_std, 4),
        "yaw_mean": round(yaw_mean, 3),
        "yaw_std": round(yaw_std, 3),
        "steady_vx": round(steady_vx, 4),
        "steady_vy": round(steady_vy, 4),
        "steady_wz": round(steady_wz, 4),
        "total_speed": round(total_speed, 4),
    }

all_segs = []
for s, e, direction, amp in tx_segs:
    all_segs.append(label_segment(s, e, "TX", direction, amp))
for s, e, direction, amp in ty_segs:
    all_segs.append(label_segment(s, e, "TY", direction, amp))
for s, e, direction, amp in mz_segs:
    all_segs.append(label_segment(s, e, "MZ", direction, amp))

all_segs.sort(key=lambda x: x["t_start"])

# 写 CSV
fieldnames = [
    "seg_id", "idx_start", "idx_end", "t_start", "t_end", "duration",
    "channel", "direction", "direction_name", "motion_type", "holding",
    "amplitude", "amp_pct", "motor_median",
    "depth_mean", "depth_std", "yaw_mean", "yaw_std",
    "steady_vx", "steady_vy", "steady_wz", "total_speed",
]

os.makedirs(os.path.dirname(CSV_OUT), exist_ok=True)
with open(CSV_OUT, "w", newline="", encoding="utf-8") as f:
    writer = csv.DictWriter(f, fieldnames=fieldnames)
    writer.writeheader()
    for i, seg in enumerate(all_segs):
        seg["seg_id"] = i + 1
        writer.writerow(seg)

print(f"\n  输出段数: {len(all_segs)}")
print(f"  输出文件: {CSV_OUT}")

# 按通道统计
from collections import Counter
ch_counts = Counter(s["channel"] for s in all_segs)
for ch in ["TX", "TY", "MZ"]:
    amps = Counter((s["direction_name"], s["amp_pct"]) for s in all_segs if s["channel"] == ch)
    print(f"\n  {ch} ({ch_counts.get(ch, 0)} 段):")
    for (dname, pct), cnt in sorted(amps.items()):
        print(f"    {dname} {pct}: {cnt} 段")

# ---------------------------------------------------------------------------
# 5. 打印摘要
# ---------------------------------------------------------------------------
print(f"\n{'='*95}")
print(f"{'ID':>4s} {'通道':>4s} {'方向':>6s} {'幅值':>6s} {'约束':>8s} {'开始':>10s} {'时长':>7s} {'稳态V':>8s} {'深度':>8s} {'航向std':>7s}")
print(f"{'-'*95}")
for seg in all_segs:
    t_str = f"{seg['t_start']-t0:.0f}s"
    if seg['channel'] == 'TX':
        v_str = f"vx={seg['steady_vx']:.3f}"
    elif seg['channel'] == 'TY':
        v_str = f"vy={seg['steady_vy']:.3f}"
    else:
        v_str = f"wz={seg['steady_wz']:.1f}"
    print(f"{seg['seg_id']:4d} {seg['channel']:>4s} {seg['direction_name']:>6s} {seg['amp_pct']:>6s} {seg['holding']:>8s} {t_str:>10s} {seg['duration']:>6.1f}s {v_str:>8s} {seg['depth_mean']:>8.2f} {seg['yaw_std']:>7.2f}")

print(f"\n输出: {CSV_OUT}")
