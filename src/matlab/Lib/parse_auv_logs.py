#!/usr/bin/env python
"""
解析 data/logs/ 目录下的 auv_data_*.jsonl 文件，合并为 CSV。
- 以 /debug_auv_data (AUVData) 为时间主轴
- /nav (NavData) 和 /sensor_status (SensorStatus) 按最近时间戳插值对齐
- power1 = 控制电源, power2 = 动力电源
"""

import json
import glob
import os
import sys
import csv
import numpy as np

# ---------------------------------------------------------------------------
# 路径配置
# ---------------------------------------------------------------------------
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
LOG_DIR = os.path.join(PROJECT_ROOT, "data", "logs")
OUT_DIR = os.path.join(PROJECT_ROOT, "data", "csv")
OUT_FILE = os.path.join(OUT_DIR, "auv_data_merged.csv")

# ---------------------------------------------------------------------------
# 1. 读取所有 auv_data_*.jsonl 并按 topic 分类
# ---------------------------------------------------------------------------
def load_all_logs(log_dir):
    """读取所有 auv_data JSONL 文件，按 topic 分桶"""
    nav_records = []       # /nav
    debug_records = []     # /debug_auv_data
    sensor_records = []    # /sensor_status

    # 只保留 6月16日 的有效数据段（排除 6月15日测试 + 短数据段）
    VALID_PREFIXES = [
        "auv_data_20260616_113859",   # 主要测试段 1
        "auv_data_20260616_123235",   # 测试段 2
        "auv_data_20260616_132234",   # 主要测试段 3
    ]

    pattern = os.path.join(log_dir, "auv_data_*.jsonl")
    files = sorted(glob.glob(pattern))
    print(f"找到 {len(files)} 个 auv_data 文件")

    for fpath in files:
        fname = os.path.basename(fpath)
        # 过滤：仅保留白名单中的文件
        if not any(fname.startswith(p) for p in VALID_PREFIXES):
            print(f"  跳过（非 6/16 有效数据段）: {fname}")
            continue
        fsize = os.path.getsize(fpath)
        if fsize == 0:
            print(f"  跳过空文件: {fname}")
            continue
        print(f"  读取: {fname} ({fsize / 1e6:.1f} MB)")
        with open(fpath, "r", encoding="utf-8") as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                try:
                    obj = json.loads(line)
                except json.JSONDecodeError:
                    continue
                topic = obj.get("topic", "")
                if topic == "/nav":
                    nav_records.append(obj)
                elif topic == "/debug_auv_data":
                    debug_records.append(obj)
                elif topic == "/sensor_status":
                    sensor_records.append(obj)

    return nav_records, debug_records, sensor_records

# ---------------------------------------------------------------------------
# 2. 时间对齐：以 debug 时间轴为主，找最近的 nav / sensor
# ---------------------------------------------------------------------------
def align_by_nearest(primary, secondary, primary_time_fn, secondary_time_fn):
    """
    对 primary 的每条记录，从 secondary 中找到时间戳最接近的记录。
    返回 list of (secondary_record or None)。
    """
    secondary_times = np.array([secondary_time_fn(r) for r in secondary])
    aligned = []
    for p in primary:
        t = primary_time_fn(p)
        idx = np.argmin(np.abs(secondary_times - t))
        aligned.append(secondary[idx])
    return aligned

# ---------------------------------------------------------------------------
# 3. 提取各 topic 数据
# ---------------------------------------------------------------------------
def extract_nav(rec):
    """从 NavData 记录中提取字段字典"""
    d = rec.get("data", {})
    return {
        "nav_latitude":     d.get("latitude"),
        "nav_longitude":    d.get("longitude"),
        "nav_altitude":     d.get("altitude"),
        "nav_heave":        d.get("heave"),
        "nav_vn":           d.get("vn"),
        "nav_ve":           d.get("ve"),
        "nav_vd":           d.get("vd"),
        "nav_roll":         d.get("roll"),
        "nav_pitch":        d.get("pitch"),
        "nav_heading":      d.get("heading"),
        "nav_depth":        d.get("depth"),
        "nav_ins_status":   d.get("ins_status"),
        "nav_gps_lat":      d.get("gps_latitude"),
        "nav_gps_lon":      d.get("gps_longitude"),
        "nav_gps_alt":      d.get("gps_altitude"),
        "nav_gps_vel":      d.get("gps_vel"),
        "nav_gps_heading":  d.get("gps_heading"),
        "nav_gps_status":   d.get("gps_status"),
        "nav_dvl_vx":       d.get("dvl_vx"),
        "nav_dvl_vy":       d.get("dvl_vy"),
        "nav_dvl_vz":       d.get("dvl_vz"),
        "nav_dvl_altitude": d.get("dvl_altitude"),
        "nav_dvl_status":   d.get("dvl_status"),
        "nav_gyro_x":       d.get("gyro_x"),
        "nav_gyro_y":       d.get("gyro_y"),
        "nav_gyro_z":       d.get("gyro_z"),
        "nav_accel_x":      d.get("accel_x"),
        "nav_accel_y":      d.get("accel_y"),
        "nav_accel_z":      d.get("accel_z"),
        "nav_imu_temp":     d.get("temperature"),
        "nav_imu_status":   d.get("imu_status"),
        "nav_counter":      d.get("counter"),
    }

def extract_debug(rec):
    """从 AUVData 记录中提取字段字典"""
    d = rec.get("data", {})
    pose = d.get("pose", {})
    target = d.get("target", {})
    t = d.get("time", {})
    mf = d.get("motor_force", {})
    lv = d.get("linear_velocity", [0, 0, 0])
    av = d.get("angular_velocity", [0, 0, 0])
    s = d.get("sensor", {})

    return {
        # --- 时间 ---
        "pc_time":          rec.get("pc_time"),
        "stamp_time":       rec.get("stamp", {}).get("time"),
        "stamp_secs":       rec.get("stamp", {}).get("secs"),
        "stamp_nsecs":      rec.get("stamp", {}).get("nsecs"),
        "year":             t.get("year"),
        "month":            t.get("month"),
        "day":              t.get("day"),
        "hour":             t.get("hour"),
        "minute":           t.get("minute"),
        "second":           t.get("second"),
        "seq":              d.get("header", {}).get("seq"),
        # --- 控制模式 ---
        "control_mode":     d.get("control_mode"),
        # --- 当前位姿 ---
        "latitude":         pose.get("latitude"),
        "longitude":        pose.get("longitude"),
        "altitude":         pose.get("altitude"),
        "depth":            pose.get("depth"),
        "roll":             pose.get("roll"),
        "pitch":            pose.get("pitch"),
        "yaw":              pose.get("yaw"),
        "speed":            pose.get("speed"),
        # --- 目标位姿 ---
        "target_latitude":  target.get("latitude"),
        "target_longitude": target.get("longitude"),
        "target_altitude":  target.get("altitude"),
        "target_depth":     target.get("depth"),
        "target_roll":      target.get("roll"),
        "target_pitch":     target.get("pitch"),
        "target_yaw":       target.get("yaw"),
        "target_speed":     target.get("speed"),
        # --- 推进力/力矩指令 ---
        "motor_TX":         mf.get("TX"),
        "motor_TY":         mf.get("TY"),
        "motor_TZ":         mf.get("TZ"),
        "motor_MX":         mf.get("MX"),
        "motor_MY":         mf.get("MY"),
        "motor_MZ":         mf.get("MZ"),
        # --- 速度 ---
        "linear_vx":        lv[0] if len(lv) > 0 else None,
        "linear_vy":        lv[1] if len(lv) > 1 else None,
        "linear_vz":        lv[2] if len(lv) > 2 else None,
        "angular_wx":       av[0] if len(av) > 0 else None,
        "angular_wy":       av[1] if len(av) > 1 else None,
        "angular_wz":       av[2] if len(av) > 2 else None,
        # --- 电控状态 ---
        "ctrl_temp":        s.get("temperature"),
        "ctrl_voltage":     s.get("voltage"),
        "ctrl_current":     s.get("current"),
        "battery":          s.get("battery"),
        "leak_alarm":       s.get("leak_alarm"),
        "sensor_valid":     s.get("sensor_valid"),
        "sensor_updated":   s.get("sensor_updated"),
        "fault_status":     s.get("fault_status"),
        "power_status":     s.get("power_status"),
    }

def extract_sensor(rec):
    """从 SensorStatus 记录中提取字段字典"""
    d = rec.get("data", {})
    return {
        # power1 = 控制电源
        "ctrl_pwr_voltage":  d.get("power1_voltage"),
        "ctrl_pwr_current":  d.get("power1_current"),
        "ctrl_pwr_power":    d.get("power1_power"),
        "ctrl_pwr_valid":    d.get("power1_valid"),
        # power2 = 动力电源
        "prop_pwr_voltage":  d.get("power2_voltage"),
        "prop_pwr_current":  d.get("power2_current"),
        "prop_pwr_power":    d.get("power2_power"),
        "prop_pwr_valid":    d.get("power2_valid"),
    }

# ---------------------------------------------------------------------------
# 4. 主流程
# ---------------------------------------------------------------------------
def main():
    print("=" * 60)
    print("AUV 日志解析 → CSV")
    print("=" * 60)

    # 加载
    print("\n[1/4] 加载 JSONL 文件...")
    nav_recs, debug_recs, sensor_recs = load_all_logs(LOG_DIR)
    print(f"  NavData:     {len(nav_recs):>8} 条")
    print(f"  AUVData:     {len(debug_recs):>8} 条")
    print(f"  SensorStatus:{len(sensor_recs):>8} 条")

    if not debug_recs:
        print("\n错误: 没有找到 AUVData 记录，无法生成 CSV")
        sys.exit(1)

    # 排序（按 stamp.time）
    print("\n[2/4] 按时间戳排序...")
    debug_recs.sort(key=lambda r: r.get("stamp", {}).get("time", 0))
    nav_recs.sort(key=lambda r: r.get("stamp", {}).get("time", 0))
    sensor_recs.sort(key=lambda r: r.get("stamp", {}).get("time", 0))

    # 时间对齐
    print("\n[3/4] 时间对齐（以 AUVData 为主轴，最近邻匹配）...")
    def stamp_time(r):
        return r.get("stamp", {}).get("time", 0)

    nav_aligned = align_by_nearest(debug_recs, nav_recs, stamp_time, stamp_time)
    sensor_aligned = align_by_nearest(debug_recs, sensor_recs, stamp_time, stamp_time)

    # 合并写入 CSV
    print("\n[4/4] 生成 CSV...")
    os.makedirs(OUT_DIR, exist_ok=True)

    all_fields = []
    # 先构建字段列表
    sample_debug = extract_debug(debug_recs[0])
    sample_nav = extract_nav(nav_recs[0]) if nav_recs else {}
    sample_sensor = extract_sensor(sensor_recs[0]) if sensor_recs else {}

    fieldnames = list(sample_debug.keys())
    fieldnames += [k for k in sample_nav.keys() if k not in sample_debug]
    fieldnames += [k for k in sample_sensor.keys() if k not in sample_debug and k not in sample_nav]

    with open(OUT_FILE, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()

        for i, debug_rec in enumerate(debug_recs):
            row = extract_debug(debug_rec)

            # 合并 nav
            nav_rec = nav_aligned[i]
            if nav_rec is not None:
                row.update(extract_nav(nav_rec))

            # 合并 sensor
            sensor_rec = sensor_aligned[i]
            if sensor_rec is not None:
                row.update(extract_sensor(sensor_rec))

            writer.writerow(row)

            if (i + 1) % 10000 == 0:
                print(f"  已写入 {i + 1}/{len(debug_recs)} 行...")

    print(f"\n完成! 输出文件: {OUT_FILE}")
    print(f"总行数: {len(debug_recs)}")
    print(f"总列数: {len(fieldnames)}")

    # 显示前几列作为预览
    print(f"\nCSV 列名 ({len(fieldnames)} 列):")
    for i, name in enumerate(fieldnames):
        print(f"  {i + 1:3d}. {name}")

if __name__ == "__main__":
    main()
