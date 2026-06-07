"""生成 2026-06-01 水池实验 u/v/w/q/r 单轴阶跃响应图。"""

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


DATA_FILE = Path("data/raw/debug_data260601-1.csv")
REPO_PIC_DIR = Path("assets/figures")
OBSIDIAN_DIR = Path("C:/Users/sixuh/Documents/A_temp/test_obs/test")
OUT_NAME = "pool_test_20260601_single_axis_step_uvwqr.png"


def pad_limits(values: np.ndarray) -> tuple[float, float]:
    """按数据范围增加显示边距，避免曲线贴边。"""
    values = values[np.isfinite(values)]
    if values.size == 0:
        return -1.0, 1.0
    low = float(np.min(values))
    high = float(np.max(values))
    if abs(high - low) < 1e-9:
        pad = max(1.0, abs(high) * 0.1)
    else:
        pad = (high - low) * 0.15
    return low - pad, high + pad


def find_segments(mask: np.ndarray) -> list[tuple[int, int]]:
    """从布尔掩码中提取连续区间。"""
    padded = np.r_[False, mask, False]
    edges = np.diff(padded.astype(int))
    starts = np.flatnonzero(edges == 1)
    ends = np.flatnonzero(edges == -1) - 1
    return list(zip(starts, ends))


def choose_window(t: np.ndarray, cmd: np.ndarray, resp: np.ndarray, mode: np.ndarray, threshold: float) -> int:
    """优先选择 mode=1 中持续时间足够长且响应变化最大的阶跃段。"""
    active = (mode == 1) & (np.abs(cmd) >= threshold)
    segments = find_segments(active)
    best_score = -np.inf
    best_start = int(np.argmax(np.abs(cmd)))

    for start, end in segments:
        duration = t[end] - t[start]
        if duration < 3.0:
            continue
        pre = (t >= t[start] - 8.0) & (t < t[start])
        post = (t >= t[start]) & (t <= t[start] + 42.0)
        if not np.any(post):
            continue
        baseline = np.nanmedian(resp[pre]) if np.any(pre) else resp[start]
        resp_delta = np.nanmax(np.abs(resp[post] - baseline))
        cmd_peak = np.nanmax(np.abs(cmd[start : end + 1]))
        score = duration * max(resp_delta, 1e-3) * cmd_peak
        if score > best_score:
            best_score = score
            best_start = start

    return best_start


def main() -> None:
    """读取 CSV，生成五自由度阶跃响应汇总图。"""
    data = pd.read_csv(DATA_FILE)
    t = (data["pc_timestamp"] - data["pc_timestamp"].iloc[0]).to_numpy()
    mode = data["mode"].to_numpy()

    channels = [
        {
            "name": "u",
            "title": "Surge u",
            "cmd": data["force_cmd1"].to_numpy(),
            "resp": data["linear_vel_x"].to_numpy(),
            "cmd_label": "TX command (CAN-g)",
            "resp_label": "u (m/s)",
            "threshold": 600,
        },
        {
            "name": "v",
            "title": "Sway v",
            "cmd": data["force_cmd2"].to_numpy(),
            "resp": data["linear_vel_y"].to_numpy(),
            "cmd_label": "TY command (CAN-g)",
            "resp_label": "v (m/s)",
            "threshold": 600,
        },
        {
            "name": "w",
            "title": "Heave w",
            "cmd": data["force_cmd3"].to_numpy(),
            "resp": data["linear_vel_z"].to_numpy(),
            "cmd_label": "TZ command (CAN-g)",
            "resp_label": "w (m/s)",
            "threshold": 600,
        },
        {
            "name": "q",
            "title": "Pitch rate q (no MY step, TZ-coupled response)",
            "cmd": data["force_cmd3"].to_numpy(),
            "resp": data["angular_vel_y"].to_numpy(),
            "cmd_label": "TZ command (CAN-g)",
            "resp_label": "q (deg/s)",
            "threshold": 600,
        },
        {
            "name": "r",
            "title": "Yaw rate r",
            "cmd": data["force_cmd6"].to_numpy(),
            "resp": data["angular_vel_z"].to_numpy(),
            "cmd_label": "MZ command (CAN-g)",
            "resp_label": "r (deg/s)",
            "threshold": 600,
        },
    ]

    REPO_PIC_DIR.mkdir(parents=True, exist_ok=True)
    fig, axes = plt.subplots(len(channels), 1, figsize=(14.5, 13), sharex=False)
    fig.suptitle("AUV pool test single-axis step responses (2026-06-01, mode=1, u/v/w/q/r)", fontsize=14)

    print("单轴阶跃响应窗口:")
    print(f"{'DOF':<8} {'t0(s)':<10} {'cmd':<10} {'resp0':<10} {'resp_peak':<10}")

    for ax, channel in zip(axes, channels):
        step_idx = choose_window(t, channel["cmd"], channel["resp"], mode, channel["threshold"])
        t0 = t[step_idx]
        window = (t >= t0 - 8.0) & (t <= t0 + 42.0)
        rel_t = t[window] - t0
        cmd_seg = channel["cmd"][window]
        resp_seg = channel["resp"][window]

        baseline_mask = rel_t < 0
        resp0 = float(np.nanmedian(resp_seg[baseline_mask])) if np.any(baseline_mask) else float(resp_seg[0])
        peak_idx = int(np.nanargmax(np.abs(resp_seg - resp0)))
        resp_peak = float(resp_seg[peak_idx])

        ax.step(rel_t, cmd_seg, where="post", color="#0050c8", linewidth=1.8, label=channel["cmd_label"])
        ax.scatter(rel_t[::8], cmd_seg[::8], color="#0050c8", s=7)
        ax.set_ylabel(channel["cmd_label"], color="#0050c8")
        ax.tick_params(axis="y", labelcolor="#0050c8")
        ax.set_ylim(*pad_limits(cmd_seg))
        ax.grid(True, alpha=0.28)
        ax.axvline(0, color="#666666", linestyle="--", linewidth=1.0)
        ax.text(0.3, ax.get_ylim()[0] + 0.08 * (ax.get_ylim()[1] - ax.get_ylim()[0]), "step start", color="#666666", fontsize=8)

        ax_resp = ax.twinx()
        ax_resp.plot(rel_t, resp_seg, color="#d92510", linewidth=1.8, label=channel["resp_label"])
        ax_resp.scatter(rel_t[::8], resp_seg[::8], color="#d92510", s=7)
        ax_resp.set_ylabel(channel["resp_label"], color="#d92510")
        ax_resp.tick_params(axis="y", labelcolor="#d92510")
        ax_resp.set_ylim(*pad_limits(resp_seg))

        ax.set_title(f"{channel['title']} step response (t0={t0:.1f}s)", fontsize=10)
        ax.set_xlabel("Relative time (s)")

        print(f"{channel['name']:<8} {t0:<10.1f} {channel['cmd'][step_idx]:<10.0f} {resp0:<10.3f} {resp_peak:<10.3f}")

    fig.tight_layout(rect=(0, 0, 1, 0.975))
    repo_out = REPO_PIC_DIR / OUT_NAME
    obsidian_out = OBSIDIAN_DIR / OUT_NAME
    fig.savefig(repo_out, dpi=200)
    if OBSIDIAN_DIR.exists():
        fig.savefig(obsidian_out, dpi=200)
    plt.close(fig)

    print(f"图片已保存: {repo_out}")
    print(f"图片已复制: {obsidian_out}")


if __name__ == "__main__":
    main()
