import matplotlib.pyplot as plt
import matplotlib
import numpy as np

matplotlib.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'DejaVu Sans']
matplotlib.rcParams['axes.unicode_minus'] = False

# ===== 数据提取 (每个推力等级取一个代表值) =====
# 依据: 推进器推力校准方案_20260621 第十一节
# 选取原则: 多段取最高/最可信的稳态速度

# TX 前进 (Surge Forward) — 40%取0.592, 60%取0.685, 100%取0.870
tx_fwd = {
    'thrust_pct': [20, 40, 60, 80, 100],
    'T_est':      [1.78, 3.55, 5.33, 7.10, 8.88],
    'v_ss':       [0.251, 0.592, 0.685, 0.773, 0.870],
    'label':      'TX qianjin'
}

# TX 后退 (Surge Reverse) — 100%取0.477(有响应段)
tx_rev = {
    'thrust_pct': [20, 40, 60, 80, 100],
    'T_est':      [0.72, 1.44, 2.17, 2.89, 3.61],
    'v_ss':       [0.197, 0.362, 0.438, 0.488, 0.477],
    'label':      'TX houtui'
}

# TY 右移 (Sway Starboard)
ty_right = {
    'thrust_pct': [20, 40, 60, 80, 100],
    'T_est':      [2.41, 4.82, 7.24, 9.65, 12.06],
    'v_ss':       [0.108, 0.163, 0.196, 0.232, 0.280],
    'label':      'TY youyi'
}

# TY 左移 (Sway Port) — 80%取0.160(中值)
ty_left = {
    'thrust_pct': [20, 40, 60, 80, 100],
    'T_est':      [1.17, 2.33, 3.50, 4.66, 5.83],
    'v_ss':       [0.049, 0.105, 0.137, 0.160, 0.195],
    'label':      'TY zuoyi'
}

# MZ 右转 (Yaw Right) — 20%取0.390, 40%取0.515
mz_right = {
    'thrust_pct': [20, 40, 60, 100],
    'T_est':      [2.60, 4.24, 6.52, 10.86],
    'w_ss':       [0.390, 0.515, 0.539, 0.743],
    'label':      'MZ youzhuan'
}

# MZ 左转 (Yaw Left) — 仅100%有效
mz_left = {
    'thrust_pct': [100],
    'T_est':      [10.86],
    'w_ss':       [0.736],
    'label':      'MZ zuozhuan'
}

# ===== 绘图 =====
fig, axes = plt.subplots(2, 2, figsize=(14, 11))

# --- (a) Surge: Thrust vs Steady-state Velocity ---
ax = axes[0, 0]
ax.plot(tx_fwd['T_est'], tx_fwd['v_ss'], 'o-', color='#2196F3', linewidth=2, markersize=9,
        label='TX Forward')
ax.plot(tx_rev['T_est'], tx_rev['v_ss'], 's--', color='#FF5722', linewidth=2, markersize=9,
        label='TX Reverse')
for pct, t, v in zip(tx_fwd['thrust_pct'], tx_fwd['T_est'], tx_fwd['v_ss']):
    ax.annotate(f'{pct}%', (t, v), textcoords="offset points",
                xytext=(12, 5), fontsize=9, color='#2196F3')
for pct, t, v in zip(tx_rev['thrust_pct'], tx_rev['T_est'], tx_rev['v_ss']):
    ax.annotate(f'{pct}%', (t, v), textcoords="offset points",
                xytext=(12, -10), fontsize=9, color='#FF5722')

# CFD阻力曲线: F = d1*u + d2*u*|u|
u_fit = np.linspace(0, 1.0, 100)
F_fit = 0.65 * u_fit + 10.85 * u_fit**2
ax.plot(F_fit, u_fit, '--', color='gray', linewidth=1, alpha=0.6, label='CFD drag')

ax.set_xlabel('Thrust / Drag (N)', fontsize=12)
ax.set_ylabel('Steady-state Velocity (m/s)', fontsize=12)
ax.set_title('Surge: Steady Velocity vs Thrust', fontsize=14, fontweight='bold')
ax.legend(fontsize=10)
ax.grid(True, alpha=0.3)
ax.set_xlim(0, 10)
ax.set_ylim(0, 1.0)

# --- (b) Sway: Thrust vs Velocity ---
ax = axes[0, 1]
ax.plot(ty_right['T_est'], ty_right['v_ss'], 'o-', color='#4CAF50', linewidth=2, markersize=9,
        label='TY Starboard')
ax.plot(ty_left['T_est'], ty_left['v_ss'], 's--', color='#9C27B0', linewidth=2, markersize=9,
        label='TY Port')
for pct, t, v in zip(ty_right['thrust_pct'], ty_right['T_est'], ty_right['v_ss']):
    ax.annotate(f'{pct}%', (t, v), textcoords="offset points",
                xytext=(12, 5), fontsize=9, color='#4CAF50')
for pct, t, v in zip(ty_left['thrust_pct'], ty_left['T_est'], ty_left['v_ss']):
    ax.annotate(f'{pct}%', (t, v), textcoords="offset points",
                xytext=(12, -10), fontsize=9, color='#9C27B0')

v_fit = np.linspace(0, 0.35, 100)
Fy_fit = 1.67 * v_fit + 178.59 * v_fit**2
ax.plot(Fy_fit, v_fit, '--', color='gray', linewidth=1, alpha=0.6, label='CFD drag')

ax.set_xlabel('Thrust / Drag (N)', fontsize=12)
ax.set_ylabel('Steady-state Velocity (m/s)', fontsize=12)
ax.set_title('Sway: Steady Velocity vs Thrust', fontsize=14, fontweight='bold')
ax.legend(fontsize=10)
ax.grid(True, alpha=0.3)
ax.set_xlim(0, 14)
ax.set_ylim(0, 0.35)

# --- (c) Thrust Level % vs Velocity (Surge+Sway) ---
ax = axes[1, 0]
ax.plot(tx_fwd['thrust_pct'], tx_fwd['v_ss'], 'o-', color='#2196F3', linewidth=2, markersize=9,
        label='TX Forward')
ax.plot(tx_rev['thrust_pct'], tx_rev['v_ss'], 's--', color='#FF5722', linewidth=2, markersize=9,
        label='TX Reverse')
ax.plot(ty_right['thrust_pct'], ty_right['v_ss'], 'D-', color='#4CAF50', linewidth=2, markersize=9,
        label='TY Starboard')
ax.plot(ty_left['thrust_pct'], ty_left['v_ss'], '^--', color='#9C27B0', linewidth=2, markersize=9,
        label='TY Port')
ax.set_xlabel('Thrust Level (%)', fontsize=12)
ax.set_ylabel('Steady-state Velocity (m/s)', fontsize=12)
ax.set_title('Translation: Thrust Level vs Steady Velocity', fontsize=14, fontweight='bold')
ax.legend(fontsize=9)
ax.grid(True, alpha=0.3)
ax.set_ylim(0, 1.0)

# --- (d) Yaw: Torque vs Angular Velocity ---
ax = axes[1, 1]
ax.plot(mz_right['T_est'], mz_right['w_ss'], 'o-', color='#FF9800', linewidth=2, markersize=9,
        label='MZ Starboard')
ax.plot(mz_left['T_est'], mz_left['w_ss'], 's', color='#E91E63', markersize=12,
        label='MZ Port')
for pct, t, w in zip(mz_right['thrust_pct'], mz_right['T_est'], mz_right['w_ss']):
    ax.annotate(f'{pct}%', (t, w), textcoords="offset points",
                xytext=(12, 5), fontsize=9, color='#FF9800')
for pct, t, w in zip(mz_left['thrust_pct'], mz_left['T_est'], mz_left['w_ss']):
    ax.annotate(f'{pct}%', (t, w), textcoords="offset points",
                xytext=(12, -10), fontsize=9, color='#E91E63')

r_fit = np.linspace(0, 0.015, 100)
N_fit = 0.2046 * r_fit + 18.20 * r_fit**2
r_fit_deg = np.rad2deg(r_fit)
ax.plot(N_fit, r_fit_deg, '--', color='gray', linewidth=1, alpha=0.6,
        label='CFD yaw drag')

ax.set_xlabel('Torque / Drag moment (N m)', fontsize=12)
ax.set_ylabel('Steady-state Yaw Rate (deg/s)', fontsize=12)
ax.set_title('Yaw: Steady Yaw Rate vs Torque', fontsize=14, fontweight='bold')
ax.legend(fontsize=10)
ax.grid(True, alpha=0.3)
ax.set_xlim(0, 12)
ax.set_ylim(0, 1.0)

plt.tight_layout()
out_path = r'C:\Users\sixuh\Documents\A_temp\test_obs\test\AUV性能评估_稳态速度vs推力.png'
plt.savefig(out_path, dpi=150, bbox_inches='tight')
plt.close()
print(f"Image saved: {out_path}")

# ===== Print summary =====
print("\n" + "=" * 60)
print("Steady-state Velocity Summary (one point per thrust level)")
print("=" * 60)

for name, d in [("TX Forward", tx_fwd), ("TX Reverse", tx_rev),
                ("TY Starboard", ty_right), ("TY Port", ty_left)]:
    print(f"\n{name}:")
    for pct, t, v in zip(d['thrust_pct'], d['T_est'], d['v_ss']):
        print(f"  {pct:3d}%  T={t:5.2f} N  ->  v={v:.3f} m/s")

for name, d in [("MZ Starboard", mz_right), ("MZ Port", mz_left)]:
    print(f"\n{name}:")
    for pct, t, w in zip(d['thrust_pct'], d['T_est'], d['w_ss']):
        print(f"  {pct:3d}%  M={t:5.2f} Nm ->  w={w:.3f} deg/s")

# Key metrics
print("\n" + "=" * 60)
print("Key Performance Metrics")
print("=" * 60)
print(f"Max forward speed:     {tx_fwd['v_ss'][-1]:.2f} m/s ({tx_fwd['v_ss'][-1]/0.514:.1f} kn)")
print(f"Max reverse speed:     {tx_rev['v_ss'][-2]:.2f} m/s (100% anomaly segments excluded)")
print(f"Fwd/Rev ratio:         {tx_rev['v_ss'][3]/tx_fwd['v_ss'][3]*100:.0f}% (@80%)")
print(f"Max starboard speed:   {ty_right['v_ss'][-1]:.3f} m/s")
print(f"Max port speed:        {ty_left['v_ss'][-1]:.3f} m/s")
print(f"Stbd/Port ratio:       {ty_left['v_ss'][-1]/ty_right['v_ss'][-1]*100:.0f}%")
print(f"Max starboard yaw rate:{mz_right['w_ss'][-1]:.2f} deg/s")
print(f"Max port yaw rate:     {mz_left['w_ss'][-1]:.2f} deg/s")
