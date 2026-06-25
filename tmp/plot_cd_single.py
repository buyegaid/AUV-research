import matplotlib.pyplot as plt
import matplotlib
import numpy as np

matplotlib.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'DejaVu Sans']
matplotlib.rcParams['axes.unicode_minus'] = False

# ===== 数据 =====
tx_fwd = {
    'thrust_pct': [20, 40, 60, 80, 100],
    'T_est':      [1.78, 3.55, 5.33, 7.10, 8.88],
    'v_ss':       [0.251, 0.592, 0.685, 0.773, 0.870],
    'label':      'TX 前进'
}
tx_rev = {
    'thrust_pct': [20, 40, 60, 80, 100],
    'T_est':      [0.72, 1.44, 2.17, 2.89, 3.61],
    'v_ss':       [0.197, 0.362, 0.438, 0.488, 0.477],
    'label':      'TX 后退'
}
ty_right = {
    'thrust_pct': [20, 40, 60, 80, 100],
    'T_est':      [2.41, 4.82, 7.24, 9.65, 12.06],
    'v_ss':       [0.108, 0.163, 0.196, 0.232, 0.280],
    'label':      'TY 右移'
}
ty_left = {
    'thrust_pct': [20, 40, 60, 80, 100],
    'T_est':      [1.17, 2.33, 3.50, 4.66, 5.83],
    'v_ss':       [0.049, 0.105, 0.137, 0.160, 0.195],
    'label':      'TY 左移'
}
mz_right = {
    'thrust_pct': [20, 40, 60, 100],
    'T_est':      [2.60, 4.24, 6.52, 10.86],
    'w_ss':       [0.390*180/np.pi, 0.515*180/np.pi, 0.539*180/np.pi, 0.743*180/np.pi],
    'label':      'MZ 右转'
}
mz_left = {
    'thrust_pct': [100],
    'T_est':      [10.86],
    'w_ss':       [0.736*180/np.pi],
    'label':      'MZ 左转'
}

# ===== (c) 平动通道：推力等级 vs 稳态速度 =====
fig, ax = plt.subplots(figsize=(10, 7))

ax.plot(tx_fwd['thrust_pct'], tx_fwd['v_ss'], 'o-', color='#2196F3', lw=2.5, ms=10, label=tx_fwd['label'])
ax.plot(tx_rev['thrust_pct'], tx_rev['v_ss'], 's--', color='#FF5722', lw=2.5, ms=10, label=tx_rev['label'])
ax.plot(ty_right['thrust_pct'], ty_right['v_ss'], 'D-', color='#4CAF50', lw=2.5, ms=10, label=ty_right['label'])
ax.plot(ty_left['thrust_pct'], ty_left['v_ss'], '^--', color='#9C27B0', lw=2.5, ms=10, label=ty_left['label'])

# 在每个数据点标注速度值
for pct, v in zip(tx_fwd['thrust_pct'], tx_fwd['v_ss']):
    ax.annotate(f'{v:.3f}', (pct, v), textcoords="offset points", xytext=(12, 5),
                fontsize=8, color='#2196F3')
for pct, v in zip(tx_rev['thrust_pct'], tx_rev['v_ss']):
    ax.annotate(f'{v:.3f}', (pct, v), textcoords="offset points", xytext=(12, -10),
                fontsize=8, color='#FF5722')
for pct, v in zip(ty_right['thrust_pct'], ty_right['v_ss']):
    ax.annotate(f'{v:.3f}', (pct, v), textcoords="offset points", xytext=(12, 5),
                fontsize=8, color='#4CAF50')
for pct, v in zip(ty_left['thrust_pct'], ty_left['v_ss']):
    ax.annotate(f'{v:.3f}', (pct, v), textcoords="offset points", xytext=(-12, -12),
                fontsize=8, color='#9C27B0')

ax.set_xlabel('推力等级 (%)', fontsize=14)
ax.set_ylabel('稳态速度 (m/s)', fontsize=14)
ax.set_title('平动通道：推力等级 vs 稳态速度', fontsize=16, fontweight='bold')
ax.legend(fontsize=12, loc='upper left')
ax.grid(True, alpha=0.3)
ax.set_xlim(0, 105)
ax.set_ylim(0, 1.0)

plt.tight_layout()
out_c = r'C:\Users\sixuh\Documents\A_temp\test_obs\test\AUV性能评估_平动_推力等级vs速度.png'
plt.savefig(out_c, dpi=150, bbox_inches='tight')
plt.close()
print(f"已保存: {out_c}")

# ===== (d) 偏航 Yaw =====
fig, ax = plt.subplots(figsize=(10, 7))

ax.plot(mz_right['T_est'], mz_right['w_ss'], 'o-', color='#FF9800', lw=2.5, ms=12, label=mz_right['label'])
ax.plot(mz_left['T_est'], mz_left['w_ss'], 's', color='#E91E63', ms=16, label=mz_left['label'])

for pct, t, w in zip(mz_right['thrust_pct'], mz_right['T_est'], mz_right['w_ss']):
    ax.annotate(f'{pct}%  {w:.1f} deg/s', (t, w), textcoords="offset points",
                xytext=(18, 8), fontsize=11, color='#FF9800', fontweight='bold')
for pct, t, w in zip(mz_left['thrust_pct'], mz_left['T_est'], mz_left['w_ss']):
    ax.annotate(f'{pct}%  {w:.1f} deg/s', (t, w), textcoords="offset points",
                xytext=(18, -16), fontsize=11, color='#E91E63', fontweight='bold')

# CFD 偏航阻尼曲线
r_fit = np.linspace(0, 1.0, 100)
N_fit = 0.2046 * r_fit + 18.20 * r_fit**2
ax.plot(N_fit, np.rad2deg(r_fit), '--', color='gray', lw=2, alpha=0.5, label='CFD偏航阻尼曲线')

# 实测最大参考线
ax.axhline(y=45, color='red', linestyle=':', lw=2, alpha=0.6)
ax.annotate('实测最大 ~45 deg/s', (1, 45.5), textcoords="offset points",
            xytext=(10, 8), fontsize=12, color='red', style='italic')

# 数据拟合趋势线
z = np.polyfit(mz_right['T_est'], mz_right['w_ss'], 1)
T_fit = np.linspace(0, 13, 50)
w_fit = np.polyval(z, T_fit)
ax.plot(T_fit, w_fit, ':', color='#FF9800', lw=1, alpha=0.4)

ax.set_xlabel('力矩 / 阻力矩 (N·m)', fontsize=14)
ax.set_ylabel('稳态角速度 (°/s)', fontsize=14)
ax.set_title('偏航 (Yaw)：稳态角速度 vs 力矩', fontsize=16, fontweight='bold')
ax.legend(fontsize=12, loc='lower right')
ax.grid(True, alpha=0.3)
ax.set_xlim(0, 13)
ax.set_ylim(0, 55)

plt.tight_layout()
out_d = r'C:\Users\sixuh\Documents\A_temp\test_obs\test\AUV性能评估_偏航_力矩vs角速度.png'
plt.savefig(out_d, dpi=150, bbox_inches='tight')
plt.close()
print(f"已保存: {out_d}")
