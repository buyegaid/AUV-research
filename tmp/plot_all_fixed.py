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
    'label':      'TX Forward'
}
tx_rev = {
    'thrust_pct': [20, 40, 60, 80, 100],
    'T_est':      [0.72, 1.44, 2.17, 2.89, 3.61],
    'v_ss':       [0.197, 0.362, 0.438, 0.488, 0.477],
    'label':      'TX Reverse'
}
ty_right = {
    'thrust_pct': [20, 40, 60, 80, 100],
    'T_est':      [2.41, 4.82, 7.24, 9.65, 12.06],
    'v_ss':       [0.108, 0.163, 0.196, 0.232, 0.280],
    'label':      'TY Starboard'
}
ty_left = {
    'thrust_pct': [20, 40, 60, 80, 100],
    'T_est':      [1.17, 2.33, 3.50, 4.66, 5.83],
    'v_ss':       [0.049, 0.105, 0.137, 0.160, 0.195],
    'label':      'TY Port'
}
# 修正: rad/s -> deg/s
mz_right = {
    'thrust_pct': [20, 40, 60, 100],
    'T_est':      [2.60, 4.24, 6.52, 10.86],
    'w_ss':       [0.390*180/np.pi, 0.515*180/np.pi, 0.539*180/np.pi, 0.743*180/np.pi],
    'label':      'MZ Starboard'
}
mz_left = {
    'thrust_pct': [100],
    'T_est':      [10.86],
    'w_ss':       [0.736*180/np.pi],
    'label':      'MZ Port'
}

# ===== 绘图 =====
fig, axes = plt.subplots(2, 2, figsize=(14, 11))

# --- (a) Surge ---
ax = axes[0, 0]
ax.plot(tx_fwd['T_est'], tx_fwd['v_ss'], 'o-', color='#2196F3', lw=2, ms=9, label='TX Forward')
ax.plot(tx_rev['T_est'], tx_rev['v_ss'], 's--', color='#FF5722', lw=2, ms=9, label='TX Reverse')
for pct, t, v in zip(tx_fwd['thrust_pct'], tx_fwd['T_est'], tx_fwd['v_ss']):
    ax.annotate(f'{pct}%', (t, v), textcoords="offset points", xytext=(12, 5), fontsize=9, color='#2196F3')
for pct, t, v in zip(tx_rev['thrust_pct'], tx_rev['T_est'], tx_rev['v_ss']):
    ax.annotate(f'{pct}%', (t, v), textcoords="offset points", xytext=(12, -10), fontsize=9, color='#FF5722')
u_fit = np.linspace(0, 1.0, 100)
ax.plot(0.65*u_fit + 10.85*u_fit**2, u_fit, '--', color='gray', lw=1, alpha=0.5, label='CFD drag')
ax.set_xlabel('Thrust / Drag (N)', fontsize=12)
ax.set_ylabel('Steady-state Velocity (m/s)', fontsize=12)
ax.set_title('Surge: Velocity vs Thrust', fontsize=14, fontweight='bold')
ax.legend(fontsize=10); ax.grid(True, alpha=0.3); ax.set_xlim(0, 10); ax.set_ylim(0, 1.0)

# --- (b) Sway ---
ax = axes[0, 1]
ax.plot(ty_right['T_est'], ty_right['v_ss'], 'o-', color='#4CAF50', lw=2, ms=9, label='TY Starboard')
ax.plot(ty_left['T_est'], ty_left['v_ss'], 's--', color='#9C27B0', lw=2, ms=9, label='TY Port')
for pct, t, v in zip(ty_right['thrust_pct'], ty_right['T_est'], ty_right['v_ss']):
    ax.annotate(f'{pct}%', (t, v), textcoords="offset points", xytext=(12, 5), fontsize=9, color='#4CAF50')
for pct, t, v in zip(ty_left['thrust_pct'], ty_left['T_est'], ty_left['v_ss']):
    ax.annotate(f'{pct}%', (t, v), textcoords="offset points", xytext=(12, -10), fontsize=9, color='#9C27B0')
v_fit = np.linspace(0, 0.35, 100)
ax.plot(1.67*v_fit + 178.59*v_fit**2, v_fit, '--', color='gray', lw=1, alpha=0.5, label='CFD drag')
ax.set_xlabel('Thrust / Drag (N)', fontsize=12)
ax.set_ylabel('Steady-state Velocity (m/s)', fontsize=12)
ax.set_title('Sway: Velocity vs Thrust', fontsize=14, fontweight='bold')
ax.legend(fontsize=10); ax.grid(True, alpha=0.3); ax.set_xlim(0, 14); ax.set_ylim(0, 0.35)

# --- (c) Thrust Level % vs Velocity ---
ax = axes[1, 0]
ax.plot(tx_fwd['thrust_pct'], tx_fwd['v_ss'], 'o-', color='#2196F3', lw=2, ms=9, label='TX Forward')
ax.plot(tx_rev['thrust_pct'], tx_rev['v_ss'], 's--', color='#FF5722', lw=2, ms=9, label='TX Reverse')
ax.plot(ty_right['thrust_pct'], ty_right['v_ss'], 'D-', color='#4CAF50', lw=2, ms=9, label='TY Starboard')
ax.plot(ty_left['thrust_pct'], ty_left['v_ss'], '^--', color='#9C27B0', lw=2, ms=9, label='TY Port')
ax.set_xlabel('Thrust Level (%)', fontsize=12)
ax.set_ylabel('Steady-state Velocity (m/s)', fontsize=12)
ax.set_title('Translation: Thrust Level vs Velocity', fontsize=14, fontweight='bold')
ax.legend(fontsize=9); ax.grid(True, alpha=0.3); ax.set_ylim(0, 1.0)

# --- (d) Yaw (unit corrected) ---
ax = axes[1, 1]
ax.plot(mz_right['T_est'], mz_right['w_ss'], 'o-', color='#FF9800', lw=2.5, ms=10, label='MZ Starboard')
ax.plot(mz_left['T_est'], mz_left['w_ss'], 's', color='#E91E63', ms=14, label='MZ Port')
for pct, t, w in zip(mz_right['thrust_pct'], mz_right['T_est'], mz_right['w_ss']):
    ax.annotate(f'{pct}%', (t, w), textcoords="offset points", xytext=(14, 6), fontsize=10,
                color='#FF9800', fontweight='bold')
for pct, t, w in zip(mz_left['thrust_pct'], mz_left['T_est'], mz_left['w_ss']):
    ax.annotate(f'{pct}%', (t, w), textcoords="offset points", xytext=(14, -14), fontsize=10,
                color='#E91E63', fontweight='bold')
r_fit = np.linspace(0, 1.0, 100)
N_fit = 0.2046 * r_fit + 18.20 * r_fit**2
ax.plot(N_fit, np.rad2deg(r_fit), '--', color='gray', lw=1.5, alpha=0.5, label='CFD yaw drag')
ax.axhline(y=45, color='red', linestyle=':', lw=1.5, alpha=0.6)
ax.annotate('reported ~45 deg/s', (1, 45), textcoords="offset points",
            xytext=(5, 5), fontsize=9, color='red', style='italic')
ax.set_xlabel('Torque / Drag Moment (N m)', fontsize=12)
ax.set_ylabel('Steady-state Yaw Rate (deg/s)', fontsize=12)
ax.set_title('Yaw: Yaw Rate vs Torque (unit: deg/s)', fontsize=14, fontweight='bold')
ax.legend(fontsize=10, loc='lower right'); ax.grid(True, alpha=0.3)
ax.set_xlim(0, 13); ax.set_ylim(0, 55)

plt.tight_layout()
out_path = r'C:\Users\sixuh\Documents\A_temp\test_obs\test\AUV性能评估_稳态速度vs推力.png'
plt.savefig(out_path, dpi=150, bbox_inches='tight')
plt.close()
print(f"Updated: {out_path}")

# ===== 最终汇总 =====
print("\n" + "=" * 60)
print("Final Summary (Yaw unit corrected)")
print("=" * 60)
print(f"\nSurge:  max fwd {tx_fwd['v_ss'][-1]:.2f} m/s ({tx_fwd['v_ss'][-1]/0.514:.1f} kn) | rev {tx_rev['v_ss'][3]:.2f} m/s | ratio {tx_rev['v_ss'][3]/tx_fwd['v_ss'][3]*100:.0f}%")
print(f"Sway:   max stbd {ty_right['v_ss'][-1]:.3f} m/s | port {ty_left['v_ss'][-1]:.3f} m/s | ratio {ty_left['v_ss'][-1]/ty_right['v_ss'][-1]*100:.0f}%")
print(f"Yaw:    max stbd {mz_right['w_ss'][-1]:.1f} deg/s | port {mz_left['w_ss'][-1]:.1f} deg/s | ratio ~{mz_left['w_ss'][-1]/mz_right['w_ss'][-1]*100:.0f}%")
print(f"\nNote: original table header 'omega_ss (deg/s)' was wrong; values are in rad/s.")
print(f"  ex: 0.743 rad/s * 180/pi = 42.6 deg/s (matches reported ~45 deg/s)")
