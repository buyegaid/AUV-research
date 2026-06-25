import matplotlib.pyplot as plt
import matplotlib
import numpy as np

matplotlib.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'DejaVu Sans']
matplotlib.rcParams['axes.unicode_minus'] = False

# ===== 偏航数据 —— 原表数值为 rad/s，修正单位 =====
# MZ 右转: ω_ss 原值(rad/s) → 转换为 °/s
mz_right_rad = [0.390, 0.515, 0.539, 0.743]       # rad/s
mz_right_deg = [r * 180/np.pi for r in mz_right_rad]  # °/s
mz_right_T   = [2.60, 4.24, 6.52, 10.86]            # N·m
mz_right_pct = [20, 40, 60, 100]

# MZ 左转: 仅100%有效
mz_left_rad = [0.736]
mz_left_deg = [r * 180/np.pi for r in mz_left_rad]
mz_left_T   = [10.86]
mz_left_pct  = [100]

# CFD偏航阻尼曲线 (rad/s)
r_rad_fit = np.linspace(0, 1.0, 100)
N_fit = 0.2046 * r_rad_fit + 18.20 * r_rad_fit**2
r_deg_fit = np.rad2deg(r_rad_fit)

fig, ax = plt.subplots(figsize=(10, 7))

# 数据点
ax.plot(mz_right_T, mz_right_deg, 'o-', color='#FF9800', linewidth=2.5, markersize=10,
        label='MZ Starboard Turn')
ax.plot(mz_left_T, mz_left_deg, 's', color='#E91E63', markersize=14,
        label='MZ Port Turn')

# 标注推力等级百分比
for pct, t, w in zip(mz_right_pct, mz_right_T, mz_right_deg):
    ax.annotate(f'{pct}%', (t, w), textcoords="offset points",
                xytext=(14, 6), fontsize=11, color='#FF9800', fontweight='bold')
for pct, t, w in zip(mz_left_pct, mz_left_T, mz_left_deg):
    ax.annotate(f'{pct}%', (t, w), textcoords="offset points",
                xytext=(14, -14), fontsize=11, color='#E91E63', fontweight='bold')

# CFD 阻力曲线
ax.plot(N_fit, r_deg_fit, '--', color='gray', linewidth=1.5, alpha=0.6,
        label='CFD Yaw Drag (d1=0.205, d2=18.2)')

# 标注用户实测 45 deg/s 参考线
ax.axhline(y=45, color='red', linestyle=':', linewidth=1.5, alpha=0.7)
ax.annotate('Reported max ~45 deg/s', (1, 45), textcoords="offset points",
            xytext=(5, 5), fontsize=10, color='red', style='italic')

ax.set_xlabel('Torque / Drag Moment (N m)', fontsize=13)
ax.set_ylabel('Steady-state Yaw Rate (deg/s)', fontsize=13)
ax.set_title('Yaw: Steady Yaw Rate vs Torque (unit corrected: rad/s -> deg/s)',
             fontsize=14, fontweight='bold')
ax.legend(fontsize=11, loc='lower right')
ax.grid(True, alpha=0.3)
ax.set_xlim(0, 13)
ax.set_ylim(0, 55)

# 添加次纵轴: rad/s
secax = ax.secondary_yaxis('right', functions=(lambda d: d*np.pi/180, lambda r: r*180/np.pi))
secax.set_ylabel('Yaw Rate (rad/s)', fontsize=11)

# 打印修正后的数据表
print("=" * 65)
print("MZ Yaw Rate: 原表值(rad/s) vs 修正后(deg/s)")
print("=" * 65)
print(f"{'Thrust':>7s}  {'M (N.m)':>8s}  {'Raw(rad/s)':>10s}  {'Corr(deg/s)':>11s}")
print("-" * 65)
for pct, t, w_rad in zip(mz_right_pct, mz_right_T, mz_right_rad):
    print(f"{'R '+str(pct)+'%':>7s}  {t:8.2f}  {w_rad:10.3f}  {w_rad*180/np.pi:11.1f}")
for pct, t, w_rad in zip(mz_left_pct, mz_left_T, mz_left_rad):
    print(f"{'L '+str(pct)+'%':>7s}  {t:8.2f}  {w_rad:10.3f}  {w_rad*180/np.pi:11.1f}")

print(f"\nCFD Yaw drag check at 0.74 rad/s (42.4 deg/s):")
print(f"  N = 0.2046*0.74 + 18.20*0.74^2 = {0.2046*0.74 + 18.20*0.74**2:.1f} N.m")
print(f"  (Table T_est@100% = 10.86 N.m, reasonable match)")

plt.tight_layout()
out_path = r'C:\Users\sixuh\Documents\A_temp\test_obs\test\AUV性能评估_Yaw_单位修正.png'
plt.savefig(out_path, dpi=150, bbox_inches='tight')
plt.close()
print(f"\nImage saved: {out_path}")
