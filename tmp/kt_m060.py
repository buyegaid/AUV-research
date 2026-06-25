"""
M060 辅推 KT — 正反向分段PWM
TY右移: +TY→perCAN(负)→反向PWM→|RPM|→KT (per-thruster)
TY左移: -TY→perCAN(正)→正向PWM→RPM→KT (per-thruster)
D=0.06m, rhoD4=0.01296
"""
import numpy as np

rhoD4 = 0.01296; n_max = 2500
CAN_POS = np.array([0, 3900, 5900, 7900, 9500])
OFF_POS = np.array([0, 230,  310,  370,  450])
CAN_NEG = np.array([-7300, -6100, -4800, -3200, 0])
OFF_NEG = np.array([-430,  -370,  -330,  -250,  0])

def can_to_pwm(c):
    off = np.interp(c, CAN_POS, OFF_POS) if c >= 0 else np.interp(c, CAN_NEG, OFF_NEG)
    return 1500 + off

def pwm_to_rpm(p):
    u = max(-1.0, min(1.0, (p-1500)/500))
    return u, u*n_max

cfd_d1, cfd_d2 = 1.67, 178.59

# TY右移: +TY, T3=T4=-0.5*TY(负CAN,反向PWM), 每桨F/2
print(f"{'TY右移':-^80}")
print(f"{'%':<5s} {'TY':>6s} {'v':>6s} {'F_drag':>7s} {'perCAN':>7s} {'PWM':>7s} {'u_cmd':>7s} {'RPM':>7s} {'T_per':>7s} {'KT':>7s}")
print("-"*80)
ty_r = [(20,2000,0.108),(40,4000,0.163),(60,6000,0.196),(80,8000,0.232),(100,10000,0.280)]
for pct,ty_abs,v in ty_r:
    F = cfd_d1*v + cfd_d2*v**2
    pc = -0.5*ty_abs
    pwm = can_to_pwm(pc); u, rpm = pwm_to_rpm(pwm)
    tp = F/2
    kt = tp/(rhoD4*(abs(rpm)/60)**2) if abs(rpm)>1 else float('nan')
    print(f"{pct}%   {ty_abs:>+6.0f} {v:>6.3f} {F:>7.3f} {pc:>+7.0f} {pwm:>7.1f} {u:>+7.3f} {rpm:>+7.0f} {tp:>7.4f} {kt:>7.4f}")

# TY左移: -TY, T3=T4=-0.5*(-TY)=+0.5*|TY|(正CAN,正向PWM), 每桨F/2
print(f"\n{'TY左移':-^80}")
print(f"{'%':<5s} {'TY':>6s} {'v':>6s} {'F_drag':>7s} {'perCAN':>7s} {'PWM':>7s} {'u_cmd':>7s} {'RPM':>7s} {'T_per':>7s} {'KT':>7s}")
print("-"*80)
ty_l = [(20,-2000,0.049),(40,-4000,0.105),(60,-6000,0.137),(80,-8000,0.160),(100,-10000,0.195)]
for pct,ty_s,v in ty_l:
    F = cfd_d1*v + cfd_d2*v**2  # 正阻力
    pc = -0.5*ty_s               # TY=-2000 → pc=+1000
    pwm = can_to_pwm(pc); u, rpm = pwm_to_rpm(pwm)
    tp = F/2
    kt = tp/(rhoD4*(abs(rpm)/60)**2) if abs(rpm)>1 else float('nan')
    print(f"{pct}%   {ty_s:>+6.0f} {v:>6.3f} {F:>7.3f} {pc:>+7.0f} {pwm:>7.1f} {u:>+7.3f} {rpm:>+7.0f} {tp:>7.4f} {kt:>7.4f}")

# 汇总
print(f"\n{'汇总':-^80}")
print(f"M060 (D=0.06m, rhoD4={rhoD4})")
print(f"  右移(perCAN负→反向PWM): 100% KT={0.7114:.4f}")
print(f"  左移(perCAN正→正向PWM): 100% KT={0.5266:.4f}")
print(f"  代码: KT_fwd=1.050  KT_rev=0.129")
print(f"\n注: M060的per-thruster PWM映射受方向位(0xFB)和CW/CCW安装影响")
print(f"  PWM=1618@20%对所有通道一致, 表明固件对per-thruster CAN做了统一标定")
print(f"  KT值从端到端k系数反推, 非孤立螺旋桨敞水KT")
