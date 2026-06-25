"""
KT 重算 — 简化PWM映射(无死区补偿): CAN-g [-7300,9500] → PWM offset [-430,450]
"""
import numpy as np

rho = 1000.0
n_max = 2500.0
D_main = 0.08
D_aux  = 0.06
rhoD4_main = rho * D_main**4
rhoD4_aux  = rho * D_aux**4

# CFD 阻力系数
cfd_X = (0.65, 10.85)
cfd_Y = (1.67, 178.59)

# PWM 标定 (简化: 直接线性映射, 无死区)
# 正向: CAN 0→9500, offset 0→450. 线性插值: offset = CAN/9500*450
# 反向: CAN -7300→0, offset -430→0. 线性插值: offset = CAN/7300*430
def can_to_pwm_simple(per_can):
    if per_can >= 0:
        return 1500 + per_can/9500 * 450
    else:
        return 1500 + per_can/7300 * 430

# PWM → RPM (无死区, 直接线性)
# 正向: PWM∈[1500,2000] → u_cmd∈[0,1]
# 反向: PWM∈[1000,1500] → u_cmd∈[-1,0]
def pwm_to_rpm_simple(pwm):
    if pwm > 1500:
        u_cmd = (pwm - 1500) / 500   # 0~1
    elif pwm < 1500:
        u_cmd = (pwm - 1500) / 500   # -1~0
    else:
        u_cmd = 0
    return u_cmd, u_cmd * n_max

# ===== TX 前进 (T5单独) =====
print("="*90)
print("TX 前进 — T5 M080 主推 (无死区)")
print(f"{'等级':<6s} {'DOF CAN':>7s} {'v_ss':>7s} {'F_drag':>8s} {'perCAN':>7s} {'PWM':>6s} {'u_cmd':>7s} {'RPM':>7s} {'KT':>8s}")
print("-"*90)

kt_fwd = []
tx_fwd = [(20,2000,0.251),(40,4000,0.592),(60,6000,0.685),(80,8000,0.773),(100,10000,0.870)]
d1,d2 = cfd_X
for pct, can, v in tx_fwd:
    F = d1*v + d2*v**2
    per_can = can          # TX→T5 1:1
    pwm = can_to_pwm_simple(per_can)
    u_cmd, rpm = pwm_to_rpm_simple(min(pwm, 2000))  # clamp to 2000 max
    KT = F / (rhoD4_main * (rpm/60)**2) if abs(rpm)>1 else float('nan')
    kt_fwd.append(KT)
    print(f"{pct}%     {can:>7.0f} {v:>7.3f} {F:>8.3f} {per_can:>7.0f} {pwm:>6.0f} {u_cmd:>7.3f} {rpm:>7.0f} {KT:>8.4f}")

kt_fwd_arr = np.array(kt_fwd)
print(f"\n  KT 非饱和(20-60%)均值: {np.mean(kt_fwd_arr[:3]):.4f}")
print(f"  KT 全部均值:          {np.mean(kt_fwd_arr):.4f}")

# ===== TX 后退 =====
print("\n" + "="*90)
print("TX 后退 — T5 M080 主推 (无死区)")
print(f"{'等级':<6s} {'DOF CAN':>7s} {'v_ss':>7s} {'F_drag':>8s} {'perCAN':>7s} {'PWM':>6s} {'u_cmd':>7s} {'RPM':>7s} {'KT':>8s}")
print("-"*90)

kt_rev = []
tx_rev = [(20,2000,0.197),(40,4000,0.362),(60,6000,0.438),(80,8000,0.488)]
for pct, can, v in tx_rev:
    F = d1*v + d2*v**2
    per_can = can
    pwm = can_to_pwm_simple(per_can)
    u_cmd, rpm = pwm_to_rpm_simple(min(pwm, 2000))
    KT = F / (rhoD4_main * (rpm/60)**2) if abs(rpm)>1 else float('nan')
    kt_rev.append(KT)
    print(f"{pct}%     {can:>7.0f} {v:>7.3f} {F:>8.3f} {per_can:>7.0f} {pwm:>6.0f} {u_cmd:>7.3f} {rpm:>7.0f} {KT:>8.4f}")

kt_rev_arr = np.array(kt_rev)
print(f"\n  KT 非饱和(20-60%)均值: {np.mean(kt_rev_arr[:3]):.4f}")

# ===== TY 右移 (T3+T4共同产生+Y力, TY=+2000 → perCAN=-1000 each) =====
print("\n" + "="*90)
print("TY 右移 — T3+T4 M060 辅推 (无死区)")
print(f"{'等级':<6s} {'TY CAN':>7s} {'v_ss':>7s} {'F_drag':>8s} {'perCAN':>7s} {'PWM':>6s} {'u_cmd':>7s} {'RPM':>7s} {'T_per':>8s} {'KT':>8s}")
print("-"*90)

d1y, d2y = cfd_Y
ty_r = [(20,2000,0.108),(40,4000,0.163),(60,6000,0.196),(80,8000,0.232),(100,10000,0.280)]
for pct, can, v in ty_r:
    F = d1y*v + d2y*v**2
    # T3=T4=-0.5*TY, 各自承担一半Y力
    per_can = -0.5 * can   # 负号: K矩阵 -0.5
    T_per = F / 2
    pwm = can_to_pwm_simple(per_can)
    pwm_clamped = max(1000, min(2000, pwm))
    u_cmd, rpm = pwm_to_rpm_simple(pwm_clamped)
    KT = T_per / (rhoD4_aux * (abs(rpm)/60)**2) if abs(rpm)>1 else float('nan')
    print(f"{pct}%     {can:>7.0f} {v:>7.3f} {F:>8.3f} {per_can:>7.0f} {pwm:>6.0f} {u_cmd:>7.3f} {rpm:>7.0f} {T_per:>8.4f} {KT:>8.4f}")

# ===== TY 左移 (T3+T4共同产生-Y力, TY=-2000 → perCAN=+1000 each) =====
print("\n" + "="*90)
print("TY 左移 — T3+T4 M060 辅推 (无死区)")
print(f"{'等级':<6s} {'TY CAN':>7s} {'v_ss':>7s} {'F_drag':>8s} {'perCAN':>7s} {'PWM':>6s} {'u_cmd':>7s} {'RPM':>7s} {'T_per':>8s} {'KT':>8s}")
print("-"*90)

ty_l = [(20,-2000,0.049),(40,-4000,0.105),(60,-6000,0.137),(80,-8000,0.160),(100,-10000,0.195)]
for pct, can, v in ty_l:
    F = d1y*v + d2y*v**2
    per_can = -0.5 * can   # TY=-2000 → perCAN=+1000 (正)
    T_per = F / 2
    pwm = can_to_pwm_simple(per_can)
    pwm_clamped = max(1000, min(2000, pwm))
    u_cmd, rpm = pwm_to_rpm_simple(pwm_clamped)
    KT = T_per / (rhoD4_aux * (abs(rpm)/60)**2) if abs(rpm)>1 else float('nan')
    print(f"{pct}%     {can:>7.0f} {v:>7.3f} {F:>8.3f} {per_can:>7.0f} {pwm:>6.0f} {u_cmd:>7.3f} {rpm:>7.0f} {T_per:>8.4f} {KT:>8.4f}")

# ===== 汇总 =====
print("\n" + "="*90)
print("汇总")
print("="*90)
print(f"  M080 主推(D=0.08m): KT_fwd={np.mean(kt_fwd_arr[:3]):.4f}  KT_rev={np.mean(kt_rev_arr[:3]):.4f}")
print(f"  代码实际值:          KT_fwd=0.1489       KT_rev=0.0506")
print(f"  差异:                {np.mean(kt_fwd_arr[:3])/0.1489*100:.0f}%                {np.mean(kt_rev_arr[:3])/0.0506*100:.0f}%")
