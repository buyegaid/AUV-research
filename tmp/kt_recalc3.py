"""
KT 重算 — 分段PWM映射(§2.3)，无死区
PWM分段: per-CAN[-7300,-6100,-4800,-3200,0,3900,5900,7900,9500]
       → offset[-430,-370,-330,-250,0,230,310,370,450]
PWM=1500+offset, u_cmd=(PWM-1500)/500, RPM=u_cmd*2500
"""
import numpy as np

rho = 1000.0; n_max = 2500.0
D_main = 0.08; D_aux = 0.06
rhoD4_main = rho * D_main**4
rhoD4_aux  = rho * D_aux**4

# CFD
cfd_X = (0.65, 10.85)
cfd_Y = (1.67, 178.59)

# §2.3 分段PWM标定表 (per-thruster)
CAN_TABLE = np.array([-7300, -6100, -4800, -3200, 0, 3900, 5900, 7900, 9500])
OFF_TABLE = np.array([-430,  -370,  -330,  -250,  0, 230,  310,  370,  450])

def can_to_pwm(per_can):
    """per-thruster CAN-g → PWM (无死区, 分段线性)"""
    off = np.interp(per_can, CAN_TABLE, OFF_TABLE)
    return 1500 + off

def pwm_to_rpm(pwm):
    """PWM → RPM (无死区, 线性 1000-2000)"""
    u = (pwm - 1500) / 500
    u = max(-1.0, min(1.0, u))
    return u, u * n_max

# ===== TX 前进 (T5 M080) =====
print("="*85)
print("TX 前进 — T5 M080 (分段PWM, 无死区)")
print(f"{'等级':<5s} {'DOF CAN':>7s} {'v_ss':>6s} {'F_drag':>7s} {'perCAN':>7s} {'PWM':>6s} {'u_cmd':>6s} {'RPM':>6s} {'KT':>7s}")
print("-"*85)

kt_fwd = []
tx_fwd = [(20,2000,0.251),(40,4000,0.592),(60,6000,0.685),(80,8000,0.773),(100,10000,0.870)]
d1,d2 = cfd_X
for pct, can, v in tx_fwd:
    F = d1*v + d2*v**2
    per_can = can
    pwm = can_to_pwm(per_can)
    u_cmd, rpm = pwm_to_rpm(pwm)
    KT = F/(rhoD4_main*(rpm/60)**2) if abs(rpm)>1 else float('nan')
    kt_fwd.append(KT)
    print(f" {pct}%    {can:>7.0f} {v:>6.3f} {F:>7.3f} {per_can:>7.0f} {pwm:>6.1f} {u_cmd:>6.3f} {rpm:>6.0f} {KT:>7.4f}")

kt_fwd = np.array(kt_fwd)
print(f"\n  KT 非饱和(20-60%): {np.mean(kt_fwd[:3]):.4f}")
print(f"  KT 饱和点(100%):   {kt_fwd[4]:.4f}")

# ===== TX 后退 =====
print("\n" + "="*85)
print("TX 后退 — T5 M080 (分段PWM, 无死区)")
print(f"{'等级':<5s} {'DOF CAN':>7s} {'v_ss':>6s} {'F_drag':>7s} {'perCAN':>7s} {'PWM':>6s} {'u_cmd':>6s} {'RPM':>6s} {'KT':>7s}")
print("-"*85)

kt_rev = []
tx_rev = [(20,2000,0.197),(40,4000,0.362),(60,6000,0.438),(80,8000,0.488)]
for pct, can, v in tx_rev:
    F = d1*v + d2*v**2
    per_can = can
    pwm = can_to_pwm(per_can)
    u_cmd, rpm = pwm_to_rpm(pwm)
    KT = F/(rhoD4_main*(rpm/60)**2) if abs(rpm)>1 else float('nan')
    kt_rev.append(KT)
    print(f" {pct}%    {can:>7.0f} {v:>6.3f} {F:>7.3f} {per_can:>7.0f} {pwm:>6.1f} {u_cmd:>6.3f} {rpm:>6.0f} {KT:>7.4f}")

kt_rev = np.array(kt_rev)
print(f"\n  KT 非饱和(20-60%): {np.mean(kt_rev[:3]):.4f}")

# ===== TY 右移 (T3+T4, 反向CAN-g) =====
print("\n" + "="*85)
print("TY 右移 — T3+T4 M060 (分段PWM, 无死区)")
print(f"{'等级':<5s} {'TY':>6s} {'v_ss':>6s} {'F_drag':>7s} {'perCAN':>7s} {'PWM':>6s} {'u_cmd':>6s} {'RPM':>6s} {'T_per':>7s} {'KT':>7s}")
print("-"*85)

d1y,d2y = cfd_Y
ty_r = [(20,2000,0.108),(40,4000,0.163),(60,6000,0.196),(80,8000,0.232),(100,10000,0.280)]
for pct, can, v in ty_r:
    F = d1y*v + d2y*v**2
    per_can = -0.5*can    # K矩阵 T3=-0.5*TY
    T_per = F/2
    pwm = can_to_pwm(per_can)
    u_cmd, rpm = pwm_to_rpm(pwm)
    KT = T_per/(rhoD4_aux*(abs(rpm)/60)**2) if abs(rpm)>1 else float('nan')
    print(f" {pct}%   {can:>6.0f} {v:>6.3f} {F:>7.3f} {per_can:>7.0f} {pwm:>6.1f} {u_cmd:>6.3f} {rpm:>6.0f} {T_per:>7.4f} {KT:>7.4f}")

# ===== TY 左移 (T3+T4, 正向CAN-g) =====
print("\n" + "="*85)
print("TY 左移 — T3+T4 M060 (分段PWM, 无死区)")
print(f"{'等级':<5s} {'TY':>6s} {'v_ss':>6s} {'F_drag':>7s} {'perCAN':>7s} {'PWM':>6s} {'u_cmd':>6s} {'RPM':>6s} {'T_per':>7s} {'KT':>7s}")
print("-"*85)

ty_l = [(20,-2000,0.049),(40,-4000,0.105),(60,-6000,0.137),(80,-8000,0.160),(100,-10000,0.195)]
for pct, can, v in ty_l:
    F = d1y*v + d2y*v**2
    per_can = -0.5*can    # TY=-2000 → perCAN=+1000
    T_per = F/2
    pwm = can_to_pwm(per_can)
    u_cmd, rpm = pwm_to_rpm(pwm)
    KT = T_per/(rhoD4_aux*(abs(rpm)/60)**2) if abs(rpm)>1 else float('nan')
    print(f" {pct}%   {can:>6.0f} {v:>6.3f} {F:>7.3f} {per_can:>7.0f} {pwm:>6.1f} {u_cmd:>6.3f} {rpm:>6.0f} {T_per:>7.4f} {KT:>7.4f}")

# ===== 汇总 =====
print("\n" + "="*85)
print("汇总")
print("="*85)
print(f"  M080 (D={D_main}m):")
print(f"    KT_fwd = {np.mean(kt_fwd[:3]):.4f} (非饱和)  {kt_fwd[4]:.4f} (100%)  code=0.149")
print(f"    KT_rev = {np.mean(kt_rev[:3]):.4f} (非饱和)  code=0.051")
