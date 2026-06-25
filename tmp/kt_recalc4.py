"""
KT 重算 — 分段PWM, 正反向独立映射, 无死区
CAN符号: 前进=+CAN, 后退=-CAN (K矩阵1:1映射到T5)
         TY右移=+TY→perCAN=-0.5*TY(反向PWM)
         TY左移=-TY→perCAN=+0.5*|TY|(正向PWM)
"""
import numpy as np

rho = 1000.0; n_max = 2500.0
D_main = 0.08; D_aux = 0.06
rhoD4_main = rho * D_main**4   # 0.04096
rhoD4_aux  = rho * D_aux**4    # 0.01296

# CFD
cfd_X = (0.65, 10.85); cfd_Y = (1.67, 178.59)

# §2.3 分段PWM表
CAN_POS = np.array([0, 3900, 5900, 7900, 9500])
OFF_POS = np.array([0, 230,  310,  370,  450])
CAN_NEG = np.array([-7300, -6100, -4800, -3200, 0])
OFF_NEG = np.array([-430,  -370,  -330,  -250,  0])

def can_to_pwm(per_can):
    """per-thruster CAN-g → PWM, 正反向独立分段插值"""
    if per_can >= 0:
        off = np.interp(per_can, CAN_POS, OFF_POS)
    else:
        off = np.interp(per_can, CAN_NEG, OFF_NEG)
    return 1500 + off

def pwm_to_rpm(pwm):
    """PWM → RPM (无死区 1000-2000us → ±2500RPM)"""
    u = (pwm - 1500) / 500
    u = max(-1.0, min(1.0, u))
    return u, u * n_max

# ===== TX 前进: 正CAN, 正PWM, 正RPM =====
print("="*90)
print("TX 前进 (正CAN → 正向PWM)")
print(f"{'等级':<5s} {'CAN':>6s} {'v(m/s)':>7s} {'F_drag':>7s} {'perCAN':>7s} {'PWM':>7s} {'u_cmd':>7s} {'RPM':>7s} {'KT_fwd':>8s}")
print("-"*90)

kt_fwd_vals = []
d1,d2 = cfd_X
cases_fwd = [(20,2000,0.251),(40,4000,0.592),(60,6000,0.685),(80,8000,0.773),(100,10000,0.870)]
for pct, can_abs, v_abs in cases_fwd:
    F = d1*v_abs + d2*v_abs**2
    per_can = +can_abs          # TX前进: 正CAN
    pwm = can_to_pwm(per_can)
    u_cmd, rpm = pwm_to_rpm(pwm)
    KT = F/(rhoD4_main*(rpm/60)**2) if abs(rpm)>1 else float('nan')
    kt_fwd_vals.append(KT)
    print(f" {pct}%    {can_abs:>+6.0f} {v_abs:>7.3f} {F:>7.3f} {per_can:>+7.0f} {pwm:>7.1f} {u_cmd:>+7.3f} {rpm:>+7.0f} {KT:>8.4f}")

# ===== TX 后退: 负CAN, 负PWM, 负RPM =====
print("\n" + "="*90)
print("TX 后退 (负CAN → 反向PWM)")
print(f"{'等级':<5s} {'CAN':>6s} {'v(m/s)':>7s} {'F_drag':>7s} {'perCAN':>7s} {'PWM':>7s} {'u_cmd':>7s} {'RPM':>7s} {'KT_rev':>8s}")
print("-"*90)

kt_rev_vals = []
cases_rev = [(20,-2000,0.197),(40,-4000,0.362),(60,-6000,0.438),(80,-8000,0.488)]
for pct, can_signed, v_abs in cases_rev:
    F = -(d1*v_abs + d2*v_abs**2)  # 负力(后退)
    per_can = can_signed            # TX后退: 负CAN (K矩阵1:1)
    pwm = can_to_pwm(per_can)
    u_cmd, rpm = pwm_to_rpm(pwm)
    KT = abs(F)/(rhoD4_main*(abs(rpm)/60)**2) if abs(rpm)>1 else float('nan')
    kt_rev_vals.append(KT)
    print(f" {pct}%    {can_signed:>+6.0f} {v_abs:>7.3f} {F:>7.3f} {per_can:>+7.0f} {pwm:>7.1f} {u_cmd:>+7.3f} {rpm:>+7.0f} {KT:>8.4f}")

# ===== TY 右移: +TY → perCAN=-0.5*TY(负) → 反向PWM =====
print("\n" + "="*90)
print("TY 右移 (+TY → perCAN负 → 反向PWM)")
print(f"{'等级':<5s} {'TY':>6s} {'v(m/s)':>7s} {'F_drag':>7s} {'perCAN':>7s} {'PWM':>7s} {'u_cmd':>7s} {'RPM':>7s} {'T_per':>7s} {'KT':>8s}")
print("-"*90)

d1y,d2y = cfd_Y
cases_tyr = [(20,2000,0.108),(40,4000,0.163),(60,6000,0.196),(80,8000,0.232),(100,10000,0.280)]
for pct, ty_abs, v_abs in cases_tyr:
    F = d1y*v_abs + d2y*v_abs**2
    per_can = -0.5 * ty_abs      # T3/T4 负CAN (反向)
    T_per = F/2                  # 每桨承担一半
    pwm = can_to_pwm(per_can)
    u_cmd, rpm = pwm_to_rpm(pwm)
    KT = T_per/(rhoD4_aux*(abs(rpm)/60)**2) if abs(rpm)>1 else float('nan')
    print(f" {pct}%   {ty_abs:>+6.0f} {v_abs:>7.3f} {F:>7.3f} {per_can:>+7.0f} {pwm:>7.1f} {u_cmd:>+7.3f} {rpm:>+7.0f} {T_per:>7.4f} {KT:>8.4f}")

# ===== TY 左移: -TY → perCAN=-0.5*(-TY)=+0.5*|TY|(正) → 正向PWM =====
print("\n" + "="*90)
print("TY 左移 (-TY → perCAN正 → 正向PWM)")
print(f"{'等级':<5s} {'TY':>6s} {'v(m/s)':>7s} {'F_drag':>7s} {'perCAN':>7s} {'PWM':>7s} {'u_cmd':>7s} {'RPM':>7s} {'T_per':>7s} {'KT':>8s}")
print("-"*90)

cases_tyl = [(20,-2000,0.049),(40,-4000,0.105),(60,-6000,0.137),(80,-8000,0.160),(100,-10000,0.195)]
for pct, ty_signed, v_abs in cases_tyl:
    F = -(d1y*v_abs + d2y*v_abs**2)  # 负力(左移)
    per_can = -0.5 * ty_signed   # = +0.5*|TY| → 正CAN (正向PWM)
    T_per = abs(F)/2
    pwm = can_to_pwm(per_can)
    u_cmd, rpm = pwm_to_rpm(pwm)
    KT = T_per/(rhoD4_aux*(abs(rpm)/60)**2) if abs(rpm)>1 else float('nan')
    print(f" {pct}%   {ty_signed:>+6.0f} {v_abs:>7.3f} {F:>7.3f} {per_can:>+7.0f} {pwm:>7.1f} {u_cmd:>+7.3f} {rpm:>+7.0f} {T_per:>7.4f} {KT:>8.4f}")

# ===== 汇总 =====
print("\n" + "="*90)
print("汇总对比")
print("="*90)
kf = np.array(kt_fwd_vals); kr = np.array(kt_rev_vals)
print(f"  M080 (D={D_main}m):")
print(f"    KT_fwd = {kf[4]:.4f} @100% (code=0.149, diff={abs(kf[4]-0.149)/0.149*100:.0f}%)")
print(f"    KT_rev = {kr[-1]:.4f} @80%  (code=0.051, diff={abs(kr[-1]-0.051)/0.051*100:.0f}%)")
print(f"    非饱和均值: fwd={np.mean(kf[:3]):.4f}  rev={np.mean(kr[:3]):.4f}")
