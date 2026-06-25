"""
KT 推力系数完整重算
链路: 稳态速度 → CFD阻力 → 推力分配反解 → PWM → u_cmd → RPM → KT
"""
import numpy as np

# ========== 1. 物理常数 ==========
rho = 1000.0          # 水密度 kg/m^3
n_max = 2500.0        # 最大转速 RPM
n_max_rps = n_max / 60  # 41.67 rps

# 推进器直径
D_main = 0.08   # M080 主推 8cm
D_aux  = 0.06   # M060 辅推 6cm

rhoD4_main = rho * D_main**4   # 0.04096
rhoD4_aux  = rho * D_aux**4    # 0.01296

# ========== 2. CFD阻力系数 ==========
# xhy_drag_cfd.m, 300W网格
cfd = {
    'X': {'d1': 0.65, 'd2': 10.85, 'unit': 'N',   'desc': 'Surge 纵荡'},
    'Y': {'d1': 1.67, 'd2': 178.59,'unit': 'N',   'desc': 'Sway 横荡'},
    'N': {'d1': 0.2046,'d2': 18.200,'unit': 'N.m', 'desc': 'Yaw 偏航'},
}

# ========== 3. 推力分配矩阵 B_thr (xhy_thruster_geometry.m) ==========
# tau = B_thr * [T1 T2 T3 T4 T5]'
B_thr = np.array([
    [0, 0, 0, 0, 1],                    # X = T5
    [0, 0, 1, 1, 0],                    # Y = T3+T4
    [1, 1, 0, 0, 0],                    # Z = T1+T2
    [0, 0, 0, 0, 0],                    # K = none
    [-0.344, 0.293, 0, 0, 0],           # M = -0.344*T1 + 0.293*T2
    [0, 0, 0.424, -0.376, 0]            # N = 0.424*T3 - 0.376*T4
])

# ========== 4. K矩阵 (CAN-g分配, xhy_force_moment_to_pwm.m) ==========
# per-thruster CAN-g = K * [TX TY TZ MX MY MZ]'
K_can = np.array([
    [0,   0,  -0.5, 0,  0.83,  0     ],   # T1
    [0,   0,  -0.5, 0, -0.83,  0     ],   # T2
    [0,  -0.5, 0,   0,  0,    -0.83  ],   # T3
    [0,  -0.5, 0,   0,  0,     0.83  ],   # T4
    [1.0, 0,   0,   0,  0,     0     ]    # T5
])

# ========== 5. PWM标定表 (固件分段线性插值) ==========
# 正向: per-thruster CAN-g → PWM offset
pos_thrust_g = np.array([0, 3900, 5900, 7900, 9500])
pos_pwm_off  = np.array([0, 230,  310,  370,  450])

# 反向
neg_thrust_g = np.array([-7300, -6100, -4800, -3200, 0])
neg_pwm_off  = np.array([-430,  -370,  -330,  -250,  0])

# PWM死区与中心值
PWM_CENTER = 1500
# 正向死区: PWM∈[1500, 1535] → u_cmd=0; 有效范围 [1535, 1850]
# 反向死区: PWM∈[1485, 1500] → u_cmd=0; 有效范围 [1150, 1485]
# 代码中死区补偿:
# T1:+35/-13, T2:+35/-13, T3:+36/-17, T4:+36/-17, T5:+31/-20
deadzone_pos = np.array([35, 35, 36, 36, 31])
deadzone_neg = np.array([-13, -13, -17, -17, -20])

PWM_MAX  = 1850  # 正向硬限幅
PWM_MIN  = 1150  # 反向硬限幅
PWM_OFFSET_MAX = 350  # PWM偏移限幅

# ========== 6. PWM → u_cmd → RPM ==========
def pwm_to_ucmd_rpm(pwm_us):
    """PWM(us) → u_cmd → RPM, 去除死区"""
    if pwm_us > 1535:
        u_cmd = (pwm_us - 1535) / (1850 - 1535)  # 0~1
    elif pwm_us < 1485:
        u_cmd = -(1485 - pwm_us) / (1485 - 1150)  # -1~0
    else:
        u_cmd = 0  # 死区
    rpm = n_max * u_cmd
    return u_cmd, rpm

def can_g_to_pwm_offset(per_thruster_can_g, thr_idx):
    """per-thruster CAN-g → PWM offset (含死区补偿)"""
    T = per_thruster_can_g
    if abs(T) < 1e-6:
        return 0.0
    elif T > 0:
        raw = np.interp(T, pos_thrust_g, pos_pwm_off)
        return raw + deadzone_pos[thr_idx]
    else:
        raw = np.interp(T, neg_thrust_g, neg_pwm_off)
        return raw + deadzone_neg[thr_idx]

# ========== 7. 各工况计算 ==========

# 0616实验稳态数据 (每推力等级取一代表值)
# TX前进 (T5单独工作, T3/T4=0)
tx_fwd_cases = [
    # pct, DOF_CAN_g(TX), v_ss(m/s), 文档PWM
    ('20%',  2000,  0.251, 1618),
    ('40%',  4000,  0.592, 1734),
    ('60%',  6000,  0.685, 1813),
    ('80%',  8000,  0.773, 1875),
    ('100%', 10000, 0.870, 1950),
]

# TX后退
tx_rev_cases = [
    ('20%',  2000, 0.197, 1618),
    ('40%',  4000, 0.362, 1734),
    ('60%',  6000, 0.438, 1813),
    ('80%',  8000, 0.488, 1875),
]

# TY右移 (T3+T4同向)
# 注意: TY = 2000, K矩阵 T3=-0.5*2000=-1000, T4=-0.5*2000=-1000
# 负的CAN-g表示反向推力, 在体坐标系中产生+Y力
ty_right_cases = [
    ('20%',  2000,  0.108, 1618),
    ('40%',  4000,  0.163, 1734),
    ('60%',  6000,  0.196, 1813),
    ('80%',  8000,  0.232, 1875),
    ('100%', 10000, 0.280, 1950),
]

# TY左移 (T3+T4同向, 负TY)
ty_left_cases = [
    ('20%',  -2000,  0.049, 1618),
    ('40%',  -4000,  0.105, 1734),
    ('60%',  -6000,  0.137, 1813),
    ('80%',  -8000,  0.160, 1875),
    ('100%', -10000, 0.195, 1950),
]

print("=" * 100)
print("KT 推力系数完整计算 (D_main=0.08m, D_aux=0.06m)")
print("=" * 100)

def calc_kt_surge(cases, cfd_ch, thr_idx, direction_desc):
    """纵荡通道: T5单独"""
    print(f"\n{'─'*80}")
    print(f"  {direction_desc} (T5 thr_idx={thr_idx})")
    print(f"  {'推力等级':<8s} {'DOF CAN-g':>10s} {'v_ss':>8s} {'F_drag':>10s} {'PWM':>6s} {'u_cmd':>7s} {'RPM':>7s} {'rhoD4':>10s} {'KT':>10s}")
    print(f"  {'─'*80}")

    kt_values = []
    d1, d2 = cfd_ch['d1'], cfd_ch['d2']

    for pct, can_g, v_ss, pwm_doc in cases:
        # Step 1: CFD阻力
        F_drag = d1 * v_ss + d2 * v_ss**2

        # Step 2: 推力分配 — T5推力 = 前进力 (B_thr[0,4]=1)
        T_thruster = F_drag  # 对于纯前进, T5单独产生X力

        # Step 3: per-thruster CAN-g
        # DOF CAN-g = TX, K[4,0] = 1.0, so per-thruster CAN-g = TX
        per_can = can_g

        # Step 4: CAN-g → PWM
        pwm_offset = can_g_to_pwm_offset(per_can, thr_idx)
        pwm_offset = max(-PWM_OFFSET_MAX, min(PWM_OFFSET_MAX, pwm_offset))
        pwm_calc = PWM_CENTER + pwm_offset

        # Step 5: PWM → u_cmd → RPM
        u_cmd, rpm = pwm_to_ucmd_rpm(pwm_calc)

        # Step 6: KT
        if abs(rpm) > 1:
            KT = T_thruster / (rhoD4_main * (rpm/60)**2)
            kt_values.append(KT)
        else:
            KT = float('nan')

        print(f"  {pct:<8s} {can_g:>10.0f} {v_ss:>8.3f} {F_drag:>10.3f} {pwm_calc:>6.0f} {u_cmd:>7.3f} {rpm:>7.0f} {rhoD4_main:>10.5f} {KT:>10.4f}")

    if kt_values:
        print(f"\n  KT均值(非饱和): {np.mean(kt_values[:3]):.4f}  (20-60%段)")
        print(f"  KT均值(全部):   {np.mean(kt_values):.4f}")
    return kt_values


def calc_kt_sway(cases, cfd_ch, direction_desc, sign):
    """横荡通道: T3+T4同向, per-thruster各承担一半"""
    print(f"\n{'─'*80}")
    print(f"  {direction_desc} (T3+T4同时{'正转' if sign>0 else '反转'})")
    print(f"  {'推力等级':<8s} {'DOF CAN-g':>10s} {'v_ss':>8s} {'F_drag':>10s} {'T_per':>10s} {'per CAN':>8s} {'PWM 3':>6s} {'PWM 4':>6s} {'u_cmd_3':>7s} {'RPM':>7s}")
    print(f"  {'─'*80}")

    d1, d2 = cfd_ch['d1'], cfd_ch['d2']

    for pct, can_g, v_ss, pwm_doc in cases:
        # Step 1: CFD阻力
        F_drag = d1 * v_ss + d2 * v_ss**2

        # Step 2: 推力分配 — T3+T4共同产生Y力, 各一半
        T_per = F_drag / 2  # T3和T4各承担一半

        # Step 3: DOF CAN-g → per-thruster CAN-g
        # fw_cmd = [TX=0, TY=can_g, TZ=0, MX=0, MY=0, MZ=0]
        # T3 = -0.5*TY, T4 = -0.5*TY
        fw = np.array([0, can_g, 0, 0, 0, 0])
        per_thrust = K_can @ fw  # [T1 T2 T3 T4 T5]
        per_can_3 = per_thrust[2]  # T3
        per_can_4 = per_thrust[3]  # T4

        # Step 4: CAN-g → PWM
        pwm_off_3 = can_g_to_pwm_offset(per_can_3, 2)
        pwm_off_4 = can_g_to_pwm_offset(per_can_4, 3)
        pwm_off_3 = max(-PWM_OFFSET_MAX, min(PWM_OFFSET_MAX, pwm_off_3))
        pwm_off_4 = max(-PWM_OFFSET_MAX, min(PWM_OFFSET_MAX, pwm_off_4))
        pwm_3 = PWM_CENTER + pwm_off_3
        pwm_4 = PWM_CENTER + pwm_off_4

        # Step 5: PWM → u_cmd → RPM (取T3为参考)
        u_cmd, rpm = pwm_to_ucmd_rpm(pwm_3)

        # Step 6: KT
        if abs(rpm) > 1:
            KT = abs(T_per) / (rhoD4_aux * (abs(rpm)/60)**2)
        else:
            KT = float('nan')

        print(f"  {pct:<8s} {can_g:>10.0f} {v_ss:>8.3f} {F_drag:>10.3f} {abs(T_per):>10.4f} {per_can_3:>8.0f} {pwm_3:>6.0f} {pwm_4:>6.0f} {u_cmd:>7.3f} {rpm:>7.0f}  KT={KT:.4f}")

    return


# ===== 执行 =====
print("\n" + "=" * 100)
print("TX 前进 (Surge Forward) — T5 M080 主推")
print("=" * 100)
kt_tx_fwd = calc_kt_surge(tx_fwd_cases, cfd['X'], 4, "TX前进")

print("\n" + "=" * 100)
print("TX 后退 (Surge Reverse) — T5 M080 主推")
print("=" * 100)
kt_tx_rev = calc_kt_surge(tx_rev_cases, cfd['X'], 4, "TX后退")

print("\n" + "=" * 100)
print("TY 右移 (Sway Starboard) — T3+T4 M060 辅推")
print("=" * 100)
calc_kt_sway(ty_right_cases, cfd['Y'], "TY右移", +1)

print("\n" + "=" * 100)
print("TY 左移 (Sway Port) — T3+T4 M060 辅推")
print("=" * 100)
calc_kt_sway(ty_left_cases, cfd['Y'], "TY左移", -1)

# ===== 总结 =====
print("\n" + "=" * 100)
print("计算结果汇总")
print("=" * 100)
print(f"\n  主推 M080 (D={D_main}m):")
print(f"    前进 KT_fwd ≈ {np.mean(kt_tx_fwd[:3]):.4f} (非饱和段均值)")
print(f"    后退 KT_rev ≈ {np.mean(kt_tx_rev[:3]):.4f} (非饱和段均值)")
print(f"\n  辅推 M060 (D={D_aux}m): 从TY右移/左移数据反算")
print(f"    (见上表T3/T4各推力等级的KT值)")
