function cal = xhy_pool_calibration()
%XHY_POOL_CALIBRATION 水池实验标定参数（0616+0601水池实验, CFD阻力反推）
%
% 数据来源: [[推进器推力校准方案_20260621]] §4, 0616水池阶跃实验
%   标定方法: 0616水池阶跃实验(20-80%非饱和段) → CFD阻力(300W)反推k系数
%   动力学方程: m_eff · dv/dt = k · cmd − d1 · v − d2 · v · |v|
%   0601数据仅作参考 (最大速度映射, 单点校准)
%
% 输出 cal 结构体:
%   cal.k              - DOF 级 k 系数 (N/CAN-g 或 N·m/CAN-g)
%   cal.tau_N_to_can_g - 函数句柄: 物理单位 tau_cmd → 固件 CAN-g
%   cal.g_thruster     - 各推进器推力增益 (N per CAN-g)
%   cal.make_*_params  - 生成各推进器标定参数的函数句柄
%
% CAN-g 说明:
%   CAN 指令单位"g"为无量纲值，从下位机固件 K 矩阵分配。
%   k 系数 = 实际物理力(矩) / CAN 指令值，是 K→PWM→推进器→B_thr 的端到端增益。
%   满量程限幅: 正向 +9500, 反向 -7300 CAN-g (固件推力限幅)
%
% 推进器配置:
%   T1/T2 (垂推 M060): CW/CCW 安装，深度通道
%   T3/T4 (侧推 M060): CW/CCW 安装，正反转不对称
%   T5 (主推 M080): K(5,TX)=1.0, 1:1映射
%
% 时间常数 (0616实验 vs CFD线性化, m=85.832kg):
%   Surge(m_eff=101.6): τ_exp≈2.4s, τ_cfd(0.6m/s)≈8.8s (exp快3.7×)
%   Sway(m_eff=210.6):  τ_exp≈2.5s, τ_cfd(0.2m/s)≈1.2s (exp快2×)
%   Yaw(I_eff=6.51):    τ_exp≈1.0s, τ_cfd(0.5rad/s)≈0.4s (exp快2.5×)
%
% 参考:
%   [[水池实验分析_20260601]], [[水池实验分析_20260616]]
%   [[xhy_drag_cfd.m]], [[小黄鱼推进器模型]]

cal = struct();

% ===== DOF 级 k 系数（0616 CFD阻力反推, 20-60%非饱和段） =====
% 单位: N/CAN-g（力通道）或 N·m/CAN-g（力矩通道）
% 符号: 正负来自 NED 体坐标系与 CAN 坐标系的方向定义差异
% 注: 链路1 KT见 thrust_main.m/thrust_aux.m (PWM饱和点反推, 2500RPM)

% --- Surge (TX): 0616 前进非饱和段, 5段 ---
cal.k_X = +0.000888;   % TX→Surge力 (N/CAN-g), σ=±0.00012
                        % 0601参考: 0.000567 (max-speed), KT_fwd=0.0370

% --- Sway (TY): 0616 右移非饱和段, 4段 ---
cal.k_Y = +0.001206;   % TY→Sway力 (N/CAN-g), σ=±0.00005 (高度一致)
                        % 0601参考: 0.002105 (max-speed), KT_fwd=0.388

% --- Heave (TZ): 垂推无独立实验, 复用侧推标定 ---
%   T1+T2 同向, per-thruster k 与侧推相同 (同型号 M060)
%   0616 左移: 0.000583, 右移: 0.001206
%   使用右移值 (正向, 对应下沉推力)
cal.k_Z = -0.001206;   % TZ→Heave力 (N/CAN-g), 负号: 体坐标系Z向下
                        % 0601参考: -0.000343 (IRLS辨识, 耦合严重)

% --- Yaw (MZ): 0616 右转非饱和段, 6段 ---
%   MZ→T3/T4差动, |K|=0.83, B_thr: N=0.424*T3+(-0.376)*T4
%   等效力臂≈0.4m (|0.424|+|-0.376|)/2
cal.k_N = -0.001086;   % MZ→Yaw力矩 (N·m/CAN-g), σ=±0.00015
                        % 0601参考: -0.000124 (IRLS辨识)

% --- Pitch (MY): 无水池激励数据, 从几何关系估算 ---
%   T1/T2 差动, 力臂 = |x_vert_f - x_vert_r| = 0.344-(-0.293)=0.637m
%   K(1,MY)=0.83, K(2,MY)=-0.83
%   用侧推 k 值估算 (需专项 Pitch 测试验证)
cal.k_M = -0.000181;   % MY→Pitch力矩 (N·m/CAN-g), 几何估算, 待验证

% ===== 各推进器推力增益（N per CAN-g） =====
% 推导: DOF 级 k ÷ K 矩阵分配系数, 得到 per-thruster 增益
%
% T5 (主推 M080): K(5,TX)=1.0, TX→T5 1:1
%   g_T5 = |k_X| / 1.0 = 0.000888 N/CAN-g
cal.g_main = 0.000888;  % T5 per-thruster (0616, 前进)
cal.g_main_rev = 0.000361;  % T5 反向 (0616, 后退4段, σ=±0.00005)

% T1/T2 (垂推 M060): K(1,TZ)=K(2,TZ)=-0.5
%   每桨得到 |0.5×TZ| CAN-g, 产生 |k_Z×TZ|/2 = 0.000603×|TZ| N
%   右移标定(正向) k_per = 0.001206/(2×0.5) = 0.001206 N/CAN-g (与k_Y同)
%   左移标定(反向) k_per = 0.000583/(2×0.5) = 0.000583 N/CAN-g
cal.g_vert = 0.001206;      % T1/T2 正向 (下沉, 复用右移)
cal.g_vert_rev = 0.000583;  % T1/T2 反向 (上浮, 复用左移)

% T3/T4 (侧推 M060): K(3,TY)=K(4,TY)=-0.5
%   同理 per-thruster k = cal.k_Y, cal.k_Y_rev
cal.g_side = 0.001206;      % T3/T4 正向 (右移)
cal.g_side_rev = 0.000583;  % T3/T4 反向 (左移)

% ===== N→CAN-g 转换函数句柄 =====
cal.tau_N_to_can_g = @(tau_cmd) tau_N_to_can_g_impl(tau_cmd, cal);

% ===== 推进器标定参数生成 =====
cal.make_m080_params = @(voltage_v) make_m080_calibrated(cal, voltage_v);
cal.make_m060_vert_params = @(voltage_v) make_m060_vert_calibrated(cal, voltage_v);
cal.make_m060_side_params = @(voltage_v) make_m060_side_calibrated(cal, voltage_v);

end

%% ===== 私有函数 =====

function fw_cmd = tau_N_to_can_g_impl(tau_cmd, cal)
%TAU_N_TO_CAN_G_IMPL 控制器物理单位 → 固件 CAN-g
%
% 输入:
%   tau_cmd - 6×1 向量 [X Y Z K M N]' (N, N·m)
% 输出:
%   fw_cmd  - 6×1 向量 [TX TY TZ MX MY MZ]' (CAN-g)

tau_cmd = tau_cmd(:);
if numel(tau_cmd) ~= 6
    error('xhy_pool_calibration:InvalidTau', ...
        'tau_cmd 必须为 6 维向量 [X Y Z K M N]''。');
end

fw_cmd = zeros(6, 1);
fw_cmd(1) = tau_cmd(1) / cal.k_X;    % X(N) → TX(CAN-g)
fw_cmd(2) = tau_cmd(2) / cal.k_Y;    % Y(N) → TY(CAN-g)
fw_cmd(3) = tau_cmd(3) / cal.k_Z;    % Z(N) → TZ(CAN-g)
fw_cmd(4) = 0;                        % K 横滚不可控
fw_cmd(5) = tau_cmd(5) / cal.k_M;    % M(N·m) → MY(CAN-g), 估算值
fw_cmd(6) = tau_cmd(6) / cal.k_N;    % N(N·m) → MZ(CAN-g)
end

function params = make_m080_calibrated(cal, voltage_v)
%MAKE_M080_CALIBRATED 生成 M080 主推水池标定参数
% 参考工作点: 满指令 9500 CAN-g
%   有效推力 = 9500 × 0.000888 = 8.44 N (前进)
%   有效推力 = 9500 × 0.000361 = 3.43 N (后退)
% 厂商模型 24V: 前进 66.69 N, 后退 46.09 N
%   thrust_gain_fwd = 8.44 / 66.69 ≈ 0.127
%   thrust_gain_rev = 3.43 / 46.09 ≈ 0.074
% 注: 100%指令对应饱和区, 非饱和校准外推值偏高
%   仿真中推力由链路2 k系数直接决定, thrust_gain仅缩尺

if nargin < 2, voltage_v = 24.0; end
params = m080_thruster_params();

ref_cmd_g = 9500;
T_fwd = ref_cmd_g * cal.g_main;      % 8.44 N
T_rev = ref_cmd_g * cal.g_main_rev;  % 3.43 N

mfr_fwd_24V = params.forward_thrust_n(end);  % 66.69 N
mfr_rev_24V = params.reverse_thrust_n(end);  % 46.09 N

params.thrust_gain_forward = T_fwd / mfr_fwd_24V;  % 0.127
params.thrust_gain_reverse = T_rev / mfr_rev_24V;  % 0.074
end

function params = make_m060_vert_calibrated(cal, voltage_v)
%MAKE_M060_VERT_CALIBRATED 生成 M060 垂推（T1/T2）水池标定参数
% 参考工作点: 满指令 9500 CAN-g (per-thruster: 4750 CAN-g)
%   T1+T2 同向产生下沉力
%   有效推力(正) = 9500 × 0.001206 = 11.46 N
%   有效推力(反) = 9500 × 0.000583 = 5.54 N
% 厂商模型 24V: 正推 29.42 N, 反推 21.57 N
%   thrust_gain_fwd = 11.46 / 29.42 ≈ 0.389
%   thrust_gain_rev = 5.54 / 21.57 ≈ 0.257

if nargin < 2, voltage_v = 24.0; end
params = m060_thruster_params();

ref_cmd_g = 9500;
T_fwd = ref_cmd_g * cal.g_vert;      % 11.46 N
T_rev = ref_cmd_g * cal.g_vert_rev;  % 5.54 N

mfr_fwd_24V = params.forward_thrust_n(end);  % 29.42 N
mfr_rev_24V = params.reverse_thrust_n(end);  % 21.57 N

params.thrust_gain_forward = T_fwd / mfr_fwd_24V;  % 0.389
params.thrust_gain_reverse = T_rev / mfr_rev_24V;  % 0.257
end

function params = make_m060_side_calibrated(cal, voltage_v)
%MAKE_M060_SIDE_CALIBRATED 生成 M060 侧推（T3/T4）水池标定参数
% 参考工作点: 满指令 9500 CAN-g (per-thruster: 4750 CAN-g)
%   有效推力(正) = 9500 × 0.001206 = 11.46 N (右移)
%   有效推力(反) = 9500 × 0.000583 = 5.54 N (左移)
% 厂商模型 24V: 正推 29.42 N, 反推 21.57 N
%   thrust_gain_fwd = 11.46 / 29.42 ≈ 0.389
%   thrust_gain_rev = 5.54 / 21.57 ≈ 0.257
%
% T3/T4 CW/CCW 安装, 正反向不对称已独立标定

if nargin < 2, voltage_v = 24.0; end
params = m060_thruster_params();

ref_cmd_g = 9500;
T_fwd = ref_cmd_g * cal.g_side;      % 11.46 N
T_rev = ref_cmd_g * cal.g_side_rev;  % 5.54 N

mfr_fwd_24V = params.forward_thrust_n(end);  % 29.42 N
mfr_rev_24V = params.reverse_thrust_n(end);  % 21.57 N

params.thrust_gain_forward = T_fwd / mfr_fwd_24V;  % 0.389
params.thrust_gain_reverse = T_rev / mfr_rev_24V;  % 0.257
end
