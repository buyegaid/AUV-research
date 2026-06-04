function cal = xhy_pool_calibration()
%XHY_POOL_CALIBRATION 水池实验标定参数（2026-06-01 水池实验系统辨识结果）
%
% 数据来源: [[水池实验分析_20260601]] §5 系统辨识结果
%   辨识方法: mode=1 手动操控段 → IRLS 时域灰箱辨识
%   动力学方程: m_eff · dv/dt = k · cmd − d1 · v − d2 · v · |v|
%
% 输出 cal 结构体:
%   cal.k              - DOF 级 k 系数 (N/CAN-g 或 N·m/CAN-g)
%   cal.tau_N_to_can_g  - 函数句柄: 物理单位 tau_cmd → 固件 CAN-g
%   cal.g_thruster      - 各推进器推力增益 (N per CAN-g)
%
% CAN-g 说明:
%   CAN 指令单位"g"为无量纲值，来自下位机固件协议。
%   k 系数 = 实际物理力(矩) / CAN 指令值，是 K矩阵→PWM→推进器→几何 的端到端增益。
%   符号约定: 正负号来自 NED 船体坐标系与 CAN 坐标系的方向定义差异。
%
% 推进器配置说明:
%   T1/T2 (垂推): CW/CCW 安装，深度通道作用对称
%   T3/T4 (侧推): CW/CCW 安装，正反转不对称导致侧移-转艏耦合
%   T5 (主推):   纯前进通道，K 矩阵 1:1 映射
%
% 参考:
%   [[CAN协议说明]]
%   [[水池实验分析_20260601]]

cal = struct();

% ===== DOF 级 k 系数（水池时域辨识） =====
% 单位: N/CAN-g（力通道）或 N·m/CAN-g（力矩通道）
cal.k_X = +0.001282;   % Surge  (TX → N)
cal.k_Y = -0.000843;   % Sway   (TY → N)
cal.k_Z = -0.000343;   % Heave  (TZ → N)
cal.k_N = -0.000124;   % Yaw    (MZ → N·m)

% 俯仰通道: 无水池激励数据，从几何关系估算
% MY 力臂 = |x_vert_f - x_vert_r| = 0.344 - (-0.293) = 0.637 m
% 通过 K 矩阵和垂推增益估算有效俯仰力矩系数
cal.k_M = -0.000181;   % Pitch (MY → N·m)，估算值，需水池验证

% ===== 各推进器推力增益（N per CAN-g） =====
% 推导: 通过 K 矩阵将 DOF 级 k 分解到各推进器
%
% T5 (主推): K(5,1)=1.0, TX → T5 1:1 映射
%   g_T5 = |k_X| = 0.001282 N/CAN-g
cal.g_main = abs(cal.k_X);  % 0.001282

% T1/T2 (垂推): 深度通道对称, 各承担 50%
%   K(1,3)=K(2,3)=-0.5, TZ → T1=T2=-0.5*TZ
%   g_vert = |k_Z| * 0.5 / 0.5 = |k_Z|
cal.g_vert = abs(cal.k_Z);  % 0.000343

% T3/T4 (侧推): 侧移对称, 各承担 50%
%   K(3,2)=K(4,2)=-0.5, TY → T3=T4=-0.5*TY
%   g_side = |k_Y| * 0.5 / 0.5 = |k_Y|
cal.g_side = abs(cal.k_Y);  % 0.000843

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
fw_cmd(1) = tau_cmd(1) / cal.k_X;   % X(N) → TX(CAN-g)
fw_cmd(2) = tau_cmd(2) / cal.k_Y;   % Y(N) → TY(CAN-g)，注意符号由 k_Y 处理
fw_cmd(3) = tau_cmd(3) / cal.k_Z;   % Z(N) → TZ(CAN-g)
fw_cmd(4) = 0;                       % K 横滚不可控
fw_cmd(5) = tau_cmd(5) / cal.k_M;   % M(N·m) → MY(CAN-g)，估算值
fw_cmd(6) = tau_cmd(6) / cal.k_N;   % N(N·m) → MZ(CAN-g)
end

function params = make_m080_calibrated(cal, voltage_v)
%MAKE_M080_CALIBRATED 生成 M080 水池标定参数
% 参考工作点: 满指令 9500 CAN-g → 期望推力 = 9500 * g_main = 12.18 N
% 厂商模型 24V 满指令: 66.69 N → thrust_gain = 12.18 / 66.69 ≈ 0.183

if nargin < 2, voltage_v = 24.0; end

params = m080_thruster_params();

% 计算参考工作点的期望推力
ref_cmd_g = 9500;  % CAN-g 满指令（正向限幅上限）
expected_thrust_N = ref_cmd_g * cal.g_main;  % ≈ 12.18 N

% M080 厂商模型在满指令下的推力
% 正推: 24V → 66.69 N (6.8 kgf)
% 反推: 24V → 46.09 N (4.7 kgf)
mfr_forward_24V = params.forward_thrust_n(end);  % 66.69 N
mfr_reverse_24V = params.reverse_thrust_n(end);  % 46.09 N

params.thrust_gain_forward = expected_thrust_N / mfr_forward_24V;
params.thrust_gain_reverse = expected_thrust_N / mfr_reverse_24V;
% 注: 正向/反向期望推力可能不同（推进器正反转效率差异），
%     当前水池无独立反向数据，暂取相同期望值。
%     若有反向测试数据应分别标定。
end

function params = make_m060_vert_calibrated(cal, voltage_v)
%MAKE_M060_VERT_CALIBRATED 生成 M060 垂推（T1/T2）水池标定参数
% 参考工作点: 满指令 9500 CAN-g → 期望推力 = 9500 * g_vert = 3.26 N
% 厂商模型 24V 满指令: 29.42 N → thrust_gain = 3.26 / 29.42 ≈ 0.111
%
% T1/T2 对称安装（CW/CCW），深度通道作用相同 → 正反向增益取相同值。

if nargin < 2, voltage_v = 24.0; end

params = m060_thruster_params();

ref_cmd_g = 9500;
expected_thrust_N = ref_cmd_g * cal.g_vert;  % ≈ 3.26 N

mfr_forward_24V = params.forward_thrust_n(end);  % 29.42 N
mfr_reverse_24V = params.reverse_thrust_n(end);  % 21.57 N

params.thrust_gain_forward = expected_thrust_N / mfr_forward_24V;
params.thrust_gain_reverse = expected_thrust_N / mfr_reverse_24V;
end

function params = make_m060_side_calibrated(cal, voltage_v)
%MAKE_M060_SIDE_CALIBRATED 生成 M060 侧推（T3/T4）水池标定参数
% 参考工作点: 满指令 9500 CAN-g → 期望推力 = 9500 * g_side = 8.01 N
% 厂商模型 24V 满指令: 29.42 N → thrust_gain = 8.01 / 29.42 ≈ 0.272
%
% T3/T4 CW/CCW 安装 → 正反向不对称（侧移-转艏耦合根源）
% 正反向应分别标定。当前水池无独立反向数据，暂取对称值。
% TODO: 水池反向单轴测试后分别标定

if nargin < 2, voltage_v = 24.0; end

params = m060_thruster_params();

ref_cmd_g = 9500;
expected_thrust_N = ref_cmd_g * cal.g_side;  % ≈ 8.01 N

mfr_forward_24V = params.forward_thrust_n(end);  % 29.42 N
mfr_reverse_24V = params.reverse_thrust_n(end);  % 21.57 N

params.thrust_gain_forward = expected_thrust_N / mfr_forward_24V;
params.thrust_gain_reverse = expected_thrust_N / mfr_reverse_24V;
end
