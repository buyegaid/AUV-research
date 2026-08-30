%% XHY AUV 仿真主入口
% 控制架构: 3D ALOS制导 → 单通道SMC(航向/深度/速度) → 推力分配 → XHY动力学
% DepthMode = 1: 俯仰角控制深度（ALOS给theta_ref → SMC pitch → M_cmd）
% DepthMode = 2: 直接Z力控制深度（深度误差 → SMC heave → Z_cmd）
clear; close all; clc;
clear smc_yaw_xhy smc_pitch_xhy smc_surge_xhy smc_heave_xhy my_ALOS3D vec_leso_update_adv vec_pieso_update

%% 宏参数
useESO    = 0;   % 是否使用ESO补偿
usePIESO  = 0;   % 是否使用物理信息ESO（PI-ESO，Gauss-Markov海流模型）
TrajMode  = 1;   % 1-直线, 2-圆形
DepthMode = 2;   % 1-俯仰控深, 2-直接Z力控深
ThrMode   = 1;   % 1-数字孪生(PWM+M080/M060+水池标定), 2-传统(RPM+KT系数)
params    = get_params;

h = 0.01;
T = 1000;
t = 0:h:T;
N = length(t);

%% 初始状态
xn = 0; yn = 0; zn = 10; % 初始位置
psi0 = 0; % 艏向
x = [1; 0; 0; 0; 0; 0; xn; yn; zn; 0; 0; psi0];  % [u v w p q r x y z phi theta psi]

%% 参考量初始化
theta_d = 0; q_d = 0;
psi_d   = psi0; r_d = 0;
u_d     = 1;    u_d_dot = 0;
z_d     = zn;   w_d = 0;  w_d_dot = 0;  % 深度参考（Mode 2用）
x_d = xn;
y_d = yn;
%% 海流
Vc     = params.current.Vc;
betaVc = params.current.betaVc;
wc     = params.current.wc;

%% 传统 RPM 推力分配参数（ThrMode=2 向后兼容路径）
thr_params.rho         = 1026;
thr_params.D_prop_main = 0.10;      % 主推直径10cm
thr_params.D_prop_aux  = 0.06;      % 辅推直径6cm
thr_params.KT_main_fwd = 0.0293;     % 主推正向KT（CFD阻力校准 2026-06-03）
thr_params.KT_main_rev = 0.0201;     % 主推反向KT（同比缩放）
thr_params.KT_aux_fwd  = 0.327;      % 辅推正向KT（CFD阻力校准 2026-06-03）
thr_params.KT_aux_rev  = 0.327;      % 辅推反向KT（无反向数据，暂取同正向）
thr_params.n_max   = 2500;
thr_params.x_vert_f = +0.344;
thr_params.x_vert_r = -0.293;
thr_params.x_side_f = +0.424;
thr_params.x_side_r = -0.376;

%% 数字孪生推进器配置（ThrMode=1）
if ThrMode == 1
    % 加载水池标定参数
    cal = xhy_pool_calibration();

    % 生成各推进器水池标定参数（24V供电）
    p_m080_main = cal.make_m080_params(24.0);
    p_m060_vert = cal.make_m060_vert_params(24.0);
    p_m060_side = cal.make_m060_side_params(24.0);

    % xhy.m PWM模式的配置
    opt_xhy.mode = 'pwm';
    opt_xhy.thruster_params = {p_m060_vert, p_m060_vert, p_m060_side, p_m060_side, p_m080_main};
    opt_xhy.voltage_v = 24.0;

    % 推力分配参数（xhy_force_moment_to_pwm 使用）
    alloc_params = struct();
    alloc_params.pwm_prev_doc = zeros(5,1);  % 初始无上一周期PWM

    fprintf('推进器标定增益:\n');
    fprintf('  M080 T5主推:   fwd=%.4f, rev=%.4f\n', ...
        p_m080_main.thrust_gain_forward, p_m080_main.thrust_gain_reverse);
    fprintf('  M060 T1/T2垂推: fwd=%.4f, rev=%.4f\n', ...
        p_m060_vert.thrust_gain_forward, p_m060_vert.thrust_gain_reverse);
    fprintf('  M060 T3/T4侧推: fwd=%.4f, rev=%.4f\n', ...
        p_m060_side.thrust_gain_forward, p_m060_side.thrust_gain_reverse);
else
    opt_xhy = struct('mode', 'rpm');
end

%% ESO状态
Z = zeros(6, 3);

%% 轨迹
if TrajMode == 1
    pts = line_traj([x_d; y_d; z_d], 0, 50, 50);
else
    pts = traj(50, 50);
end
z_d = pts.pos.z(1);  % 目标深度（NED，向下为正），循环外初始化

%% 历史数据
hist.t    = t;
hist.x    = zeros(N, 12);
hist.ui   = zeros(N, 5);
hist.tau  = zeros(N, 6);
hist.xd   = zeros(N, 4);   % [psi_d theta_d psi_ref theta_ref]
hist.traj = pts;

ui = zeros(5,1);
timebar(1, N, 'XHY仿真');

%% 主循环
for i = 1:N
    % 状态提取
    u     = x(1);  w = x(3);  q = x(5);  r = x(6);
    xn    = x(7);  yn = x(8); zn = x(9);
    theta = x(11); psi = x(12);

    % 动力学矩阵（用于ESO的已知加速度）
    [~, ~, M, C, D, g_vec, tau_thr] = xhy(x, ui, Vc, betaVc, wc, opt_xhy);

    % 计算相对速度（去除海流分量）
    u_c_x = Vc * cos(betaVc - psi);
    u_c_y = Vc * sin(betaVc - psi);
    nu_c  = [u_c_x; u_c_y; wc; 0; 0; 0];
    nu_r  = x(1:6) - nu_c;

    % ESO更新：已知加速度 = M^{-1}*(tau - C*nu_r - D*nu_r - g)
    a_known = M \ (tau_thr - C*nu_r - D*nu_r - g_vec);
    if usePIESO
        [Z, ~] = vec_pieso_update(Z, x(1:6), a_known, params.pieso, h);
    else
        [Z, ~] = vec_leso_update_adv(Z, x(1:6), a_known, params.eso, h);
    end

    % ESO扰动估计（加速度单位 → 力/力矩单位）
    if useESO
        hat_d = M * Z(:, 3);
    else
        hat_d = zeros(6, 1);
    end

    % 制导
    [psi_ref, theta_ref, ~, ~, ~, ~, ~] = my_ALOS3D(xn, yn, zn, h, pts, params.alos);
    [psi_d, r_d] = LOSobserver(psi_d, r_d, psi_ref, h, params.alos.K_f);
    r_d = sat(r_d, 0.5);

    % 深度控制（两种模式）
    if DepthMode == 1
        % 俯仰角控深：ALOS给theta_ref，SMC pitch输出俯仰力矩M
        [theta_d, q_d] = LOSobserver(theta_d, q_d, theta_ref, h, params.alos.K_f);
        q_d = sat(q_d, 0.5);
        M_cmd = smc_pitch_xhy(theta, q, theta_d, q_d, 0, h, params.xhy.pitch) - hat_d(5);
        Z_cmd = 0;
    else
        % 直接Z力控深：深度误差→SMC heave输出沉浮力Z，俯仰保持水平
        Z_cmd = smc_heave_xhy(zn, w, z_d, w_d, w_d_dot, h, params.xhy.heave) - hat_d(3);
        M_cmd = 0;
    end

    % 单通道SMC控制律
    X_cmd = smc_surge_xhy(u, u_d, u_d_dot, h, params.xhy.surge) - hat_d(1);
    N_cmd = smc_yaw_xhy(psi, r, psi_d, r_d, 0, h, params.xhy.yaw)   - hat_d(6);

    % 组装6-DOF力/力矩指令 [X Y Z K M N]
    tau_cmd = [X_cmd; 0; Z_cmd; 0; M_cmd; N_cmd];

    % 推力分配（数字孪生: N→CAN-g→K矩阵→PWM→M080/M060）
    if ThrMode == 1
        fw_cmd = cal.tau_N_to_can_g(tau_cmd);
        [pwm_us_doc, ~] = xhy_force_moment_to_pwm(fw_cmd, alloc_params, alloc_params.pwm_prev_doc);
        ui = pwm_us_doc;  % 统一顺序 [T1 T2 T3 T4 T5]
        alloc_params.pwm_prev_doc = pwm_us_doc;
    else
        [ui, ~] = thrust_allocation_xhy(tau_cmd, thr_params);
    end

    % 存储
    hist.x(i,:)  = x';
    hist.ui(i,:) = ui';
    hist.tau(i,:) = tau_cmd';
    hist.xd(i,:) = [psi_d, theta_d, psi_ref, theta_ref];

    % 状态更新
    x     = rk4(@xhy, h, x, ui, Vc, betaVc, wc, opt_xhy);
    x(12) = ssa(x(12));

    timebar;
end

%% 绘图
plot_xhy_results(hist);
