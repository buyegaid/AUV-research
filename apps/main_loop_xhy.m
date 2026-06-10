function hist = main_loop_xhy(useESO, TrajMode, CurrentModel, params)
% MAIN_LOOP_XHY 使用XHY模型和6-DOF SMC+ESO的主仿真循环
%
%   hist = main_loop_xhy(useESO, TrajMode, CurrentModel, params)
%
%   输入:
%     useESO:        是否使用ESO补偿 (0/1)
%     TrajMode:      轨迹模式 (1-直线, 2-圆形)
%     CurrentModel:  海流模型 (1-静态, 2-动态)
%     params:        参数结构体 (来自get_params)
%
%   输出:
%     hist:          历史数据结构体

clear my_ALOS3D
clear vec_leso_update_adv
clear smc_6dof

%% 参数设置
T = 500;                % 仿真时间 (s)
h = 0.01;               % 时间步长 (s)
t = 0:h:T;
nTimeSteps = length(t);

% 初始状态
xn = 300; yn = 0; zn = 10;
phi = 0; theta = 0; psi = -pi/2;
U = 1;
x = [U; zeros(5,1); xn; yn; zn; phi; theta; psi]; % 12维状态

% 期望速度/姿态初始化
nu_d = [1; 0; 0; 0; 0; 0];      % 期望速度 [u v w p q r]'
nu_d_dot = zeros(6,1);          % 期望加速度
theta_d = 0; q_d = 0;
psi_d = psi; r_d = 0;

% 海流参数
Vc = params.current.Vc;
betaVc = params.current.betaVc;
wc = params.current.wc;

% 推进器初始化
ui = [0; 0; 0; 0; 600];  % [T1 T2 T3 T4 T5]' (RPM)
n_max = 2500;
r_max = params.auv.r_max;
q_max = params.auv.q_max;

% SMC参数
smc_params.lambda = [0.5; 0.5; 0.5; 0.3; 0.3; 0.3];  % 滑模面斜率
smc_params.Kd = [5; 5; 5; 2; 2; 2];                  % 线性增益
smc_params.Ks = [2; 2; 2; 1; 1; 1];                  % 切换增益
smc_params.phi_b = [0.1; 0.1; 0.1; 0.05; 0.05; 0.05]; % 边界层厚度
smc_params.rho_eso = useESO * [0.8; 0.8; 0.8; 0.5; 0.5; 0.5]; % ESO补偿系数
smc_params.dt = h;

% ESO参数
eso_params = params.eso;
Z = zeros(6, 3);  % ESO状态 [Z1 Z2 Z3], 6个通道

% ALOS参数
alos_params = params.alos;

% 推力分配参数
thrust_params.rho = 1026;
thrust_params.D_prop = 0.10;
thrust_params.KT = 0.22;
thrust_params.n_max = n_max;
thrust_params.x_vert_f = +0.344;
thrust_params.x_vert_r = -0.293;
thrust_params.x_side_f = +0.424;
thrust_params.x_side_r = -0.376;

% 轨迹初始化
if TrajMode == 1
    pts = line_traj([xn;yn;zn], 0, 50, 50);
else
    pts = traj(50, 50);
end

% 历史数据存储
hist.t = t;
hist.x = zeros(nTimeSteps, 12);
hist.nu_d = zeros(nTimeSteps, 6);
hist.ui = zeros(nTimeSteps, 5);
hist.tau_cmd = zeros(nTimeSteps, 6);
hist.wave = zeros(nTimeSteps, 3);
hist.Z = zeros(nTimeSteps, 18);
hist.guidance = zeros(nTimeSteps, 6); % [psi_ref theta_ref y_e z_e psi_d theta_d]
hist.debug = zeros(nTimeSteps, 6);    % sigma或其他调试量

timebar(1, nTimeSteps, 'XHY AUV 6-DOF SMC+ESO仿真');

%% 主循环
for i = 1:nTimeSteps
    % 1. 状态提取
    u = x(1); v = x(2); w = x(3);
    p = x(4); q = x(5); r = x(6);
    xn = x(7); yn = x(8); zn = x(9);
    phi = x(10); theta = x(11); psi = x(12);
    nu = x(1:6);

    % 2. 海流更新（动态模型）
    if CurrentModel == 2
        if i > 1000
            Vc_d = 0.65;
            w_V = 0.05;
            Vc = exp(-h*w_V) * Vc + (1 - exp(-h*w_V)) * Vc_d;
        end
        if i > 2000
            betaVc_d = deg2rad(160);
            w_beta = 0.1;
            betaVc = exp(-h*w_beta) * betaVc + (1 - exp(-h*w_beta)) * betaVc_d;
        end
        betaVc = betaVc + randn / 1000;
        Vc = Vc + 0.002 * randn;
    end

    % 3. 获取动力学矩阵（用于控制器）
    [~, ~, M, C_nu, D_nu, g_vec, tau_thr] = xhy(x, ui, Vc, betaVc, wc);

    % 4. ESO更新
    y = nu;  % 测量值（速度）
    % 已知加速度项（从上一步动力学计算）
    if i == 1
        a_known = zeros(6,1);
    else
        a_known = M \ (tau_thr - C_nu - D_nu - g_vec);
    end
    [Z, aux] = vec_leso_update_adv(Z, y, a_known, eso_params, h);
    hat_d = Z(:, 3);  % 扰动估计（加速度单位）

    % 5. 制导：3D ALOS
    [psi_ref, theta_ref, y_e, z_e, alpha_c_hat, beta_c_hat, d] = ...
        my_ALOS3D(xn, yn, zn, h, pts, alos_params);

    % 滤波期望姿态
    [theta_d, q_d] = LOSobserver(theta_d, q_d, theta_ref, h, alos_params.K_f);
    [psi_d, r_d] = LOSobserver(psi_d, r_d, psi_ref, h, alos_params.K_f);

    % 限幅
    r_d = sat(r_d, r_max);
    q_d = sat(q_d, q_max);

    % 6. 构建期望速度向量
    nu_d = [1; 0; 0; 0; q_d; r_d];  % 期望: u=1m/s, 其他速度为0, 期望角速度来自制导
    nu_d_dot = zeros(6,1);          % 简化：假设期望加速度为0

    % 7. SMC控制律
    [tau_cmd, debug] = smc_6dof(nu, nu_d, nu_d_dot, M, C_nu, D_nu, g_vec, hat_d, smc_params);

    % 8. 推力分配：6-DOF力/力矩 → 5推进器RPM
    [ui, alloc_info] = thrust_allocation_xhy(tau_cmd, thrust_params);

    % 9. 存储历史数据
    hist.x(i, :) = x';
    hist.nu_d(i, :) = nu_d';
    hist.ui(i, :) = ui';
    hist.tau_cmd(i, :) = tau_cmd';
    hist.wave(i, :) = [Vc, betaVc, wc];
    hist.Z(i, :) = [Z(:,1)', Z(:,2)', Z(:,3)'];
    hist.guidance(i, :) = [psi_ref, theta_ref, y_e, z_e, psi_d, theta_d];
    hist.debug(i, :) = debug.sigma';

    % 10. 状态更新：RK4积分
    x = rk4(@xhy, h, x, ui, Vc, betaVc, wc);
    x(12) = ssa(x(12));  % 航向角归一化到[-pi, pi]

    timebar;
end

end
