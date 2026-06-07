function hist = xhy_pi_eso_simulator(method, current_scenario, params_in)
% XHY_PI_ESO_SIMULATOR 物理信息ESO-SMC对比仿真
%
% 对比三种方法在不同海流场景下的轨迹跟踪性能
%
% 输入:
%   method:           控制方法
%                     0 = 纯SMC（无ESO）
%                     1 = 经典LESO+SMC（标准ESO）
%                     2 = 物理信息ESO+SMC（PI-ESO，Gauss-Markov模型）
%   current_scenario: 海流场景 1-4（见 gauss_markov_current.m）
%   params_in:        可选，覆盖默认参数
%
% 输出:
%   hist: 历史数据结构体，含以下字段:
%     .t          时间向量
%     .x          状态矩阵 (N×12)
%     .ui         推进器RPM (N×5)
%     .tau        力/力矩指令 (N×6)
%     .xd         参考量 [psi_d theta_d psi_ref theta_ref] (N×4)
%     .traj       轨迹点
%     .Z          ESO状态 (N×18)
%     .hat_d      扰动估计 (N×6)
%     .Vc_t       实际流速序列 (N×1)
%     .betaVc_t   实际流向序列 (N×1)
%     .method     方法编号
%     .scenario   场景编号

project_root = setup_paths();

%% 参数设置
if nargin < 3 || isempty(params_in)
    params = get_params;
else
    params = params_in;
end

% 添加PI-ESO参数
if ~isfield(params, 'pieso')
    params.pieso = params.eso;
    params.pieso.tau_c = 50;  % Gauss-Markov相关时间常数 (s)
end

%% 仿真参数
h = 0.01;
T = 600;   % 仿真时长 (s)，直线600m@1m/s
t = 0:h:T;
N = length(t);

%% 初始状态（从原点出发，航向0°朝北）
zn  = 10;
psi0 = 0;   % 航向正北
x = [1; 0; 0; 0; 0; 0; 0; 0; zn; 0; 0; psi0];

%% 参考量初始化
theta_d = 0; q_d = 0;
psi_d   = psi0; r_d = 0;
u_d     = 1;    u_d_dot = 0;
z_d     = zn;   w_d = 0;  w_d_dot = 0;

%% 推力分配参数
thr_params.rho     = 1026;
thr_params.D_prop  = 0.10;
thr_params.KT      = 0.22;
thr_params.n_max   = 2500;
thr_params.x_vert_f = +0.344;
thr_params.x_vert_r = -0.293;
thr_params.x_side_f = +0.424;
thr_params.x_side_r = -0.376;

%% 轨迹（沿x轴直线，600m，20个航点）
n_pts = 20;
pts.pos.x = linspace(0, 600, n_pts)';
pts.pos.y = zeros(n_pts, 1);
pts.pos.z = zn * ones(n_pts, 1);
z_d = pts.pos.z(1);
params.alos.R_switch = 10;  % 切换半径 < 航点间距31.6m

%% 预生成海流序列
current_params.Vc_mean     = 0.8;   % 增大流速至0.8m/s，使扰动超出SMC鲁棒边界
current_params.betaVc_mean = pi/4;
current_params.sigma_Vc    = 0.15;
current_params.tau_c       = 100;  % 场景2的相关时间

% 场景3/4需要位置信息，先用零初始化，循环中实时更新
current_params.depth = zn * ones(1, N);
current_params.pos_x = zeros(1, N);
current_params.pos_y = zeros(1, N);

% 对场景1/2，预先生成完整序列
[Vc_t, betaVc_t, wc_t] = gauss_markov_current(current_scenario, t, current_params);

%% ESO状态初始化（Z1初始化为初始速度，消除初始瞬态）
Z = zeros(6, 3);
Z(:,1) = x(1:6);

%% 历史数据
hist.t      = t;
hist.x      = zeros(N, 12);
hist.ui     = zeros(N, 5);
hist.tau    = zeros(N, 6);
hist.xd     = zeros(N, 4);
hist.traj   = pts;
hist.Z      = zeros(N, 18);
hist.hat_d  = zeros(N, 6);
hist.Vc_t   = Vc_t';
hist.betaVc_t = betaVc_t';
hist.method   = method;
hist.scenario = current_scenario;

ui = zeros(5,1);

%% 主循环
for i = 1:N
    % 当前海流（场景3/4实时计算，场景1/2使用预生成序列）
    if current_scenario == 3
        % 深度剪切流：Vc随深度线性减小
        z_max   = 50;
        Vc_surf = current_params.Vc_mean * 1.5;
        Vc      = Vc_surf * max(0, 1 - x(9) / z_max);
        betaVc  = current_params.betaVc_mean;
        wc      = 0;
        Vc_t(i) = Vc; betaVc_t(i) = betaVc;
    elseif current_scenario == 4
        % 空间相关流场：高斯相关结构
        x0 = 25; y0 = 25; L = 20; A = 0.5;
        Vc     = current_params.Vc_mean * (1 + A * exp(-((x(7)-x0)^2 + (x(8)-y0)^2) / (2*L^2)));
        betaVc = current_params.betaVc_mean;
        wc     = 0;
        Vc_t(i) = Vc; betaVc_t(i) = betaVc;
    else
        Vc = Vc_t(i); betaVc = betaVc_t(i); wc = wc_t(i);
    end

    % 状态提取
    u     = x(1);  w = x(3);  q = x(5);  r = x(6);
    xn    = x(7);  yn = x(8); zn = x(9);
    theta = x(11); psi = x(12);

    % 动力学矩阵
    [~, ~, M, C, D, g_vec, tau_thr] = xhy(x, ui, Vc, betaVc, wc);

    % 相对速度
    u_c_x = Vc * cos(betaVc - psi);
    u_c_y = Vc * sin(betaVc - psi);
    nu_c  = [u_c_x; u_c_y; wc; 0; 0; 0];
    nu_r  = x(1:6) - nu_c;

    % 已知加速度（用绝对速度nu，ESO估计海流引起的力/力矩差）
    a_known = M \ (tau_thr - C*x(1:6) - D*x(1:6) - g_vec);

    % ESO更新（根据方法选择）
    switch method
        case 0
            % 纯SMC：不更新ESO，hat_d=0
            hat_d = zeros(6, 1);
        case 1
            % 经典LESO
            [Z, ~] = vec_leso_update_adv(Z, x(1:6), a_known, params.eso, h);
            hat_d  = M * Z(:, 3);
        case 2
            % 物理信息ESO（Gauss-Markov）
            [Z, ~] = vec_pieso_update(Z, x(1:6), a_known, params.pieso, h);
            hat_d  = M * Z(:, 3);
    end

    % 制导
    [psi_ref, theta_ref, ~, ~, ~, ~, ~] = my_ALOS3D(xn, yn, zn, h, pts, params.alos);
    [psi_d, r_d] = LOSobserver(psi_d, r_d, psi_ref, h, params.alos.K_f);
    r_d = sat(r_d, 0.5);

    % 深度控制（直接Z力模式）
    Z_cmd = smc_heave_xhy(zn, w, z_d, w_d, w_d_dot, h, params.xhy.heave) - hat_d(3);
    M_cmd = 0;

    % 单通道SMC + ESO前馈补偿
    X_cmd = smc_surge_xhy(u, u_d, u_d_dot, h, params.xhy.surge) - hat_d(1);
    N_cmd = smc_yaw_xhy(psi, r, psi_d, r_d, 0, h, params.xhy.yaw) - hat_d(6);

    % 力/力矩指令
    tau_cmd = [X_cmd; 0; Z_cmd; 0; M_cmd; N_cmd];

    % 推力分配
    [ui, ~] = thrust_allocation_xhy(tau_cmd, thr_params);

    % 存储
    hist.x(i,:)    = x';
    hist.ui(i,:)   = ui';
    hist.tau(i,:)  = tau_cmd';
    hist.xd(i,:)   = [psi_d, theta_d, psi_ref, theta_ref];
    hist.Z(i,:)    = [Z(:,1); Z(:,2); Z(:,3)]';
    hist.hat_d(i,:) = hat_d';

    % 状态更新
    x     = rk4(@xhy, h, x, ui, Vc, betaVc, wc);
    x(12) = ssa(x(12));
end

% 更新最终海流记录
hist.Vc_t     = Vc_t';
hist.betaVc_t = betaVc_t';
end
