function xhy_sim_compare(ObserverMode, TrajMode, CurrentMode, DepthMode, useModelMismatch, mismatch_level, compAblation)
%% EG-UCCO + EKF + 运动学观测器 集成仿真
% 用法: xhy_sim_compare(ObserverMode, TrajMode, CurrentMode, DepthMode, ...
%                        useModelMismatch, mismatch_level, compAblation)
%   方法: 1=SMC 2=KIN 3=LESO 4=PIESO 5=EKF 6=UCCO 7=UCCO-plain 8=ALL
%   轨迹: 1=直线 2=圆形
%   海流: 1=恒定 2=GM缓变 3=剪切 4=空间涡
%   compAblation: 0=全补 1=无补 2=仅surge 3=仅yaw
%   2026-06-04

if nargin < 1, ObserverMode = 6; end
if nargin < 2, TrajMode = 2; end
if nargin < 3, CurrentMode = 2; end
if nargin < 4, DepthMode = 2; end
if nargin < 5, useModelMismatch = false; end
if nargin < 6, mismatch_level = 0.10; end
if nargin < 7, compAblation = 0; end

clear eg_ucco_update ekf_current_estimator kin_current_observer

if ObserverMode == 8
    % 批量运行所有方法
    all_methods = [1 2 3 4 5 6];
    all_results = cell(length(all_methods), 1);
    for m = 1:length(all_methods)
        fprintf('\n===== 运行方法 %d/%d =====\n', m, length(all_methods));
        all_results{m} = run_single_sim(all_methods(m), TrajMode, CurrentMode, ...
                                        DepthMode, useModelMismatch, mismatch_level);
    end
    % 对比分析
    compare_all_observers(all_results, all_methods);
    return;
end

result = run_single_sim(ObserverMode, TrajMode, CurrentMode, DepthMode, ...
                        useModelMismatch, mismatch_level);
plot_single_result(result);

%% ====== 单次仿真 ======
function result = run_single_sim(ObserverMode, TrajMode, CurrentMode, DepthMode, ...
                                  useMismatch, mismatch_level)
method_names = {'SMC','KIN-SMC','LESO-SMC','PIESO-SMC','EKF-SMC','UCCO-SMC','UCCO-plain'};
fprintf('方法: %s | 轨迹: %d | 海流场景: %d\n', method_names{ObserverMode}, TrajMode, CurrentMode);

params = get_params;

h = 0.01;  T = 100;  t = 0:h:T;  N = length(t);  % 对比实验用100s

%% 初始状态
xn = 0; yn = 0; zn = 10;  psi0 = 0;
x = [1; 0; 0; 0; 0; 0; xn; yn; zn; 0; 0; psi0];

%% 参考量初始化
theta_d = 0; q_d = 0;  psi_d = psi0; r_d = 0;
u_d = 1;  u_d_dot = 0;  z_d = zn; w_d = 0; w_d_dot = 0;

%% 海流参数 — 预生成整个时变序列（避免每步调用GM生成器）
gm_params = struct('Vc_mean', 0.3, 'betaVc_mean', pi/4, 'sigma_Vc', 0.1, 'tau_c', 100);
[Vc_seq, beta_seq, wc_seq] = gauss_markov_current(CurrentMode, t, gm_params);
Vc_true  = Vc_seq(1);
beta_true = beta_seq(1);
wc_true  = wc_seq(1);

%% 推力分配参数
thr_params = get_thruster_params();

%% ESO状态
Z = zeros(6, 3);

%% EG-UCCO状态
c_hat   = [0; 0];    % [cN; cE] 惯性系海流估计
P_c     = 0.01 * eye(2);
nu_obs_ucco = x(1:6);

%% EKF状态
if params.ekf.use_bias
    x_ekf = [x(1:6); 0; 0; zeros(6,1)];
    P_ekf = 0.01 * eye(14);
else
    x_ekf = [x(1:6); 0; 0];
    P_ekf = 0.01 * eye(8);
end

%% 运动学观测器状态
Vc_kin = 0;  beta_kin = 0;
x_hat_kin = [xn; yn; zn];

%% 轨迹
if TrajMode == 1
    pts = line_traj([xn; yn; zn], 0, 50, 50);
else
    pts = traj(50, 50);
end
z_d = pts.pos.z(1);

%% 历史数据
hist.t = t;
hist.x = zeros(N, 12);
hist.ui = zeros(N, 5);
hist.tau = zeros(N, 6);
hist.tau_cmd = zeros(N, 6);
hist.traj = pts;
hist.xd   = zeros(N, 4);    % [psi_d theta_d psi_ref theta_ref]
hist.c_est = zeros(N, 2);     % [cN_hat, cE_hat]
hist.c_true = zeros(N, 2);    % [cN_true, cE_true]
hist.conf = zeros(N, 1);       % 置信度
hist.Vc_est = zeros(N, 1);     % Vc估计值（用于统一比较）
hist.beta_est = zeros(N, 1);   % βc估计值
hist.aux_ucco = cell(N, 1);

ui = zeros(5,1);
timebar(1, N, sprintf('XHY仿真 [%s]', method_names{ObserverMode}));

%% ====== 主循环 ======
for i = 1:N
    % 状态提取
    u = x(1); v = x(2); w = x(3); p = x(4); q = x(5); r = x(6);
    xn = x(7); yn = x(8); zn = x(9);
    phi = x(10); theta = x(11); psi = x(12);

    % 时变海流（从预生成序列中取）
    Vc_true  = Vc_seq(i);
    beta_true = beta_seq(i);
    wc_true  = wc_seq(i);

    % 动力学矩阵
    [~, ~, M, C, D, g_vec, tau_thr] = xhy(x, ui, Vc_true, beta_true, wc_true);

    % 相对速度
    u_c_x = Vc_true * cos(beta_true - psi);
    u_c_y = Vc_true * sin(beta_true - psi);
    nu_c_true = [u_c_x; u_c_y; wc_true; 0; 0; 0];
    nu_r = x(1:6) - nu_c_true;

    % 已知加速度
    a_known = M \ (tau_thr - C*nu_r - D*nu_r - g_vec);

    %% ====== 海流估计（按ObserverMode选择） ======
    hat_d = zeros(6, 1);  % ESO补偿量（力/力矩单位）
    Vc_est = 0; beta_est = 0;  % 用于控制和记录
    confidence = 0;

    switch ObserverMode
        case 1  % SMC — 无补偿
            % hat_d = 0

        case 2  % KIN-SMC — 运动学海流观测器
            [Vc_kin, beta_kin, x_hat_kin, aux_kin] = ...
                kin_current_observer(Vc_kin, beta_kin, x_hat_kin, ...
                    x(1:6), [xn; yn; zn], psi, theta, params.kin, h);
            Vc_est = Vc_kin;
            beta_est = beta_kin;

            % 前馈补偿 = 估计海流引起的净力/力矩
            hat_d = compute_current_compensation(Vc_est, beta_est, x(1:6), psi, tau_thr, M);

        case 3  % LESO-SMC — 标准LESO
            [Z, aux_eso] = vec_leso_update_adv(Z, x(1:6), a_known, params.eso, h);
            hat_d = M * Z(:, 3);

        case 4  % PIESO-SMC — 物理信息ESO
            [Z, aux_eso] = vec_pieso_update(Z, x(1:6), a_known, params.pieso, h);
            hat_d = M * Z(:, 3);

        case 5  % EKF-SMC — CFD增广EKF
            [x_ekf, P_ekf, aux_ekf] = ekf_current_estimator(x_ekf, P_ekf, ...
                x(1:6), tau_thr, psi, M, params, h);
            Vc_est = norm(aux_ekf.c_hat);
            beta_est = atan2(aux_ekf.c_hat(2), aux_ekf.c_hat(1));

            % EKF前馈补偿
            hat_d = compute_current_compensation(Vc_est, beta_est, x(1:6), psi, tau_thr, M);

            c_hat = aux_ekf.c_hat;
            confidence = 1 / (1 + trace(P_ekf(7:8,7:8)) / 0.01);

        case 6  % UCCO-SMC — EG-UCCO简化版（仅Yaw前馈，Surge由PID处理）
            [c_hat, aux_ucco] = eg_ucco_simple(c_hat, x(1:6), tau_thr, psi, M, params.ucco, h);
            Vc_est = norm(c_hat);
            beta_est = atan2(c_hat(2), c_hat(1));
            confidence = 0.5;
            hist.aux_ucco{i} = aux_ucco;
            hat_d_full = compute_current_compensation(Vc_est, beta_est, x(1:6), psi, tau_thr, M);
            % 仅Yaw前馈（消融实验证实Surge补偿损害航向控制）
            switch compAblation
                case 0, hat_d = [0;0;0;0;0;hat_d_full(6)];        % 仅Yaw（默认最优）
                case 1, hat_d = zeros(6,1);                        % 无补偿
                case 2, hat_d = hat_d_full;                        % 全补偿（劣化）
                case 3, hat_d = [hat_d_full(1);0;0;0;0;hat_d_full(6)]; % Surge+Yaw

        case 7  % UCCO-plain — 消融版（无门控/无死区）
            params_plain = params.ucco;
            params_plain.gate_mu = 0;        % 关闭门控
            params_plain.delta_single = zeros(6,1);  % 关闭死区
            params_plain.delta_coupling = zeros(6,1);
            params_plain.delta_thruster = 0;
            [c_hat, P_c, nu_obs_ucco, aux_ucco] = eg_ucco_update(...
                c_hat, P_c, nu_obs_ucco, x(1:6), tau_thr, psi, M, params_plain, h);
            Vc_est = norm(c_hat);
            beta_est = atan2(c_hat(2), c_hat(1));

            % UCCO-plain前馈补偿
            hat_d = compute_current_compensation(Vc_est, beta_est, x(1:6), psi, tau_thr, M);
    end

    %% ====== 制导 + 控制 ======
    [psi_ref, theta_ref, ~, ~, ~, ~, ~] = my_ALOS3D(xn, yn, zn, h, pts, params.alos);
    [psi_d, r_d] = LOSobserver(psi_d, r_d, psi_ref, h, params.alos.K_f);
    r_d = sat(r_d, 0.5);

    if DepthMode == 1
        [theta_d, q_d] = LOSobserver(theta_d, q_d, theta_ref, h, params.alos.K_f);
        q_d = sat(q_d, 0.5);
        M_cmd = smc_pitch_xhy(theta, q, theta_d, q_d, 0, h, params.xhy.pitch) - hat_d(5);
        Z_cmd = 0;
    else
        Z_cmd = smc_heave_xhy(zn, w, z_d, w_d, w_d_dot, h, params.xhy.heave) - hat_d(3);
        M_cmd = 0;
    end

    X_cmd = smc_surge_xhy(u, u_d, u_d_dot, h, params.xhy.surge) - hat_d(1);
    N_cmd = smc_yaw_xhy(psi, r, psi_d, r_d, 0, h, params.xhy.yaw) - hat_d(6);

    tau_cmd = [X_cmd; 0; Z_cmd; 0; M_cmd; N_cmd];
    [ui, ~] = thrust_allocation_xhy(tau_cmd, thr_params);

    %% ====== 存储 ======
    hist.x(i,:) = x';
    hist.ui(i,:) = ui';
    hist.tau(i,:) = tau_cmd';
    hist.tau_cmd(i,:) = tau_cmd';
    hist.xd(i,:) = [psi_d, theta_d, psi_ref, theta_ref];

    % 海流真值（惯性系）
    cN_true = Vc_true * cos(beta_true);
    cE_true = Vc_true * sin(beta_true);
    hist.c_true(i,:) = [cN_true, cE_true];
    hist.c_est(i,:) = [Vc_est*cos(beta_est), Vc_est*sin(beta_est)];
    hist.conf(i) = confidence;
    hist.Vc_est(i) = Vc_est;
    hist.beta_est(i) = beta_est;

    %% ====== 状态更新 ======
    if useMismatch
        % 模型失配：扰动阻力系数
        x_mis = xhy_mismatch(x, ui, Vc_true, beta_true, wc_true, mismatch_level);
    else
        x_mis = rk4(@xhy, h, x, ui, Vc_true, beta_true, wc_true);
    end
    x = x_mis;
    x(12) = ssa(x(12));

    timebar;
end

%% ====== 打包结果 ======
result.method = method_names{ObserverMode};
result.hist = hist;
result.params = params;
result.CurrentMode = CurrentMode;
result.TrajMode = TrajMode;
result.mismatch = useMismatch;

% 计算指标
cN_err = hist.c_est(:,1) - hist.c_true(:,1);
cE_err = hist.c_est(:,2) - hist.c_true(:,2);
result.rmse_Vc = sqrt(mean(cN_err.^2 + cE_err.^2));
result.rmse_psi = sqrt(mean((hist.x(:,12) - hist.xd(:,1)).^2));

fprintf('  RMSE_Vc=%.4f m/s, RMSE_psi=%.4f rad\n', result.rmse_Vc, result.rmse_psi);
end

%% ====== 模型失配版本 ======
function x_next = xhy_mismatch(x, ui, Vc, betaVc, w_c, level)
% 带模型失配的XHY动力学（用于Inverse Crime避免）
% 阻力系数被随机扰动 level（如±10%）
persistent pert_factors
if isempty(pert_factors)
    rng(20260604);  % 固定随机种子保证可重复
    pert_factors = 1 + level * (2*rand(6,1) - 1);  % 各DOF独立扰动
end

% 调用标准xhy
[xdot, ~, M, C, D, g_vec, tau] = xhy(x, ui, Vc, betaVc, w_c);

% 扰动D矩阵的对角元素
nu = x(1:6);
nu_r = nu - [Vc*cos(betaVc-x(12)); Vc*sin(betaVc-x(12)); w_c; 0; 0; 0];
[~, D_orig] = xhy_drag_cfd(nu_r, M);

% 应用扰动
D_pert = diag(pert_factors) * D_orig;
tau_drag_pert = -D_pert * nu_r;  % 简化处理

% 重新计算加速度
xdot(1:6) = xdot(1:6) + M \ (tau_drag_pert - xhy_drag_cfd_simple(nu_r));

x_next = x + xdot * 0.01;  % Euler步（与主循环dt一致）
end

function tau_drag = xhy_drag_cfd_simple(nu_r)
% 简化版阻力计算（避免重复调用xhy_drag_cfd的内部逻辑）
u=nu_r(1); v=nu_r(2); w=nu_r(3); p=nu_r(4); q=nu_r(5); r=nu_r(6);
Fx = -(0.7580*u + 3.7729*u*abs(u));
Fy = -(0.0*v + 130.7023*v*abs(v));
Fz = -(1.4485*w + 45.0442*w*abs(w));
K = 0;
My = -(0.001345*q + 0.00453*q*abs(q));
Nz = -(0.027322*r + 0.067903*r*abs(r));
tau_drag = [Fx; Fy; Fz; K; My; Nz];
end

%% ====== 辅助函数 ======
function [C, g] = compute_cg(nu_r, psi, M)
m = 33;
Ix = 0.540804; Iy = 2.107488; Iz = 1.849137;
Ig = diag([Ix Iy Iz]);
nu2 = nu_r(4:6);
O3 = zeros(3,3);
CRB = [m*Smtrx(nu2), O3; O3, -Smtrx(Ig*nu2)];
MA = diag([11.2773, 132.1086, 47.104, 0.006, 0.043, 0.138]);
CA = m2c(MA, nu_r);
CA(5,3)=0; CA(3,5)=0; CA(5,1)=0; CA(1,5)=0; CA(6,1)=0; CA(1,6)=0;
C = CRB + CA;
mu = 63.446827;
g_mu = gravity(mu);
W = m * g_mu; B = W * 1.01;
[~, R] = eulerang(0, 0, psi);
r_bG = [0;0;0]; r_bB = [0;0;-0.03];
g = gRvect(W, B, R, r_bG, r_bB);
end

function thr = get_thruster_params()
thr.rho = 1026;
thr.D_prop_main = 0.10;
thr.D_prop_aux = 0.06;
thr.KT_main_fwd = 0.0293;
thr.KT_main_rev = 0.0201;
thr.KT_aux_fwd = 0.327;
thr.KT_aux_rev = 0.327;
thr.n_max = 2500;
thr.x_vert_f = +0.344;
thr.x_vert_r = -0.293;
thr.x_side_f = +0.424;
thr.x_side_r = -0.376;
end

function plot_single_result(result)
% 单次仿真结果绘图
figure('Name', sprintf('海流估计结果 — %s', result.method), 'Position', [100 100 1200 800]);

t = result.hist.t;
c_est = result.hist.c_est;
c_true = result.hist.c_true;

% 海流估计 vs 真值
subplot(2,3,1);
plot(t, c_true(:,1), 'k--', 'LineWidth', 1.5); hold on;
plot(t, c_est(:,1), 'b-', 'LineWidth', 1.2);
xlabel('时间 (s)'); ylabel('c_N (m/s)');
legend('真值', '估计'); title('北向海流分量'); grid on;

subplot(2,3,2);
plot(t, c_true(:,2), 'k--', 'LineWidth', 1.5); hold on;
plot(t, c_est(:,2), 'r-', 'LineWidth', 1.2);
xlabel('时间 (s)'); ylabel('c_E (m/s)');
legend('真值', '估计'); title('东向海流分量'); grid on;

% 海流速度幅值
subplot(2,3,3);
Vc_true = sqrt(c_true(:,1).^2 + c_true(:,2).^2);
Vc_est = sqrt(c_est(:,1).^2 + c_est(:,2).^2);
plot(t, Vc_true, 'k--', 'LineWidth', 1.5); hold on;
plot(t, result.hist.Vc_est, 'g-', 'LineWidth', 1.2);
xlabel('时间 (s)'); ylabel('V_c (m/s)');
legend('真值', '估计'); title('海流速度幅值'); grid on;

% 估计误差
subplot(2,3,4);
c_err = sqrt((c_est(:,1)-c_true(:,1)).^2 + (c_est(:,2)-c_true(:,2)).^2);
plot(t, c_err, 'b-', 'LineWidth', 1.2);
xlabel('时间 (s)'); ylabel('|c_{err}| (m/s)');
title(sprintf('估计误差 RMSE=%.4f m/s', result.rmse_Vc)); grid on;

% 置信度（如果有）
subplot(2,3,5);
if any(result.hist.conf > 0)
    plot(t, result.hist.conf, 'm-', 'LineWidth', 1.2);
    xlabel('时间 (s)'); ylabel('置信度'); title('估计置信度'); grid on;
else
    text(0.5, 0.5, 'N/A', 'FontSize', 14, 'HorizontalAlignment', 'center');
    title('置信度（无）');
end

% 轨迹
subplot(2,3,6);
plot(result.hist.x(:,7), result.hist.x(:,8), 'b-', 'LineWidth', 1);
hold on;
plot(result.hist.traj.pos.x, result.hist.traj.pos.y, 'r--', 'LineWidth', 1.5);
xlabel('x (m)'); ylabel('y (m)'); legend('实际', '参考');
title('水平轨迹'); grid on; axis equal;

sgtitle(sprintf('海流估计: %s (场景%d, 轨迹%d)', ...
    result.method, result.CurrentMode, result.TrajMode));
end

function compare_all_observers(all_results, all_methods)
% 对比所有观测器方法
method_names = {'SMC','KIN','LESO','PIESO','EKF','UCCO'};
n_methods = length(all_methods);

fprintf('\n========== 对比结果 ==========\n');
fprintf('%-12s %12s %12s\n', '方法', 'RMSE_Vc', 'RMSE_psi');
fprintf('%s\n', repmat('-', 1, 40));
for i = 1:n_methods
    fprintf('%-12s %12.4f %12.4f\n', ...
        method_names{all_methods(i)}, ...
        all_results{i}.rmse_Vc, all_results{i}.rmse_psi);
end

% 对比图
figure('Name', '观测器方法对比', 'Position', [100 100 1400 600]);

for i = 1:n_methods
    subplot(2, 3, i);
    t = all_results{i}.hist.t;
    c_true = all_results{i}.hist.c_true;
    c_est = all_results{i}.hist.c_est;
    Vc_err = sqrt((c_est(:,1)-c_true(:,1)).^2 + (c_est(:,2)-c_true(:,2)).^2);
    plot(t, Vc_err, 'LineWidth', 1.2);
    xlabel('时间 (s)'); ylabel('|c_{err}| (m/s)');
    title(sprintf('%s (RMSE=%.4f)', method_names{all_methods(i)}, all_results{i}.rmse_Vc));
    grid on; ylim([0, max(0.5, max(Vc_err)*1.2)]);
end
sgtitle('海流估计误差对比');
end
function hat_d = compute_current_compensation(Vc_est, beta_est, nu, psi, tau_thr, M)
% 计算海流前馈补偿力/力矩
% hat_d = M * (加速度含海流 - 加速度零海流)
% 即：估计海流对AUV动力学产生的净力效应
if Vc_est < 1e-6
    hat_d = zeros(6, 1);
    return;
end

% 海流船体系分量
u_c = Vc_est * cos(beta_est - psi);
v_c = Vc_est * sin(beta_est - psi);
nu_c = [u_c; v_c; 0; 0; 0; 0];

% 含估计海流的动力学加速度
nu_r_est = nu - nu_c;
[tau_drag_est, ~] = xhy_drag_cfd(nu_r_est, M);
[C_est, g_est] = compute_cg(nu_r_est, psi, M);
Dnu_c = [nu(6)*v_c; -nu(6)*u_c; 0; 0; 0; 0];
nu_dot_c = Dnu_c + M \ (tau_thr + tau_drag_est - C_est*nu_r_est - g_est);

% 零海流动力学加速度
[tau_drag_0, ~] = xhy_drag_cfd(nu, M);
[C_0, g_0] = compute_cg(nu, psi, M);
nu_dot_0 = M \ (tau_thr + tau_drag_0 - C_0*nu - g_0);

% 海流净力效应
hat_d = M * (nu_dot_c - nu_dot_0);
end

end  % xhy_sim_compare 主函数
