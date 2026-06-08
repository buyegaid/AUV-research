%% UCCO观测器测试：圆形轨迹，恒定海流
% 轨迹: 圆形（半径300m），航速 1 m/s
% 海流: Vc=0.5 m/s, beta_c=45°（恒定）
% 仿真: 100s, dt=0.01s
% 观测器: EG-UCCO简化版（速度层面预测-校正）
%
% 2026-06-08

clear; close all; clc;
clear eg_ucco_simple  % 清除persistent状态

%% ===== 路径设置 =====
project_root = setup_paths();

%% ===== 仿真参数 =====
dt     = 0.01;      % 时间步长 (s)
T_end  = 100;       % 仿真时间 (s)
t_vec  = 0:dt:T_end;
N      = length(t_vec);

%% ===== 海流设置 =====
Vc     = 0.5;       % 海流速度 (m/s)
betaVc = deg2rad(45); % 海流方向 (rad, NED坐标系)
wc     = 0;         % 垂向海流

% 真实海流在NED坐标系的分量（用于对比验证）
c_true_N = Vc * cos(betaVc);  % N分量
c_true_E = Vc * sin(betaVc);  % E分量

%% ===== 参考轨迹：圆形 =====
% 使用 traj() 生成圆形航迹点
pts = traj(20, 50);  % 弧长20m间距，50个点，半径300m

%% ===== 控制器参数 =====
params = get_params();
params.alos.K_f = 0.2;  % LOS滤波器增益

%% ===== 推进器参数（RPM模式，简化） =====
thr_params.rho         = 1026;
thr_params.D_prop_main = 0.10;
thr_params.D_prop_aux  = 0.06;
thr_params.KT_main_fwd = 0.0293;
thr_params.KT_main_rev = 0.0201;
thr_params.KT_aux_fwd  = 0.327;
thr_params.KT_aux_rev  = 0.327;
thr_params.n_max       = 2500;
thr_params.x_vert_f    = +0.344;
thr_params.x_vert_r    = -0.293;
thr_params.x_side_f    = +0.424;
thr_params.x_side_r    = -0.376;

%% ===== 初始状态 =====
u0     = 1.0;   % 初始前进速度 (m/s)
psi0   = 0;     % 初始航向角 (rad)
xn0    = 350;   % 初始N位置 (m) — 偏离圆心稍远以观察收敛
yn0    = 0;     % 初始E位置 (m)
zn0    = 10;    % 初始深度 (m)

x_state = [u0; 0; 0; 0; 0; 0; xn0; yn0; zn0; 0; 0; psi0];

%% ===== 观测器初始化 =====
c_hat   = [0; 0];  % 初始海流估计 [cN_hat; cE_hat] (m/s)
M_const = compute_M_constant();  % 惯性矩阵（常数）

%% ===== 参考量初始化 =====
psi_d   = psi0;
r_d     = 0;
u_d     = 1.0;
u_d_dot = 0;

%% ===== 历史记录 =====
hist.t       = t_vec;
hist.nu      = zeros(N, 6);    % 真实速度(对地)
hist.pos     = zeros(N, 3);    % [x y z]
hist.psi     = zeros(N, 1);    % 航向角
hist.tau     = zeros(N, 6);    % 控制力/力矩
hist.ui      = zeros(N, 5);    % 推进器RPM
hist.c_hat   = zeros(N, 2);    % 海流估计 [cN cE]
hist.c_error = zeros(N, 2);    % 估计误差
hist.aux     = cell(N, 1);     % UCCO诊断信息

%% ===== 初始化推进器 =====
ui = zeros(5, 1);

fprintf('===== UCCO 圆形轨迹测试 =====\n');
fprintf('真实海流: Vc=%.1f m/s, beta_c=%.0f°, [cN=%.3f, cE=%.3f] m/s\n', ...
    Vc, rad2deg(betaVc), c_true_N, c_true_E);
fprintf('仿真时间: %d s, 步长: %.3f s\n', T_end, dt);
fprintf('轨迹: 圆形, R=300 m, 目标航速: 1 m/s\n\n');

%% ===== 主循环 =====
timebar(1, N, 'UCCO圆形轨迹测试');

for i = 1:N
    % ----- 状态提取 -----
    nu   = x_state(1:6);
    xn   = x_state(7);
    yn   = x_state(8);
    zn   = x_state(9);
    psi  = x_state(12);

    % ----- 获取动力学矩阵（用于控制计算） -----
    [~, ~, M, C, D, g_vec, tau_thr] = xhy(x_state, ui, Vc, betaVc, wc);

    % ----- ALOS 制导 -----
    [psi_ref, ~, ~, ~, ~, ~, ~] = my_ALOS3D(xn, yn, zn, dt, pts, params.alos);
    [psi_d, r_d] = LOSobserver(psi_d, r_d, psi_ref, dt, params.alos.K_f);
    r_d = sat(r_d, 0.5);  % 偏航角速度限制

    % ----- 控制律 -----
    X_cmd = smc_surge_xhy(nu(1), u_d, u_d_dot, dt, params.xhy.surge);
    N_cmd = smc_yaw_xhy(psi, nu(6), psi_d, r_d, 0, dt, params.xhy.yaw);

    % 组装力/力矩指令（仅控制Surge和Yaw）
    tau_cmd = [X_cmd; 0; 0; 0; 0; N_cmd];

    % ----- 推力分配（RPM模式） -----
    [ui, ~] = thrust_allocation_xhy(tau_cmd, thr_params);

    % ----- UCCO 海流观测器更新 -----
    [c_hat, aux] = eg_ucco_simple(c_hat, nu, tau_thr, psi, M_const, params.ucco, dt);

    % ----- 记录数据 -----
    hist.nu(i, :)    = nu';
    hist.pos(i, :)   = [xn, yn, zn];
    hist.psi(i)      = psi;
    hist.tau(i, :)   = tau_cmd';
    hist.ui(i, :)    = ui';
    hist.c_hat(i, :) = c_hat';
    hist.c_error(i, :) = (c_hat - [c_true_N; c_true_E])';
    hist.aux{i}      = aux;

    % ----- 状态更新（RK4积分） -----
    x_state = rk4(@xhy, dt, x_state, ui, Vc, betaVc, wc);
    x_state(12) = ssa(x_state(12));

    timebar;
end
fprintf('\n');

%% ===== 结果输出 =====
% 计算稳态估计精度（最后20s）
steady_idx = t_vec > 80;
c_err_steady = hist.c_error(steady_idx, :);
c_hat_steady = hist.c_hat(steady_idx, :);
rmse_N = sqrt(mean(c_err_steady(:,1).^2));
rmse_E = sqrt(mean(c_err_steady(:,2).^2));
fprintf('===== 稳态估计精度 (t=80-100s) =====\n');
fprintf('  cN 估计值: %.4f ± %.4f m/s (真实: %.4f)\n', mean(c_hat_steady(:,1)), std(c_hat_steady(:,1)), c_true_N);
fprintf('  cE 估计值: %.4f ± %.4f m/s (真实: %.4f)\n', mean(c_hat_steady(:,2)), std(c_hat_steady(:,2)), c_true_E);
fprintf('  cN RMSE: %.4f m/s\n', rmse_N);
fprintf('  cE RMSE: %.4f m/s\n', rmse_E);

%% ===== 绘图 =====
% 创建结果目录
if ~exist(fullfile(project_root, 'results'), 'dir')
    mkdir(fullfile(project_root, 'results'));
end

% --- 图1: 海流估计 vs 真实值 ---
figure('Name', 'UCCO海流估计', 'Color', 'w', 'Position', [100, 100, 900, 600]);

subplot(2,2,1);
plot(t_vec, c_true_N * ones(N,1), 'k--', 'LineWidth', 1.5); hold on;
plot(t_vec, hist.c_hat(:,1), 'b-', 'LineWidth', 1.5);
grid on; box on;
xlabel('时间 (s)'); ylabel('c_N (m/s)');
title('海流 N 分量估计');
legend('真实值', 'UCCO估计', 'Location', 'best');

subplot(2,2,2);
plot(t_vec, c_true_E * ones(N,1), 'k--', 'LineWidth', 1.5); hold on;
plot(t_vec, hist.c_hat(:,2), 'r-', 'LineWidth', 1.5);
grid on; box on;
xlabel('时间 (s)'); ylabel('c_E (m/s)');
title('海流 E 分量估计');
legend('真实值', 'UCCO估计', 'Location', 'best');

subplot(2,2,3);
plot(t_vec, hist.c_error(:,1), 'b-', 'LineWidth', 1.0); hold on;
plot(t_vec, hist.c_error(:,2), 'r-', 'LineWidth', 1.0);
grid on; box on;
xlabel('时间 (s)'); ylabel('估计误差 (m/s)');
title('海流估计误差');
legend('c_N 误差', 'c_E 误差', 'Location', 'best');

subplot(2,2,4);
excited_log = zeros(N,1);
for i = 1:N
    if ~isempty(hist.aux{i})
        excited_log(i) = hist.aux{i}.excited;
    end
end
yyaxis left;
plot(t_vec, excited_log, 'g-', 'LineWidth', 1.0);
ylim([-0.1, 1.1]); ylabel('激励状态');
yyaxis right;
lambda_log = zeros(N,1);
for i = 1:N
    if ~isempty(hist.aux{i})
        lambda_log(i) = hist.aux{i}.lambda_min;
    end
end
semilogy(t_vec, max(lambda_log, 1e-20), 'm-', 'LineWidth', 0.8);
ylabel('Gramian最小特征值'); xlabel('时间 (s)');
title('激励门控状态');
legend('激励 (1=有, 0=无)', '\lambda_{min}(W_c)', 'Location', 'best');
grid on; box on;

sgtitle('UCCO 海流观测器 — 圆形轨迹 (Vc=0.5 m/s, \beta_c=45°)');

% --- 图2: 速度跟踪 ---
figure('Name', '速度跟踪', 'Color', 'w', 'Position', [150, 150, 900, 400]);

subplot(1,2,1);
plot(t_vec, hist.nu(:,1), 'b-', 'LineWidth', 1.2); hold on;
yline(u_d, 'k--', 'LineWidth', 1.5);
grid on; box on;
xlabel('时间 (s)'); ylabel('u (m/s)');
title('前进速度跟踪');
legend('实际速度 u', '目标速度 u_d', 'Location', 'best');

subplot(1,2,2);
plot(t_vec, rad2deg(hist.psi), 'b-', 'LineWidth', 1.2);
grid on; box on;
xlabel('时间 (s)'); ylabel('\psi (deg)');
title('航向角');

sgtitle('AUV 运动状态');

% --- 图3: 轨迹 ---
figure('Name', '轨迹', 'Color', 'w', 'Position', [200, 200, 600, 500]);
plot(hist.pos(:,1), hist.pos(:,2), 'b-', 'LineWidth', 1.2); hold on;
plot(pts.pos.x, pts.pos.y, 'ro-', 'MarkerSize', 4, 'LineWidth', 0.5);
xlabel('x (m)'); ylabel('y (m)');
title('水平面轨迹');
legend('AUV轨迹', '参考航点', 'Location', 'best');
grid on; box on; axis equal;

fprintf('\n绘图完成。\n');
fprintf('结果保存在 results/ 目录\n');

%% ===== 辅助函数 =====
function M = compute_M_constant()
% 计算XHY的常数惯性矩阵 M = MRB + MA
m = 85.832;
Ix = 0.553864787 + 0.865274;
Iy = 2.162341935 + 5.011187;
Iz = 1.849137 + 4.541468;
MRB = diag([m, m, m, Ix, Iy, Iz]);
MA = diag([15.81, 124.73, 42.87, 0.014, 0.041, 0.123]);
M = MRB + MA;
end
