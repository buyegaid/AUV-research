%% UCCO观测器测试：圆形轨迹，恒定海流
% 轨迹: 圆形（半径300m），航速 1 m/s
% 海流: Vc=0.5 m/s, beta_c=45°（恒定）
% 仿真: 100s, dt=0.01s
% 观测器: EG-UCCO简化版（速度层面预测-校正）
%
% 2026-06-08

clear; close all; clc;
% 清除所有persistent状态
clear my_ALOS3D LOSobserver smc_surge_xhy smc_yaw_xhy smc_heave_xhy eg_ucco_simple

%% ===== 路径设置 =====
project_root = setup_paths();

%% ===== 仿真参数 =====
dt     = 0.01;       % 时间步长 (s)
T_end  = 2000;        % 仿真时间 (s)
t_vec  = 0:dt:T_end;
N      = length(t_vec);

%% ===== 海流设置 =====
Vc     = 0.5;        % 海流速度 (m/s)
betaVc = deg2rad(45); % 海流方向 (rad, NED坐标系)
wc     = 0;          % 垂向海流

% 真实海流在NED坐标系的分量（用于对比验证）
c_true_N = Vc * cos(betaVc);  % N分量
c_true_E = Vc * sin(betaVc);  % E分量

%% ===== 参考轨迹：圆形 =====
pts = traj(50, 50);  % 弧长50m间距，50个点，半径300m（粗离散多边形逼近圆）
% wp1 = (300, 0, 0), 逆时针方向

%% ===== 控制器参数 =====
params = get_params();
params.alos.K_f = 0.5;      % LOS滤波器增益（默认值）
params.alos.R_switch = 20;  % 增大航点切换半径：海流导致~10m横向稳态误差，
                            % traj(100,10)航点间距100m，默认R_switch=10刚够不到

%% ===== 推进器参数（RPM模式） =====
thr_params.rho         = 1026;
thr_params.D_prop_main = 0.10;
thr_params.D_prop_aux  = 0.06;
thr_params.KT_main_fwd = 0.0310;
thr_params.KT_main_rev = 0.0213;
thr_params.KT_aux_fwd  = 0.444;
thr_params.KT_aux_rev  = 0.166;
thr_params.n_max       = 2500;
thr_params.x_vert_f    = +0.344;
thr_params.x_vert_r    = -0.293;
thr_params.x_side_f    = +0.424;
thr_params.x_side_r    = -0.376;

%% ===== 初始状态（在圆上，切向逆时针） =====
u0     = 1.0;        % 初始前进速度 (m/s)
psi0   = pi/2;        % 航向: 正东（圆在(300,0)处的逆时针切线方向）
xn0    = 300;         % 圆上起点 x=R=300
yn0    = 0;           % y=0
zn0    = 10;          % 深度10m
z_d    = 10;          % 目标深度

x_state = [u0; 0; 0; 0; 0; 0; xn0; yn0; zn0; 0; 0; psi0];

%% ===== 观测器初始化 =====
c_hat   = [0; 0];     % 初始海流估计 [cN_hat; cE_hat] (m/s)
M_const = compute_M_constant();  % 惯性矩阵（常数）

%% ===== 参考量初始化 =====
psi_d   = psi0;
r_d     = 0;
u_d     = 1.0;
u_d_dot = 0;

%% ===== 历史记录 =====
hist.t        = t_vec;
hist.nu       = zeros(N, 6);    % 真实速度(对地)
hist.pos      = zeros(N, 3);    % [x y z]
hist.psi      = zeros(N, 1);    % 航向角
hist.psi_ref  = zeros(N, 1);    % 参考航向
hist.tau      = zeros(N, 6);    % 控制力/力矩
hist.tau_thr  = zeros(N, 6);    % 实际推进器力/力矩
hist.ui       = zeros(N, 5);    % 推进器RPM
hist.c_hat    = zeros(N, 2);    % 海流估计 [cN cE]
hist.c_error  = zeros(N, 2);    % 估计误差
hist.y_e      = zeros(N, 1);    % 横向跟踪误差
hist.excited  = zeros(N, 1);    % 激励状态
hist.aux      = cell(N, 1);     % UCCO诊断信息

%% ===== 初始化推进器 =====
ui = zeros(5, 1);

fprintf('===== UCCO 圆形轨迹测试 =====\n');
fprintf('真实海流: Vc=%.1f m/s, beta_c=%.0f°, [cN=%.3f, cE=%.3f] m/s\n', ...
    Vc, rad2deg(betaVc), c_true_N, c_true_E);
fprintf('初始位置: (%.0f, %.0f, %.0f), 初始航向: %.0f°\n', xn0, yn0, zn0, rad2deg(psi0));
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
    w_heave = nu(3);  % 垂向速度

    % ----- 获取动力学矩阵 -----
    [~, ~, M, C, D, g_vec, tau_thr] = xhy(x_state, ui, Vc, betaVc, wc);

    % ----- ALOS 制导 -----
    [psi_ref, ~, y_e, ~, ~, ~, ~] = my_ALOS3D(xn, yn, zn, dt, pts, params.alos);
    [psi_d, r_d] = LOSobserver(psi_d, r_d, psi_ref, dt, params.alos.K_f);
    r_d = sat(r_d, 0.5);

    % ----- 控制律（三通道：surge + yaw + heave） -----
    X_cmd = smc_surge_xhy(nu(1), u_d, u_d_dot, dt, params.xhy.surge);
    N_cmd = smc_yaw_xhy(psi, nu(6), psi_d, r_d, 0, dt, params.xhy.yaw);
    Z_cmd = smc_heave_xhy(zn, w_heave, z_d, 0, 0, dt, params.xhy.heave);

    % 组装力/力矩指令（surge + heave + yaw; sway/roll/pitch=0）
    tau_cmd = [X_cmd; 0; Z_cmd; 0; 0; N_cmd];

    % ----- 推力分配（RPM模式） -----
    [ui, ~] = thrust_allocation_xhy(tau_cmd, thr_params);

    % ----- UCCO 海流观测器更新 -----
    [c_hat, aux_ucco] = eg_ucco_simple(c_hat, nu, tau_thr, psi, M_const, params.ucco, dt);

    % ----- 记录数据 -----
    hist.nu(i, :)     = nu';
    hist.pos(i, :)    = [xn, yn, zn];
    hist.psi(i)       = psi;
    hist.psi_ref(i)   = psi_ref;
    hist.tau(i, :)    = tau_cmd';
    hist.tau_thr(i,:) = tau_thr';
    hist.ui(i, :)     = ui';
    hist.c_hat(i, :)  = c_hat';
    hist.c_error(i, :)= (c_hat - [c_true_N; c_true_E])';
    hist.y_e(i)       = y_e;
    hist.aux{i}       = aux_ucco;
    if ~isempty(aux_ucco)
        hist.excited(i) = aux_ucco.excited;
    end

    % ----- 状态更新（RK4积分） -----
    x_state = rk4(@xhy, dt, x_state, ui, Vc, betaVc, wc);
    x_state(12) = ssa(x_state(12));

    timebar;
end
fprintf('\n');

%% ===== 轨迹跟踪诊断 =====
pos_final = hist.pos(end, :);
y_e_rms = sqrt(mean(hist.y_e.^2));
fprintf('===== 轨迹跟踪诊断 =====\n');
fprintf('  终点位置: (%.1f, %.1f, %.1f)\n', pos_final(1), pos_final(2), pos_final(3));
fprintf('  横向误差 RMSE: %.2f m\n', y_e_rms);
fprintf('  最大横向误差: %.2f m\n', max(abs(hist.y_e)));
fprintf('  深度 RMSE: %.2f m\n', sqrt(mean((hist.pos(:,3) - z_d).^2)));

%% ===== 结果输出 =====
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
if ~exist(fullfile(project_root, 'results'), 'dir')
    mkdir(fullfile(project_root, 'results'));
end

% --- 图1: 轨迹跟踪 ---
figure('Name', '轨迹跟踪', 'Color', 'w', 'Position', [50, 50, 900, 600]);

subplot(2,2,[1 3]);
plot(hist.pos(:,1), hist.pos(:,2), 'b-', 'LineWidth', 1.2); hold on;
plot(pts.pos.x, pts.pos.y, 'ro-', 'MarkerSize', 3, 'LineWidth', 0.5);
% 标记起点
plot(hist.pos(1,1), hist.pos(1,2), 'g^', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
plot(hist.pos(end,1), hist.pos(end,2), 'rs', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
xlabel('x (m)'); ylabel('y (m)');
title(sprintf('水平面轨迹 (y_e RMSE=%.1f m)', y_e_rms));
legend('AUV轨迹', '参考航点', '起点', '终点', 'Location', 'best');
grid on; box on; axis equal;

subplot(2,2,2);
plot(t_vec, hist.y_e, 'b-', 'LineWidth', 1.0);
grid on; box on;
xlabel('时间 (s)'); ylabel('y_e (m)');
title('横向跟踪误差');

subplot(2,2,4);
plot(t_vec, hist.pos(:,3), 'b-', 'LineWidth', 1.2); hold on;
yline(z_d, 'k--', 'LineWidth', 1.0);
grid on; box on;
xlabel('时间 (s)'); ylabel('z (m)');
title('深度跟踪');
legend('实际深度', '目标深度', 'Location', 'best');

sgtitle('XHY AUV 圆形轨迹跟踪');

% --- 图2: 速度跟踪 ---
figure('Name', '速度与航向', 'Color', 'w', 'Position', [100, 100, 900, 400]);

subplot(1,2,1);
plot(t_vec, hist.nu(:,1), 'b-', 'LineWidth', 1.2); hold on;
yline(u_d, 'k--', 'LineWidth', 1.5);
grid on; box on;
xlabel('时间 (s)'); ylabel('u (m/s)');
title('前进速度');
legend('实际 u', '目标 u_d', 'Location', 'best');

subplot(1,2,2);
plot(t_vec, rad2deg(hist.psi), 'b-', 'LineWidth', 1.2); hold on;
plot(t_vec, rad2deg(hist.psi_ref), 'r--', 'LineWidth', 1.0);
grid on; box on;
xlabel('时间 (s)'); ylabel('\psi (deg)');
title('航向跟踪');
legend('实际 \psi', '参考 \psi_{ref}', 'Location', 'best');

% --- 图3: 海流估计 ---
figure('Name', 'UCCO海流估计', 'Color', 'w', 'Position', [150, 150, 900, 600]);

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
yline(0, 'k--');
grid on; box on;
xlabel('时间 (s)'); ylabel('估计误差 (m/s)');
title('海流估计误差');
legend('c_N 误差', 'c_E 误差', 'Location', 'best');

subplot(2,2,4);
yyaxis left;
plot(t_vec, hist.excited, 'g-', 'LineWidth', 1.0);
ylim([-0.1, 1.1]); ylabel('激励状态');
yyaxis right;
lambda_log = zeros(N,1);
for i = 1:N
    if ~isempty(hist.aux{i})
        lambda_log(i) = hist.aux{i}.lambda_min;
    end
end
semilogy(t_vec, max(lambda_log, 1e-20), 'm-', 'LineWidth', 0.8);
ylabel('Gramian 最小特征值'); xlabel('时间 (s)');
title('激励门控状态');
legend('激励 (1=有)', '\lambda_{min}(W_c)', 'Location', 'best');
grid on; box on;

sgtitle('UCCO 海流观测器 — 圆形轨迹 (Vc=0.5 m/s, \beta_c=45°)');

% --- 图4: 控制力/力矩 ---
figure('Name', '控制力', 'Color', 'w', 'Position', [200, 200, 900, 400]);

subplot(1,3,1);
plot(t_vec, hist.tau(:,1), 'b-', 'LineWidth', 1.0);
grid on; box on;
xlabel('时间 (s)'); ylabel('X (N)');
title('纵荡力指令');

subplot(1,3,2);
plot(t_vec, hist.tau(:,3), 'b-', 'LineWidth', 1.0);
grid on; box on;
xlabel('时间 (s)'); ylabel('Z (N)');
title('沉浮力指令');

subplot(1,3,3);
plot(t_vec, hist.tau(:,6), 'b-', 'LineWidth', 1.0);
grid on; box on;
xlabel('时间 (s)'); ylabel('N (N·m)');
title('偏航力矩指令');

sgtitle('控制力/力矩指令');

fprintf('\n绘图完成。\n');

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
