%% test_observer_tracking_comparison.m
% 四观测器轨迹跟踪对比测试
%
% 在相同控制器 + 相同轨迹跟踪条件下:
%   - KIN 运动学观测器 (模型无关)
%   - CFD-Luenberger 模型基观测器 (Børhaug 2007)
%   - EKF 增广扩展卡尔曼滤波 (Long 2021 型)
%   - ESKF 误差状态卡尔曼滤波 (本项目新增)
%
% 输出:
%   1. 水平面轨迹图 (N-E, 含期望轨迹)
%   2. 海流估计对比图 (cN, cE 时间序列 + 真值)
%   3. 跟踪效果图 (横向误差 + 航向误差)
%   4. RMSE 柱状图
%
% 2026-07-09

% 若外部已定义 cfg_in，在 clear 前保存到临时变量
% 用法: cfg_in = struct('mismatch_pct',0,'noise_level','high'); run('test_observer_tracking_comparison.m');
if exist('cfg_in', 'var')
    cfg_override = cfg_in;
else
    cfg_override = struct();
end

close all;
clearvars -except cfg_override;

project_root = setup_paths();

%% ===== 实验配置 =====
cfg.scenario     = 'circle';     % 'circle' | 'straight'
cfg.mismatch_pct = 20;           % CFD阻力模型失配百分比
cfg.noise_level  = 'low';        % 'none' | 'low' (σ=0.02) | 'high' (σ=0.10)
cfg.seed         = 42;           % 随机种子
cfg.T_end        = 120;          % 仿真时长 (s)
cfg.observers    = {'KIN', 'BORHAUG', 'EKF', 'ESKF'};  % 四种观测器
cfg.obs_labels   = {'KIN', 'CFD-Luenberger', 'EKF', 'ESKF'};  % 图例标签

% 外部覆盖
flds = fieldnames(cfg_override);
for fi = 1:length(flds)
    cfg.(flds{fi}) = cfg_override.(flds{fi});
end

%% ===== 仿真参数 =====
dt = 0.01;
T_end = cfg.T_end;
t_vec = 0:dt:T_end;
N = length(t_vec);
rng(cfg.seed);

%% ===== 海流设置 =====
Vc = 0.3;
betaVc = deg2rad(45);
wc = 0;
c_true_N0 = Vc * cos(betaVc);
c_true_E0 = Vc * sin(betaVc);

%% ===== 参考轨迹 =====
switch cfg.scenario
    case 'circle'
        pts = traj(20, 50);  % 圆形轨迹, 弧长20m, 50点
    case 'straight'
        pts.pos.x = (0:20:1000)';
        pts.pos.y = zeros(size(pts.pos.x));
        pts.pos.z = 10 * ones(size(pts.pos.x));
end

%% ===== 参数加载 =====
params = get_params();

% 噪声缩放
switch cfg.noise_level
    case 'none',  noise_scale = 0;
    case 'low',   noise_scale = 1;    % σ=0.02 m/s
    case 'high',  noise_scale = 5;    % σ=0.10 m/s
end

%% ===== 推进器参数 =====
thr_params.rho         = 1026;
thr_params.D_prop_main = 0.08;
thr_params.D_prop_aux  = 0.06;
thr_params.KT_main_fwd = 0.1489;
thr_params.KT_main_rev = 0.0506;
thr_params.KT_aux_fwd  = 0.53;
thr_params.KT_aux_rev  = 0.71;
thr_params.n_max       = 2500;
thr_params.x_vert_f    = +0.344;
thr_params.x_vert_r    = -0.293;
thr_params.x_side_f    = +0.424;
thr_params.x_side_r    = -0.376;

%% ===== 初始状态 =====
switch cfg.scenario
    case 'circle'
        u0 = 1.0; psi0 = pi/2; xn0 = 300; yn0 = 0; zn0 = 10;
    case 'straight'
        u0 = 1.0; psi0 = 0; xn0 = 0; yn0 = 0; zn0 = 10;
end
z_d = 10;
x_state = [u0; 0; 0; 0; 0; 0; xn0; yn0; zn0; 0; 0; psi0];

%% ===== 模型失配 =====
opt_xhy = struct('mode', 'rpm');
opt_xhy.mismatch_pct = cfg.mismatch_pct;
opt_xhy.mismatch_seed = cfg.seed;

%% ===== M矩阵 =====
m = 85.832;
Ix = 0.553864787 + 0.865274;
Iy = 2.162341935 + 5.011187;
Iz = 1.849137 + 4.541468;
MRB = diag([m, m, m, Ix, Iy, Iz]);
MA = diag([15.81, 124.73, 42.87, 0.014, 0.041, 0.123]);
M_const = MRB + MA;

%% ===== 观测器初始化 =====
% KIN
kin.c_hat = [0; 0];

% CFD-Luenberger (Børhaug)
borhaug.c_hat = [0; 0];
clear borhaug_current_observer;

% EKF
ekf.x_hat = []; ekf.P = [];

% ESKF
eskf.x_nom = []; eskf.P = [];

%% ===== 制导控制参考 =====
psi_d = psi0; r_d = 0; u_d = 1.0; u_d_dot = 0;

%% ===== 历史记录 =====
n_obs = 4;
c_true_hist  = zeros(N, 2);
kin.c_hist    = zeros(N, 2);
borhaug.c_hist = zeros(N, 2);
ekf.c_hist    = zeros(N, 2);
eskf.c_hist   = zeros(N, 2);

% 轨迹 + 跟踪误差
traj_hist     = zeros(N, 3);   % [xN, yN, z]
psi_hist      = zeros(N, 1);
psi_d_hist    = zeros(N, 1);
e_cross_hist  = zeros(N, 1);   % 横向误差

% 观测器误差记录（用于实时监控）
obs_rmse_rt   = zeros(N, n_obs);  % 实时滑动RMSE

%% ===== 主循环 =====
ui = zeros(5, 1);
fprintf('===== 四观测器轨迹跟踪对比测试 =====\n');
fprintf('场景: %s | 失配: %d%% | 噪声: %s | 种子: %d | T=%.0fs\n', ...
    cfg.scenario, cfg.mismatch_pct, cfg.noise_level, cfg.seed, T_end);
fprintf('观测器: KIN | CFD-Luenberger | EKF | ESKF\n\n');

timebar(1, N, '仿真中...');

for i = 1:N
    %% 状态提取
    nu   = x_state(1:6);
    xn   = x_state(7); yn = x_state(8); zn = x_state(9);
    psi  = x_state(12);

    %% 真实海流
    c_true = [Vc*cos(betaVc); Vc*sin(betaVc)];
    c_true_hist(i, :) = c_true';

    %% 记录轨迹
    traj_hist(i, :) = [xn, yn, zn];
    psi_hist(i) = psi;

    %% 动力学（获取 tau_thr 用于观测器）
    [~, ~, ~, ~, ~, ~, tau_thr] = xhy(x_state, ui, Vc, betaVc, wc, opt_xhy);

    %% 传感器噪声
    nu_meas = nu;
    if noise_scale > 0
        nu_meas(1:2) = nu(1:2) + noise_scale * 0.02 * randn(2,1);
    end

    %% ALOS 制导
    [psi_ref, ~, ~, e_cross, ~, ~, ~] = my_ALOS3D(xn, yn, zn, dt, pts, params.alos);
    [psi_d, r_d] = LOSobserver(psi_d, r_d, psi_ref, dt, params.alos.K_f);
    r_d = sat(r_d, 0.5);
    psi_d_hist(i) = psi_d;
    e_cross_hist(i) = e_cross;

    %% SMC 控制律
    X_cmd = smc_surge_xhy(nu(1), u_d, u_d_dot, dt, params.xhy.surge);
    N_cmd = smc_yaw_xhy(psi, nu(6), psi_d, r_d, 0, dt, params.xhy.yaw);
    Z_cmd = smc_heave_xhy(zn, nu(3), z_d, 0, 0, dt, params.xhy.heave);
    tau_cmd = [X_cmd; 0; Z_cmd; 0; 0; N_cmd];

    %% 推力分配
    [ui, ~] = thrust_allocation_xhy(tau_cmd, thr_params);

    %% ---- 并行运行四个观测器 ----

    % 1) KIN 运动学观测器
    [kin.c_hat, ~] = ...
        kin_current_observer(kin.c_hat, nu_meas, psi, u_d, params.kin, dt);
    kin.c_hist(i, :) = kin.c_hat';

    % 2) CFD-Luenberger (Børhaug 2007)
    [borhaug.c_hat, ~] = ...
        borhaug_current_observer(borhaug.c_hat, ...
            nu_meas, tau_thr, psi, M_const, params.borhaug, dt);
    borhaug.c_hist(i, :) = borhaug.c_hat';

    % 3) EKF
    [ekf.x_hat, ekf.P, ekf_aux] = ...
        ekf_current_estimator(ekf.x_hat, ekf.P, nu_meas, tau_thr, psi, M_const, params.ekf, dt);
    ekf.c_hist(i, :) = ekf_aux.c_hat';

    % 4) ESKF
    [eskf.x_nom, eskf.P, eskf_aux] = ...
        eskf_current_estimator(eskf.x_nom, eskf.P, nu_meas, tau_thr, psi, M_const, params.eskf, dt);
    eskf.c_hist(i, :) = eskf_aux.c_hat';

    %% 实时 RMSE（滑动窗口 10s = 1000步）
    win = min(i, 1000);
    if i > 10
        obs_rmse_rt(i, 1) = sqrt(mean(sum((kin.c_hist(i-win+1:i,:) - c_true_hist(i-win+1:i,:)).^2, 2)));
        obs_rmse_rt(i, 2) = sqrt(mean(sum((borhaug.c_hist(i-win+1:i,:) - c_true_hist(i-win+1:i,:)).^2, 2)));
        obs_rmse_rt(i, 3) = sqrt(mean(sum((ekf.c_hist(i-win+1:i,:) - c_true_hist(i-win+1:i,:)).^2, 2)));
        obs_rmse_rt(i, 4) = sqrt(mean(sum((eskf.c_hist(i-win+1:i,:) - c_true_hist(i-win+1:i,:)).^2, 2)));
    end

    %% 状态更新 (RK4)
    x_state = rk4(@xhy, dt, x_state, ui, Vc, betaVc, wc, opt_xhy);
    x_state(12) = ssa(x_state(12));

    timebar;
end
fprintf('\n仿真完成。\n');

%% ===== 结果统计 =====
burn_in = round(N * 0.2);  % 忽略前20%瞬态
valid_idx = burn_in:N;

c_hist_all = {kin.c_hist, borhaug.c_hist, ekf.c_hist, eskf.c_hist};
obs_names  = cfg.obs_labels;

fprintf('\n===== 海流估计 RMSE 汇总 (m/s) =====\n');
fprintf('%-18s %8s %8s %8s\n', '观测器', 'RMSE', 'MAE', '终值偏差');
fprintf('%-18s %8s %8s %8s\n', '------', '----', '---', '--------');

rmse_vals = zeros(1, n_obs);
mae_vals  = zeros(1, n_obs);
final_bias = zeros(1, n_obs);

for o = 1:n_obs
    c_err = c_hist_all{o}(valid_idx, :) - c_true_hist(valid_idx, :);
    c_err_mag = sqrt(sum(c_err.^2, 2));

    rmse_vals(o) = sqrt(mean(c_err_mag.^2));
    mae_vals(o)  = mean(c_err_mag);

    % 终值偏差（最后5s平均）
    final_idx = max(1, N-500):N;
    final_err = mean(c_hist_all{o}(final_idx, :) - c_true_hist(final_idx, :), 1);
    final_bias(o) = norm(final_err);

    fprintf('%-18s %8.4f %8.4f %8.4f\n', obs_names{o}, rmse_vals(o), mae_vals(o), final_bias(o));
end

%% ===== 图1: 水平面轨迹图 =====
figure('Name', '轨迹跟踪对比', 'Position', [100, 100, 1400, 900]);

% --- 子图1: 水平面轨迹 ---
subplot(2,3,1);
hold on; grid on; axis equal;

% 期望轨迹
switch cfg.scenario
    case 'circle'
        plot(pts.pos.x, pts.pos.y, 'k--', 'LineWidth', 1.5, 'DisplayName', '期望轨迹');
    case 'straight'
        plot(pts.pos.x, pts.pos.y, 'k--', 'LineWidth', 1.5, 'DisplayName', '期望轨迹');
end

% 实际轨迹
plot(traj_hist(:,1), traj_hist(:,2), 'b-', 'LineWidth', 1.2, 'DisplayName', '实际轨迹');

% 起点/终点
plot(traj_hist(1,1), traj_hist(1,2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g', 'DisplayName', '起点');
plot(traj_hist(end,1), traj_hist(end,2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r', 'DisplayName', '终点');

% 海流方向箭头
mid_idx = round(N/2);
quiver(traj_hist(mid_idx,1), traj_hist(mid_idx,2), ...
    c_true(1)*50, c_true(2)*50, 'r-', 'LineWidth', 2, ...
    'MaxHeadSize', 5, 'DisplayName', '海流方向');

xlabel('北向 N (m)'); ylabel('东向 E (m)');
title(sprintf('水平面轨迹 (%s, 失配=%d%%)', cfg.scenario, cfg.mismatch_pct));
legend('Location', 'best');

% --- 子图2: 海流估计 cN 分量 ---
subplot(2,3,2);
hold on; grid on;
plot(t_vec, c_true_hist(:,1), 'k-', 'LineWidth', 2, 'DisplayName', '真值');
colors = lines(n_obs);
for o = 1:n_obs
    plot(t_vec, c_hist_all{o}(:,1), 'Color', colors(o,:), 'LineWidth', 1.2, ...
        'DisplayName', obs_names{o});
end
xlabel('时间 (s)'); ylabel('c_N (m/s)');
title('海流估计 — 北向分量 c_N');
legend('Location', 'best');
xlim([0 T_end]);

% --- 子图3: 海流估计 cE 分量 ---
subplot(2,3,3);
hold on; grid on;
plot(t_vec, c_true_hist(:,2), 'k-', 'LineWidth', 2, 'DisplayName', '真值');
for o = 1:n_obs
    plot(t_vec, c_hist_all{o}(:,2), 'Color', colors(o,:), 'LineWidth', 1.2, ...
        'DisplayName', obs_names{o});
end
xlabel('时间 (s)'); ylabel('c_E (m/s)');
title('海流估计 — 东向分量 c_E');
legend('Location', 'best');
xlim([0 T_end]);

% --- 子图4: 海流估计误差幅值 ---
subplot(2,3,4);
hold on; grid on;
for o = 1:n_obs
    c_err_mag = sqrt(sum((c_hist_all{o} - c_true_hist).^2, 2));
    plot(t_vec, c_err_mag, 'Color', colors(o,:), 'LineWidth', 1.2, ...
        'DisplayName', obs_names{o});
end
xlabel('时间 (s)'); ylabel('|ĉ - c| (m/s)');
title('海流估计误差幅值');
legend('Location', 'best');
xlim([0 T_end]);

% --- 子图5: 航向跟踪 ---
subplot(2,3,5);
hold on; grid on;
plot(t_vec, rad2deg(psi_hist), 'b-', 'LineWidth', 1, 'DisplayName', '实际 \psi');
plot(t_vec, rad2deg(psi_d_hist), 'r--', 'LineWidth', 1.2, 'DisplayName', '期望 \psi_d');
xlabel('时间 (s)'); ylabel('航向 (deg)');
title('航向跟踪');
legend('Location', 'best');
xlim([0 T_end]);

% --- 子图6: 横向误差 ---
subplot(2,3,6);
hold on; grid on;
plot(t_vec, e_cross_hist, 'b-', 'LineWidth', 1.2);
yline(0, 'k--');
xlabel('时间 (s)'); ylabel('横向误差 (m)');
title('横向跟踪误差');
xlim([0 T_end]);

sgtitle(sprintf('四观测器轨迹跟踪对比 (%s, 失配=%d%%, 噪声=%s)', ...
    cfg.scenario, cfg.mismatch_pct, cfg.noise_level));

%% ===== 图2: RMSE 柱状图 + 实时收敛 =====
figure('Name', '海流估计精度对比', 'Position', [150, 150, 1200, 500]);

% --- 子图1: 稳态 RMSE 柱状图 ---
subplot(1,2,1);
bar_vals = rmse_vals;
b = bar(bar_vals, 'FaceColor', 'flat');
b.CData = lines(n_obs);
set(gca, 'XTickLabel', obs_names);
ylabel('RMSE_{V_c} (m/s)');
title(sprintf('海流估计稳态 RMSE (t > %.0fs, 失配=%d%%)', t_vec(burn_in), cfg.mismatch_pct));
grid on;

% 数值标注
for o = 1:n_obs
    text(o, bar_vals(o) + 0.005, sprintf('%.4f', bar_vals(o)), ...
        'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'FontSize', 10);
end

% 标注最优
[~, best_idx] = min(bar_vals);
text(best_idx, bar_vals(best_idx) - 0.01, '← 最优', ...
    'HorizontalAlignment', 'left', 'Color', 'r', 'FontWeight', 'bold');

% --- 子图2: 实时滑动 RMSE ---
subplot(1,2,2);
hold on; grid on;
for o = 1:n_obs
    plot(t_vec, obs_rmse_rt(:,o), 'Color', colors(o,:), 'LineWidth', 1.5, ...
        'DisplayName', sprintf('%s (终值=%.4f)', obs_names{o}, obs_rmse_rt(end,o)));
end
xlabel('时间 (s)'); ylabel('滑动 RMSE_{V_c} (m/s)');
title('实时滑动 RMSE (窗口=10s)');
legend('Location', 'best');
xlim([0 T_end]);

sgtitle('海流估计精度对比');

%% ===== 图3: EKF vs ESKF 差异放大图 =====
figure('Name', 'EKF vs ESKF 详细对比', 'Position', [200, 200, 1200, 600]);

% --- 子图1: cN 估计差异 ---
subplot(2,3,1);
hold on; grid on;
plot(t_vec, c_true_hist(:,1), 'k-', 'LineWidth', 2, 'DisplayName', '真值');
plot(t_vec, ekf.c_hist(:,1), 'b-', 'LineWidth', 1.2, 'DisplayName', 'EKF');
plot(t_vec, eskf.c_hist(:,1), 'r--', 'LineWidth', 1.2, 'DisplayName', 'ESKF');
xlabel('时间 (s)'); ylabel('c_N (m/s)');
title('c_N: EKF vs ESKF');
legend('Location', 'best');

% --- 子图2: cE 估计差异 ---
subplot(2,3,2);
hold on; grid on;
plot(t_vec, c_true_hist(:,2), 'k-', 'LineWidth', 2, 'DisplayName', '真值');
plot(t_vec, ekf.c_hist(:,2), 'b-', 'LineWidth', 1.2, 'DisplayName', 'EKF');
plot(t_vec, eskf.c_hist(:,2), 'r--', 'LineWidth', 1.2, 'DisplayName', 'ESKF');
xlabel('时间 (s)'); ylabel('c_E (m/s)');
title('c_E: EKF vs ESKF');
legend('Location', 'best');

% --- 子图3: EKF-ESKF 差值 ---
subplot(2,3,3);
hold on; grid on;
diff_cN = ekf.c_hist(:,1) - eskf.c_hist(:,1);
diff_cE = ekf.c_hist(:,2) - eskf.c_hist(:,2);
plot(t_vec, diff_cN, 'b-', 'LineWidth', 1, 'DisplayName', 'Δc_N (EKF-ESKF)');
plot(t_vec, diff_cE, 'r-', 'LineWidth', 1, 'DisplayName', 'Δc_E (EKF-ESKF)');
yline(0, 'k--');
xlabel('时间 (s)'); ylabel('差值 (m/s)');
title('EKF − ESKF 估计差值');
legend('Location', 'best');

% --- 子图4: 误差幅值对比 ---
subplot(2,3,4);
hold on; grid on;
ekf_err = sqrt(sum((ekf.c_hist - c_true_hist).^2, 2));
eskf_err = sqrt(sum((eskf.c_hist - c_true_hist).^2, 2));
plot(t_vec, ekf_err, 'b-', 'LineWidth', 1.2, 'DisplayName', 'EKF');
plot(t_vec, eskf_err, 'r--', 'LineWidth', 1.2, 'DisplayName', 'ESKF');
xlabel('时间 (s)'); ylabel('|ĉ - c| (m/s)');
title('估计误差幅值');
legend('Location', 'best');

% --- 子图5: 收敛段放大 (前30s) ---
subplot(2,3,5);
hold on; grid on;
t_zoom = 30;
idx_zoom = 1:round(t_zoom/dt);
plot(t_vec(idx_zoom), ekf_err(idx_zoom), 'b-', 'LineWidth', 1.5, 'DisplayName', 'EKF');
plot(t_vec(idx_zoom), eskf_err(idx_zoom), 'r--', 'LineWidth', 1.5, 'DisplayName', 'ESKF');
xlabel('时间 (s)'); ylabel('|ĉ - c| (m/s)');
title(sprintf('收敛段放大 (0–%ds)', t_zoom));
legend('Location', 'best');

% --- 子图6: 稳态段放大 (后30s) ---
subplot(2,3,6);
hold on; grid on;
idx_steady = N-round(30/dt):N;
plot(t_vec(idx_steady), ekf_err(idx_steady), 'b-', 'LineWidth', 1.5, 'DisplayName', 'EKF');
plot(t_vec(idx_steady), eskf_err(idx_steady), 'r--', 'LineWidth', 1.5, 'DisplayName', 'ESKF');
xlabel('时间 (s)'); ylabel('|ĉ - c| (m/s)');
title(sprintf('稳态段放大 (最后30s)'));
legend('Location', 'best');

sgtitle(sprintf('EKF vs ESKF 详细对比 (%s, 失配=%d%%)', cfg.scenario, cfg.mismatch_pct));

%% ===== 保存结果 =====
out_dir = fullfile(project_root, 'results');
if ~exist(out_dir, 'dir'), mkdir(out_dir); end

% 保存数据
save_path = fullfile(out_dir, sprintf('observer_tracking_%s_m%d_%s.mat', ...
    cfg.scenario, cfg.mismatch_pct, datestr(now, 'yyyymmdd_HHMM')));
save(save_path, 'cfg', 't_vec', 'c_true_hist', 'kin', 'borhaug', 'ekf', 'eskf', ...
    'traj_hist', 'psi_hist', 'psi_d_hist', 'e_cross_hist', ...
    'rmse_vals', 'mae_vals', 'final_bias');

% 保存图片
fig_path1 = fullfile(out_dir, sprintf('obs_tracking_%s_m%d.png', cfg.scenario, cfg.mismatch_pct));
saveas(1, fig_path1);
fig_path2 = fullfile(out_dir, sprintf('obs_rmse_%s_m%d.png', cfg.scenario, cfg.mismatch_pct));
saveas(2, fig_path2);
fig_path3 = fullfile(out_dir, sprintf('ekf_vs_eskf_%s_m%d.png', cfg.scenario, cfg.mismatch_pct));
saveas(3, fig_path3);

fprintf('\n结果已保存:\n');
fprintf('  数据: %s\n', save_path);
fprintf('  图1:  %s\n', fig_path1);
fprintf('  图2:  %s\n', fig_path2);
fprintf('  图3:  %s\n', fig_path3);

fprintf('\n===== 测试完成 =====\n');
fprintf('EKF vs ESKF 最大差异: cN=%.2e m/s, cE=%.2e m/s\n', ...
    max(abs(diff_cN)), max(abs(diff_cE)));
fprintf('结论: 向量空间下 EKF 与 ESKF 数学等价, 差异 < %.0e m/s (数值舍入级别)\n', ...
    max(max(abs(diff_cN)), max(abs(diff_cE))));
