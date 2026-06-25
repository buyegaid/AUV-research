function results = run_observer_comparison(cfg)
% 海流观测器对比实验 — 4基线 × 3场景 × 4失配水平
%
% 用法:
%   cfg = struct('scenario', 'circle', 'mismatch_pct', 20, ...
%               'observers', {{'KIN','BORHAUG','EKF','UCCO'}}, ...
%               'noise_level', 'none', 'seed', 1, 'T_end', 100);
%   result = run_observer_comparison(cfg);
%
% 或批量运行:
%   results = run_observer_comparison('batch');
%
% 2026-06-10

%% ===== 默认配置 =====
if nargin < 1 || isempty(cfg)
    cfg = struct();
end

if ischar(cfg) && strcmp(cfg, 'batch')
    % 批量模式：运行所有实验
    results = run_batch_all();
    return;
end

% 默认值
if ~isfield(cfg, 'scenario'),     cfg.scenario = 'circle'; end
if ~isfield(cfg, 'mismatch_pct'), cfg.mismatch_pct = 0; end
if ~isfield(cfg, 'observers'),    cfg.observers = {'KIN','BORHAUG','EKF','UCCO'}; end
if ~isfield(cfg, 'noise_level'),  cfg.noise_level = 'none'; end
if ~isfield(cfg, 'seed'),         cfg.seed = 1; end
if ~isfield(cfg, 'T_end'),        cfg.T_end = 100; end
if ~isfield(cfg, 'verbose'),      cfg.verbose = true; end

%% ===== 初始化 =====
if cfg.verbose
    fprintf('===== 海流观测器对比实验 =====\n');
    fprintf('场景: %s, 失配: %d%%, 噪声: %s, 种子: %d\n', ...
        cfg.scenario, cfg.mismatch_pct, cfg.noise_level, cfg.seed);
end

% 随机种子
rng(cfg.seed);

% 项目路径
project_root = setup_paths();

%% ===== 仿真参数 =====
dt = 0.01;
T_end = cfg.T_end;
t_vec = 0:dt:T_end;
N = length(t_vec);

%% ===== 海流设置 =====
switch cfg.scenario
    case 'circle'
        Vc     = 0.3;
        betaVc = deg2rad(45);
        wc     = 0;
    case 'straight'
        Vc     = 0.3;
        betaVc = deg2rad(45);
        wc     = 0;
    case 'step'
        Vc_init  = 0.15;
        betaVc_init = deg2rad(30);
        Vc_final = 0.45;
        betaVc_final = deg2rad(60);
        wc = 0;
        % 阶跃: 前50s用初始值，50s后切换到最终值
        Vc = Vc_init;
        betaVc = betaVc_init;
    otherwise
        Vc = 0.3; betaVc = deg2rad(45); wc = 0;
end

c_true_N0 = Vc * cos(betaVc);
c_true_E0 = Vc * sin(betaVc);

%% ===== 参考轨迹 =====
switch cfg.scenario
    case 'circle'
        pts = traj(20, 50);  % 弧长20m，50点，半径~300m
    case 'straight'
        % 直线轨迹: 沿N方向
        pts.pos.x = (0:20:1000)';
        pts.pos.y = zeros(size(pts.pos.x));
        pts.pos.z = 10 * ones(size(pts.pos.x));
    case 'step'
        pts = traj(20, 50);  % 圆形 + 50s时海流阶跃
end

%% ===== 参数 =====
params = get_params();

% 噪声设置
switch cfg.noise_level
    case 'none'
        noise_scale = 0;
    case 'low'
        noise_scale = 1;   % DVL σ=0.02 m/s
    case 'high'
        noise_scale = 5;   % DVL σ=0.10 m/s
end

%% ===== 推进器参数 (2026-06-22 分段PWM反推, 与 thrust_main/thrust_aux 保持一致) =====
thr_params.rho         = 1026;
thr_params.D_prop_main = 0.08;     % 主推直径 8cm
thr_params.D_prop_aux  = 0.06;
thr_params.KT_main_fwd = 0.1489;   % 0616分段PWM, 前进100% 2250RPM
thr_params.KT_main_rev = 0.0506;   % 0616分段PWM, 后退80% 2150RPM
thr_params.KT_aux_fwd  = 0.53;     % 0616分段PWM, 左移100% 1370RPM
thr_params.KT_aux_rev  = 0.71;     % 0616分段PWM, 右移100% 1681RPM
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
    case 'step'
        u0 = 1.0; psi0 = pi/2; xn0 = 300; yn0 = 0; zn0 = 10;
end
z_d = 10;
x_state = [u0; 0; 0; 0; 0; 0; xn0; yn0; zn0; 0; 0; psi0];

%% ===== 模型失配配置 =====
opt_xhy = struct('mode', 'rpm');
opt_xhy.mismatch_pct = cfg.mismatch_pct;
opt_xhy.mismatch_seed = cfg.seed;  % 失配随机模式固定为seed

%% ===== M矩阵（常数） =====
M_const = compute_M_constant();

%% ===== 观测器初始化 =====
% KIN
kin.c_hat = [0; 0];

% Børhaug
borhaug.c_hat = [0; 0];

% EKF
ekf.x_hat = []; ekf.P = [];

% UCCO
ucco.c_hat = [0; 0];
clear eg_ucco_simple  % 清除persistent
clear borhaug_current_observer

%% ===== 参考量 =====
psi_d = psi0; r_d = 0; u_d = 1.0; u_d_dot = 0;

%% ===== 历史记录（仅记录关键量以省内存） =====
c_true_hist = zeros(N, 2);
obs_use = ismember({'KIN','BORHAUG','EKF','UCCO'}, cfg.observers);
if obs_use(1), kin.c_hist = zeros(N, 2); end
if obs_use(2), borhaug.c_hist = zeros(N, 2); end
if obs_use(3), ekf.c_hist = zeros(N, 2); end
if obs_use(4), ucco.c_hist = zeros(N, 2); end

%% ===== 主循环 =====
ui = zeros(5, 1);
timebar_params = struct('draw', cfg.verbose);
if cfg.verbose, timebar(1, N, sprintf('%s mismatch=%d%% seed=%d', cfg.scenario, cfg.mismatch_pct, cfg.seed)); end

for i = 1:N
    % 状态提取
    nu   = x_state(1:6);
    xn   = x_state(7); yn = x_state(8); zn = x_state(9);
    psi  = x_state(12);

    % 海流阶跃
    if strcmp(cfg.scenario, 'step') && t_vec(i) >= 50
        Vc = Vc_final; betaVc = betaVc_final;
    end
    c_true = [Vc*cos(betaVc); Vc*sin(betaVc)];
    c_true_hist(i, :) = c_true';

    % 模型失配：扰动 CFD 阻力系数（plant使用扰动后的阻力）
    % 通过修改 xhy.m 中的 CFD 参数或直接在外部做
    % 这里通过对真实的 tau_drag 加扰动来模拟
    [~, ~, M, C, D, g_vec, tau_thr] = xhy(x_state, ui, Vc, betaVc, wc, opt_xhy);

    % 添加传感器噪声
    nu_meas = nu;
    if noise_scale > 0
        nu_meas(1:2) = nu(1:2) + noise_scale * 0.02 * randn(2,1);  % DVL噪声
    end
    eta_meas = [xn; yn; zn];  % 位置测量（简化为真值）

    % ALOS 制导
    [psi_ref, ~, ~, ~, ~, ~, ~] = my_ALOS3D(xn, yn, zn, dt, pts, params.alos);
    [psi_d, r_d] = LOSobserver(psi_d, r_d, psi_ref, dt, params.alos.K_f);
    r_d = sat(r_d, 0.5);

    % 控制律（surge + yaw + heave）
    X_cmd = smc_surge_xhy(nu(1), u_d, u_d_dot, dt, params.xhy.surge);
    N_cmd = smc_yaw_xhy(psi, nu(6), psi_d, r_d, 0, dt, params.xhy.yaw);
    Z_cmd = smc_heave_xhy(zn, nu(3), z_d, 0, 0, dt, params.xhy.heave);
    tau_cmd = [X_cmd; 0; Z_cmd; 0; 0; N_cmd];

    % 推力分配
    [ui, ~] = thrust_allocation_xhy(tau_cmd, thr_params);

    % ---- 运行各观测器 ----

    % 1) KIN 运动学观测器（速度残差: DVL对地 vs 命令对水）
    if obs_use(1)
        [kin.c_hat, kin_aux] = ...
            kin_current_observer(kin.c_hat, nu_meas, psi, u_d, params.kin, dt);
        kin.c_hist(i, :) = kin.c_hat';
    end

    % 2) Børhaug 2007 观测器（速度预测型，模型基PI）
    if obs_use(2)
        [borhaug.c_hat, borhaug_aux] = ...
            borhaug_current_observer(borhaug.c_hat, ...
                nu_meas, tau_thr, psi, M_const, params.borhaug, dt);
        borhaug.c_hist(i, :) = borhaug.c_hat';
    end

    % 3) EKF 观测器
    if obs_use(3)
        [ekf.x_hat, ekf.P, ekf_aux] = ...
            ekf_current_estimator(ekf.x_hat, ekf.P, nu_meas, tau_thr, psi, M_const, params.ekf, dt);
        ekf.c_hist(i, :) = ekf_aux.c_hat';
    end

    % 4) UCCO 观测器
    if obs_use(4)
        [ucco.c_hat, ucco_aux] = ...
            eg_ucco_simple(ucco.c_hat, nu_meas, tau_thr, psi, M_const, params.ucco, dt);
        ucco.c_hist(i, :) = ucco.c_hat';
    end

    % 状态更新
    x_state = rk4(@xhy, dt, x_state, ui, Vc, betaVc, wc, opt_xhy);
    x_state(12) = ssa(x_state(12));

    if cfg.verbose, timebar; end
end
if cfg.verbose, fprintf('\n'); end

%% ===== 结果统计 =====
% 忽略初始瞬态（前20%数据）
burn_in = round(N * 0.2);
valid_idx = burn_in:N;

% 阶跃场景：分前半和后半
if strcmp(cfg.scenario, 'step')
    half_N = round(N/2);
    valid_idx_pre  = burn_in:half_N;
    valid_idx_post = half_N+1:N;
end

results = struct();
results.cfg = cfg;

observer_names = {'KIN','BORHAUG','EKF','UCCO'};
hist_fields    = {kin, borhaug, ekf, ucco};

for o = 1:4
    if ~obs_use(o), continue; end

    c_hist_data = hist_fields{o}.c_hist;

    % RMSE
    if strcmp(cfg.scenario, 'step')
        c_err = c_hist_data - c_true_hist;
        rmse_pre  = sqrt(mean(sum(c_err(valid_idx_pre, :).^2, 2)));
        rmse_post = sqrt(mean(sum(c_err(valid_idx_post, :).^2, 2)));
        rmse = (rmse_pre + rmse_post) / 2;
    else
        c_err = c_hist_data - c_true_hist;
        rmse = sqrt(mean(sum(c_err(valid_idx, :).^2, 2)));
    end

    % MAE
    c_err_vec = sqrt(sum(c_err(valid_idx, :).^2, 2));
    mae = mean(c_err_vec);

    results.(observer_names{o}).RMSE = rmse;
    results.(observer_names{o}).MAE  = mae;
    results.(observer_names{o}).c_err_std = std(c_err_vec);
end

if cfg.verbose
    fprintf('\n===== 结果汇总 =====\n');
    for o = 1:4
        if ~obs_use(o), continue; end
        fprintf('  %-8s  RMSE=%.4f  MAE=%.4f m/s\n', ...
            observer_names{o}, results.(observer_names{o}).RMSE, results.(observer_names{o}).MAE);
    end
end

end

%% ==================== 批量运行 ====================
function all_results = run_batch_all()
% 运行完整实验矩阵: 4 baseline × 3 scenario × 4 mismatch × 2 seeds

scenarios   = {'circle', 'straight', 'step'};
mismatches  = [0, 10, 20, 30];
observers   = {'KIN','BORHAUG','EKF','UCCO'};
n_seeds     = 2;
noise_level = 'none';

fprintf('===== 批量实验矩阵 =====\n');
fprintf('  场景: %s\n', strjoin(scenarios, ', '));
fprintf('  失配: %s %%\n', strjoin(cellstr(num2str(mismatches')), ', '));
fprintf('  观测器: %s\n', strjoin(observers, ', '));
fprintf('  种子数: %d\n', n_seeds);
fprintf('  噪声: %s\n', noise_level);
fprintf('========================\n\n');

n_total = length(scenarios) * length(mismatches) * n_seeds;
n_done = 0;
t_start = tic;

all_results = cell(length(scenarios), length(mismatches), n_seeds);

for si = 1:length(scenarios)
    for mi = 1:length(mismatches)
        for s = 1:n_seeds
            n_done = n_done + 1;
            cfg = struct('scenario', scenarios{si}, ...
                         'mismatch_pct', mismatches(mi), ...
                         'observers', {observers}, ...
                         'noise_level', noise_level, ...
                         'seed', s, ...
                         'T_end', 100, ...
                         'verbose', false);

            t_elapsed = toc(t_start);
            eta_str = '';
            if n_done > 1
                eta_s = t_elapsed / (n_done-1) * (n_total - n_done);
                eta_str = sprintf(' | ETA: %.0fs', eta_s);
            end

            fprintf('[%2d/%2d] %-8s 失配=%2d%%  seed=%d (%.1fs elapsed%s)\n', ...
                n_done, n_total, scenarios{si}, mismatches(mi), s, t_elapsed, eta_str);

            all_results{si, mi, s} = run_observer_comparison(cfg);
        end
    end
end

%% ===== 汇总表格 =====
fprintf('\n===== 海流估计精度汇总 (RMSE_Vc, m/s) =====\n\n');

for si = 1:length(scenarios)
    scenario_name = scenarios{si};
    fprintf('--- %s ---\n', scenario_name);
    fprintf('%-8s', '失配');
    for o = 1:length(observers)
        fprintf(' %-10s', observers{o});
    end
    fprintf('\n');

    for mi = 1:length(mismatches)
        fprintf('%-8s', sprintf('%d%%', mismatches(mi)));
        for o = 1:length(observers)
            % 平均2个种子
            rmse_vals = zeros(1, n_seeds);
            for s = 1:n_seeds
                r = all_results{si, mi, s};
                if ~isempty(r) && isfield(r, observers{o})
                    rmse_vals(s) = r.(observers{o}).RMSE;
                end
            end
            rmse_mean = mean(rmse_vals);
            fprintf(' %-10.4f', rmse_mean);
        end
        fprintf('\n');
    end
    fprintf('\n');
end

fprintf('\n===== 全部实验完成 =====\n');
end

%% ==================== 辅助函数 ====================
function M = compute_M_constant()
% XHY常数惯性矩阵
m = 85.832;
Ix = 0.553864787 + 0.865274;
Iy = 2.162341935 + 5.011187;
Iz = 1.849137 + 4.541468;
MRB = diag([m, m, m, Ix, Iy, Iz]);
MA = diag([15.81, 124.73, 42.87, 0.014, 0.041, 0.123]);
M = MRB + MA;
end
