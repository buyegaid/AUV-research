function results = run_gate_gm_stress()
% RUN_GATE_GM_STRESS  Gate+GM衰减专项压力测试
%   回应 Round 1 审查: gate_open=100% → GM decay从未激活
%   设计: 超长直线(80s) + 偶尔转弯 → 迫使gate关闭 → 验证GM
%   2026-06-22

project_root = setup_paths();
params = get_params();

%% ===== 实验配置 =====
T_end = 150;  % 150s仿真
dt = 0.01;
N = round(T_end / dt) + 1;
n_seeds = 5;

% 海流: 恒定0.3 m/s
Vc = 0.3; betaVc = deg2rad(45);
c_true = [Vc*cos(betaVc); Vc*sin(betaVc)];

% 航点: 直线80s北上 → 转弯 → 直线40s东向 → 转弯
% 间距20m > R_switch=10m
u0 = 1.0;
wp_spacing = 20;
n_pts_north = max(2, floor(u0 * 80 / wp_spacing));
n_pts_east = max(2, floor(u0 * 40 / wp_spacing));

wp_x = [linspace(0, u0*80, n_pts_north), ...
        u0*80 * ones(1, 20), ...  % 转弯段
        linspace(u0*80, u0*80+u0*40, n_pts_east)];
wp_y = [zeros(1, n_pts_north), ...
        linspace(0, u0*40, 20), ...
        u0*40 * ones(1, n_pts_east)];
wp_z = 10 * ones(1, length(wp_x));

pts.pos.x = wp_x'; pts.pos.y = wp_y'; pts.pos.z = wp_z';

fprintf('===== Gate+GM 压力测试 =====\n');
fprintf('航点: 直线80s(N) → 转弯 → 直线40s(E)\n');
fprintf('海流: Vc=%.1f, βc=%.0f°\n\n', Vc, rad2deg(betaVc));

% 门控阈值扫描
gate_mu_vals = [1e-8, 1e-7, 1e-6, 5e-6, 1e-5, 5e-5];
variants = {'PCRCO','NoGM','NoGate'};  % NoGate: 永远梯度更新（无GM+无gate）

results = struct();
results.cfg.gate_mu_vals = gate_mu_vals;
results.cfg.variants = variants;
results.cfg.T_end = T_end;
results.cfg.n_seeds = n_seeds;

for gi = 1:length(gate_mu_vals)
    gm = gate_mu_vals(gi);
    fprintf('--- gate_mu=%.1e ---\n', gm);

    % 临时修改gate_mu
    params_orig = params.ucco.gate_mu;
    params.ucco.gate_mu = gm;

    for vi = 1:length(variants)
        vn = variants{vi};
        gate_pct_arr = zeros(1, n_seeds);
        rmse_arr = zeros(1, n_seeds);
        dc_max_arr = zeros(1, n_seeds);

        for s = 1:n_seeds
            rng(s);
            [gate_pct, rmse, dc_m] = run_single_gate_test(...
                vn, gm, params, pts, u0, Vc, betaVc, c_true, T_end, dt, s);
            gate_pct_arr(s) = gate_pct;
            rmse_arr(s) = rmse;
            dc_max_arr(s) = dc_m;
        end

        key = sprintf('gm%.0e_%s', gm, vn);
        results.(key).gate_pct_mean = mean(gate_pct_arr);
        results.(key).gate_pct_std = std(gate_pct_arr);
        results.(key).rmse_mean = mean(rmse_arr);
        results.(key).rmse_std = std(rmse_arr);
        results.(key).dc_max_mean = mean(dc_max_arr);

        fprintf('  %-8s gate=%.1f%%±%.1f%%  RMSE=%.4f±%.4f  dc_max=%.4f\n', ...
            vn, mean(gate_pct_arr)*100, std(gate_pct_arr)*100, ...
            mean(rmse_arr), std(rmse_arr), mean(dc_max_arr));
    end

    % 恢复
    params.ucco.gate_mu = params_orig;
end

%% ===== 汇总 =====
fprintf('\n===== Gate扫描汇总 =====\n');
fprintf('%-10s %-10s %-12s %-12s %-12s\n', 'gate_mu', '变体', 'gate%%', 'RMSE', 'dc_max');
for gi = 1:length(gate_mu_vals)
    gm = gate_mu_vals(gi);
    for vi = 1:length(variants)
        vn = variants{vi};
        key = sprintf('gm%.0e_%s', gm, vn);
        fprintf('%-10.0e %-10s %-11.1f%% %-12.4f %-12.4f\n', ...
            gm, vn, results.(key).gate_pct_mean*100, ...
            results.(key).rmse_mean, results.(key).dc_max_mean);
    end
end

save(fullfile(project_root, 'results', 'gate_gm_stress.mat'), 'results');
fprintf('\n结果已保存\n');
end

%% ==================== 单次测试 ====================
function [gate_pct, rmse, dc_max] = run_single_gate_test(...
    variant_name, gate_mu, params, pts, u0, Vc, betaVc, c_true, T_end, dt, seed)
rng(seed);

N = round(T_end / dt) + 1;

% 初始状态
x_state = [u0; 0; 0; 0; 0; 0; 0; 0; 10; 0; 0; 0];  % 北上

% 惯性矩阵
m = 85.832; Ix = 1.419; Iy = 7.174; Iz = 6.391;
MRB = diag([m,m,m,Ix,Iy,Iz]);
MA = diag([15.81, 124.73, 42.87, 0.014, 0.041, 0.123]);
M_const = MRB + MA;

% 推进器
thr_params.rho = 1026; thr_params.D_prop_main = 0.08; thr_params.D_prop_aux = 0.06;
thr_params.KT_main_fwd = 0.0567; thr_params.KT_main_rev = 0.0235;
thr_params.KT_aux_fwd = 1.2531; thr_params.KT_aux_rev = 0.6193;  % 0616非饱和
thr_params.n_max = 2500;
thr_params.x_vert_f = +0.344; thr_params.x_vert_r = -0.293;
thr_params.x_side_f = +0.424; thr_params.x_side_r = -0.376;

opt_xhy = struct('mode', 'rpm', 'mismatch_pct', 0, 'mismatch_seed', seed);

% 初始化
psi_d = 0; r_d = 0; ui = zeros(5, 1);
c_hat = [0; 0];

% 重置观测器
switch variant_name
    case 'PCRCO', clear eg_ucco_simple;
    case 'NoGM',  pcrco_no_gm([],[],[],[],[],[],[],true);
    case 'NoGate'
        clear eg_ucco_simple;  % 用PCRCO但跳过gate逻辑
end

% 临时修改gate_mu
params.ucco.gate_mu = gate_mu;

c_hist = zeros(N, 2);
gate_hist = zeros(N, 1);
dc_hist = zeros(N, 1);

for i = 1:N
    nu = x_state(1:6); xn = x_state(7); yn = x_state(8); zn = x_state(9);
    psi = x_state(12);

    % 低噪声
    nu_meas = nu + 0.02 * randn(6,1);

    % 动力学
    [~, ~, M, ~, ~, ~, tau_thr] = xhy(x_state, ui, Vc, betaVc, 0, opt_xhy);

    % 制导 + 控制
    [psi_ref, ~, ~, ~, ~, ~, ~] = my_ALOS3D(xn, yn, zn, dt, pts, params.alos);
    [psi_d, r_d] = LOSobserver(psi_d, r_d, psi_ref, dt, params.alos.K_f);
    r_d = sat(r_d, 0.5);
    X_cmd = smc_surge_xhy(nu(1), u0, 0, dt, params.xhy.surge);
    N_cmd = smc_yaw_xhy(psi, nu(6), psi_d, r_d, 0, dt, params.xhy.yaw);
    Z_cmd = smc_heave_xhy(zn, nu(3), 10, 0, 0, dt, params.xhy.heave);
    tau_cmd = [X_cmd; 0; Z_cmd; 0; 0; N_cmd];
    [ui, ~] = thrust_allocation_xhy(tau_cmd, thr_params);

    % 观测器
    ch_prev = c_hat;
    switch variant_name
        case 'PCRCO'
            [c_hat, aux] = eg_ucco_simple(c_hat, nu_meas, tau_thr, psi, M_const, params.ucco, dt);
        case 'NoGM'
            [c_hat, aux] = pcrco_no_gm(c_hat, nu_meas, tau_thr, psi, M_const, params.ucco, dt);
        case 'NoGate'
            % 永远使用梯度更新（跳过gate和GM）
            core = pcrco_core(c_hat, nu_meas, nu_meas, tau_thr, psi, M_const, params.ucco, dt);
            gamma_eff = params.ucco.K_obs * 100;
            gain = gamma_eff / (1 + core.lambda_min * 1e6);
            dc = gain * core.Phi' * core.e_vel;
            dc = max(-params.ucco.max_dc, min(params.ucco.max_dc, dc));
            c_hat = c_hat + dc;
            c_hat = max(-params.ucco.c_max, min(params.ucco.c_max, c_hat));
            aux.excited = true;
            aux.lambda_min = core.lambda_min;
    end

    c_hist(i, :) = c_hat';
    gate_hist(i) = aux.excited;
    dc_hist(i) = norm(c_hat - ch_prev);

    % 状态更新
    x_state = rk4(@xhy, dt, x_state, ui, Vc, betaVc, 0, opt_xhy);
    x_state(12) = ssa(x_state(12));
end

% 指标
c_err = c_hist - repmat(c_true', N, 1);
gate_pct = mean(gate_hist);
rmse = sqrt(mean(sum(c_err.^2, 2)));
dc_max = max(dc_hist);
end
