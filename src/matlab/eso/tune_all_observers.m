function results = tune_all_observers(method)
% TUNE_ALL_OBSERVERS  三观测器独立调参扫描
%
% 用法:
%   tune_all_observers('CFDL')   % CFD-Luenberger: 扫描 K_c
%   tune_all_observers('EKF')    % EKF: 扫描 Q_c × R0
%   tune_all_observers('PCRCO')  % PC-RCO: 扫描 K_obs × max_dc
%   tune_all_observers('all')    % 全部依次运行
%
% 2026-06-23

if nargin < 1, method = 'all'; end

project_root = setup_paths();
params = get_params();

% 公共配置
dt = 0.01; T_end = 100; N = round(T_end/dt) + 1;
Vc = 0.3; betaVc = deg2rad(45);
c_true = [Vc*cos(betaVc); Vc*sin(betaVc)];
n_seeds = 3;  % 扫描用3 seeds加速

% M矩阵 + 推进器
M_const = compute_M_constant();
thr = get_thr_params_0622();

fprintf('===== 观测器调参扫描 =====\n');
fprintf('配置: 圆形轨迹, 0%%失配, low noise, %d seeds\n\n', n_seeds);

switch method
    case {'CFDL','all'}
        tune_cfd_luenberger(params, M_const, thr, Vc, betaVc, c_true, T_end, dt, n_seeds);
end

if strcmp(method, 'all'), fprintf('\n'); end

switch method
    case {'EKF','all'}
        tune_ekf(params, M_const, thr, Vc, betaVc, c_true, T_end, dt, n_seeds);
end

if strcmp(method, 'all'), fprintf('\n'); end

switch method
    case {'PCRCO','all'}
        tune_pcrco(params, M_const, thr, Vc, betaVc, c_true, T_end, dt, n_seeds);
end

fprintf('\n===== 全部扫描完成 =====\n');
end

%% ==================== CFD-Luenberger: K_c 扫描 ====================
function tune_cfd_luenberger(params, M_const, thr, Vc, betaVc, c_true, T_end, dt, n_seeds)
fprintf('--- CFD-Luenberger: K_c 扫描 ---\n');
K_c_vals = [10, 20, 40, 80, 160, 320];
best_rmse = inf; best_kc = 80;

for ki = 1:length(K_c_vals)
    kc = K_c_vals(ki);
    rmse_arr = zeros(1, n_seeds);
    for s = 1:n_seeds
        rng(s);
        rmse_arr(s) = run_one_cfdl(kc, params, M_const, thr, Vc, betaVc, c_true, T_end, dt, s);
    end
    fprintf('  K_c=[%d;%d]: RMSE=%.4f±%.4f\n', kc, kc, mean(rmse_arr), std(rmse_arr));
    if mean(rmse_arr) < best_rmse, best_rmse = mean(rmse_arr); best_kc = kc; end
end
fprintf('  最优: K_c=[%d;%d] (RMSE=%.4f)\n', best_kc, best_kc, best_rmse);
end

function rmse = run_one_cfdl(kc, params, M_const, thr, Vc, betaVc, c_true, T_end, dt, seed)
N = round(T_end/dt) + 1; rng(seed);
params.borhaug.K_c = [kc; kc];
clear borhaug_current_observer;
c_hat = [0;0]; x_state = init_circle_state();
opt_xhy = struct('mode','rpm','mismatch_pct',0,'mismatch_seed',seed);
ui = init_control(); psi_d = pi/2; r_d = 0;
for i = 1:N
    [~, ~, ~, ~, nu_meas, tau_thr, ui, psi_d, r_d] = sim_step(x_state, ui, Vc, betaVc, 0, opt_xhy, thr, params, dt, psi_d, r_d);
    [c_hat, ~] = borhaug_current_observer(c_hat, nu_meas, tau_thr, x_state(12), M_const, params.borhaug, dt);
    x_state = rk4(@xhy, dt, x_state, ui, Vc, betaVc, 0, opt_xhy); x_state(12) = ssa(x_state(12));
end
rmse = norm(c_hat - c_true);
end

%% ==================== EKF: Q_c × R0 扫描 ====================
function tune_ekf(params, M_const, thr, Vc, betaVc, c_true, T_end, dt, n_seeds)
fprintf('--- EKF: Q_c × R0 扫描 ---\n');
Q_c_vals = [5e-5, 1e-4, 5e-4, 1e-3, 5e-3, 1e-2];
R0_vals = [4e-4, 1e-3, 4e-3, 1e-2, 4e-2, 1e-1];
best_rmse = inf; best_qc = 5e-4; best_r0 = 1e-2;

for qi = 1:length(Q_c_vals)
    for ri = 1:length(R0_vals)
        qc = Q_c_vals(qi); r0 = R0_vals(ri);
        rmse_arr = zeros(1, n_seeds);
        for s = 1:n_seeds
            rng(s);
            rmse_arr(s) = run_one_ekf(qc, r0, params, M_const, thr, Vc, betaVc, c_true, T_end, dt, s);
        end
        fprintf('  Q_c=%.0e R0=%.0e: RMSE=%.4f±%.4f\n', qc, r0, mean(rmse_arr), std(rmse_arr));
        if mean(rmse_arr) < best_rmse, best_rmse = mean(rmse_arr); best_qc = qc; best_r0 = r0; end
    end
end
fprintf('  最优: Q_c=%.0e R0=%.0e (RMSE=%.4f)\n', best_qc, best_r0, best_rmse);
end

function rmse = run_one_ekf(qc, r0, params, M_const, thr, Vc, betaVc, c_true, T_end, dt, seed)
N = round(T_end/dt) + 1; rng(seed);
params.ekf.Q_c = qc; params.ekf.R0 = r0;
x_hat = []; P = [];
x_state = init_circle_state();
opt_xhy = struct('mode','rpm','mismatch_pct',0,'mismatch_seed',seed);
ui = init_control(); psi_d = pi/2; r_d = 0;
for i = 1:N
    [~, ~, ~, ~, nu_meas, tau_thr, ui, psi_d, r_d] = sim_step(x_state, ui, Vc, betaVc, 0, opt_xhy, thr, params, dt, psi_d, r_d);
    [x_hat, P, aux] = ekf_current_estimator(x_hat, P, nu_meas, tau_thr, x_state(12), M_const, params.ekf, dt);
    x_state = rk4(@xhy, dt, x_state, ui, Vc, betaVc, 0, opt_xhy); x_state(12) = ssa(x_state(12));
end
rmse = norm(aux.c_hat - c_true);
end

%% ==================== PC-RCO: K_obs × max_dc 扫描 ====================
function tune_pcrco(params, M_const, thr, Vc, betaVc, c_true, T_end, dt, n_seeds)
fprintf('--- PC-RCO: K_obs × max_dc 扫描 ---\n');
K_vals = [5, 8, 10, 12, 15, 20];
dc_vals = [0.04, 0.06, 0.08, 0.10, 0.15, 0.20];
best_rmse = inf; best_k = 10; best_dc = 0.06;

for ki = 1:length(K_vals)
    for di = 1:length(dc_vals)
        kob = K_vals(ki); mdc = dc_vals(di);
        rmse_arr = zeros(1, n_seeds);
        for s = 1:n_seeds
            rng(s);
            rmse_arr(s) = run_one_pcrco(kob, mdc, params, M_const, thr, Vc, betaVc, c_true, T_end, dt, s);
        end
        fprintf('  K_obs=%.0f max_dc=%.2f: RMSE=%.4f±%.4f\n', kob, mdc, mean(rmse_arr), std(rmse_arr));
        if mean(rmse_arr) < best_rmse, best_rmse = mean(rmse_arr); best_k = kob; best_dc = mdc; end
    end
end
fprintf('  最优: K_obs=%.0f max_dc=%.2f (RMSE=%.4f)\n', best_k, best_dc, best_rmse);
end

function rmse = run_one_pcrco(kob, mdc, params, M_const, thr, Vc, betaVc, c_true, T_end, dt, seed)
N = round(T_end/dt) + 1; rng(seed);
params.ucco.K_obs = kob; params.ucco.max_dc = mdc;
clear eg_ucco_simple;
c_hat = [0;0]; x_state = init_circle_state();
opt_xhy = struct('mode','rpm','mismatch_pct',0,'mismatch_seed',seed);
ui = init_control(); psi_d = pi/2; r_d = 0;
for i = 1:N
    [~, ~, ~, ~, nu_meas, tau_thr, ui, psi_d, r_d] = sim_step(x_state, ui, Vc, betaVc, 0, opt_xhy, thr, params, dt, psi_d, r_d);
    [c_hat, ~] = eg_ucco_simple(c_hat, nu_meas, tau_thr, x_state(12), M_const, params.ucco, dt);
    x_state = rk4(@xhy, dt, x_state, ui, Vc, betaVc, 0, opt_xhy); x_state(12) = ssa(x_state(12));
end
rmse = norm(c_hat - c_true);
end

%% ==================== 共享辅助函数 ====================
function x_state = init_circle_state()
x_state = [1.0; 0; 0; 0; 0; 0; 300; 0; 10; 0; 0; pi/2];
end

function ui = init_control()
ui = zeros(5, 1);
end

function [nu, psi, Vc_now, betaVc_now, nu_meas, tau_thr, ui, psi_d, r_d] = sim_step(x_state, ui, Vc, betaVc, wc, opt_xhy, thr, params, dt, psi_d, r_d)
nu = x_state(1:6); xn = x_state(7); yn = x_state(8); zn = x_state(9); psi = x_state(12);
Vc_now = Vc; betaVc_now = betaVc;
nu_meas = nu + 0.02 * randn(6, 1);  % low noise
[~, ~, ~, ~, ~, ~, tau_thr] = xhy(x_state, ui, Vc_now, betaVc_now, wc, opt_xhy);

% 制导+控制
pts = traj(20, 50);
[psi_ref, ~, ~, ~, ~, ~, ~] = my_ALOS3D(xn, yn, zn, dt, pts, params.alos);
[psi_d, r_d] = LOSobserver(psi_d, r_d, psi_ref, dt, params.alos.K_f);
r_d = sat(r_d, 0.5);
X_cmd = smc_surge_xhy(nu(1), 1.0, 0, dt, params.xhy.surge);
N_cmd = smc_yaw_xhy(psi, nu(6), psi_d, r_d, 0, dt, params.xhy.yaw);
Z_cmd = smc_heave_xhy(zn, nu(3), 10, 0, 0, dt, params.xhy.heave);
tau_cmd = [X_cmd; 0; Z_cmd; 0; 0; N_cmd];
[ui, ~] = thrust_allocation_xhy(tau_cmd, thr);
end

function M = compute_M_constant()
m = 85.832;
Ix = 0.553864787 + 0.865274; Iy = 2.162341935 + 5.011187; Iz = 1.849137 + 4.541468;
MRB = diag([m,m,m,Ix,Iy,Iz]); MA = diag([15.81,124.73,42.87,0.014,0.041,0.123]); M = MRB + MA;
end

function p = get_thr_params_0622()
p.rho = 1026; p.D_prop_main = 0.08; p.D_prop_aux = 0.06;
p.KT_main_fwd = 0.1489; p.KT_main_rev = 0.0506; p.KT_aux_fwd = 0.53; p.KT_aux_rev = 0.71;
p.n_max = 2500; p.x_vert_f = +0.344; p.x_vert_r = -0.293; p.x_side_f = +0.424; p.x_side_r = -0.376;
end
