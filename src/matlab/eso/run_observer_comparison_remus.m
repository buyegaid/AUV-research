function results = run_observer_comparison_remus(cfg)
% REMUS 100 海流观测器对比实验
% 适配 REMUS 动力学（remus100.m），验证跨平台泛化能力
% 2026-06-11

if nargin < 1, cfg = struct(); end

if ~isfield(cfg, 'scenario'),     cfg.scenario = 'circle'; end
if ~isfield(cfg, 'mismatch_pct'), cfg.mismatch_pct = 0; end
if ~isfield(cfg, 'observers'),    cfg.observers = {'KIN','BORHAUG','PCRCO'}; end
if ~isfield(cfg, 'noise_level'),  cfg.noise_level = 'none'; end
if ~isfield(cfg, 'seed'),         cfg.seed = 1; end
if ~isfield(cfg, 'T_end'),        cfg.T_end = 100; end
if ~isfield(cfg, 'verbose'),      cfg.verbose = true; end

rng(cfg.seed);
project_root = setup_paths();

%% 仿真参数
dt = 0.01; T_end = cfg.T_end;
t_vec = 0:dt:T_end; N = length(t_vec);

%% 海流设置
Vc = 0.3; betaVc = deg2rad(45); wc = 0;
c_true = [Vc*cos(betaVc); Vc*sin(betaVc)];

%% 参考轨迹
switch cfg.scenario
    case 'circle'
        pts = traj(20, 50);
    case 'straight'
        pts.pos.x = (0:20:1000)'; pts.pos.y = zeros(size(pts.pos.x));
        pts.pos.z = 10 * ones(size(pts.pos.x));
    case 'step'
        pts = traj(20, 50);
end

%% 参数
params = get_params();

%% REMUS M矩阵
[~, ~, M_remus] = remus100();

%% 初始状态
switch cfg.scenario
    case 'circle'
        u0 = 1.0; psi0 = pi/2; xn0 = 300; yn0 = 0; zn0 = 10;
    case 'straight'
        u0 = 1.0; psi0 = 0; xn0 = 0; yn0 = 0; zn0 = 10;
    case 'step'
        u0 = 1.0; psi0 = pi/2; xn0 = 300; yn0 = 0; zn0 = 10;
end
x_state = [u0; 0; 0; 0; 0; 0; xn0; yn0; zn0; 0; 0; psi0];

%% 观测器初始化
obs_use = ismember({'KIN','BORHAUG','PCRCO'}, cfg.observers);
if obs_use(1), kin.c_hat = [0; 0]; end
if obs_use(2), borhaug.c_hat = [0; 0]; clear borhaug_current_observer; end
if obs_use(3), ucco.c_hat = [0; 0]; clear eg_ucco_simple; end

%% 参考量
psi_d = psi0; r_d = 0; u_d = 1.0;

%% 历史记录
c_true_hist = zeros(N, 2);
if obs_use(1), kin.c_hist = zeros(N, 2); end
if obs_use(2), borhaug.c_hist = zeros(N, 2); end
if obs_use(3), ucco.c_hist = zeros(N, 2); end

%% 主循环
ui = zeros(3, 1);  % REMUS: [delta_r, delta_s, n]
if cfg.verbose
    fprintf('REMUS %s mismatch=%d%% seed=%d\n', cfg.scenario, cfg.mismatch_pct, cfg.seed);
    timebar(1, N, 'REMUS observer comparison');
end

for i = 1:N
    nu = x_state(1:6); xn = x_state(7); yn = x_state(8); zn = x_state(9);
    psi = x_state(12); theta = x_state(11);

    % 海流阶跃
    if strcmp(cfg.scenario, 'step') && t_vec(i) >= 50
        Vc_step = 0.45; betaVc_step = deg2rad(60);
    else
        Vc_step = Vc; betaVc_step = betaVc;
    end
    c_true_hist(i, :) = [Vc_step*cos(betaVc_step); Vc_step*sin(betaVc_step)]';

    % 传感器噪声
    nu_meas = nu;
    if strcmp(cfg.noise_level, 'low'), nu_meas(1:2) = nu(1:2) + 0.02*randn(2,1); end
    if strcmp(cfg.noise_level, 'high'), nu_meas(1:2) = nu(1:2) + 0.10*randn(2,1); end

    % 获取动力学矩阵
    [~, ~, ~, ~, ~, ~, tau_thr] = remus100(x_state, ui, Vc_step, betaVc_step, wc);

    % ALOS 制导
    [psi_ref, ~, ~, ~, ~, ~, ~] = my_ALOS3D(xn, yn, zn, dt, pts, params.alos);
    [psi_d, r_d] = LOSobserver(psi_d, r_d, psi_ref, dt, params.alos.K_f);
    r_d = sat(r_d, 0.5);

    % REMUS 控制律（舵角控制）
    delta_r = smc_yaw_remus(psi, nu(6), psi_d, r_d, 0, dt);
    delta_s = smc_pitch_remus(theta, nu(5), 0, 0, dt);
    n_prop = 1525 * u_d / 2.5;  % 简化推进器控制
    ui = [delta_r; delta_s; n_prop];

    % 1) KIN
    if obs_use(1)
        [kin.c_hat, ~] = kin_current_observer(kin.c_hat, nu_meas, psi, u_d, params.kin, dt);
        kin.c_hist(i, :) = kin.c_hat';
    end

    % 2) Børhaug (REMUS版 — 使用remus_drag)
    if obs_use(2)
        [borhaug.c_hat, ~] = borhaug_current_observer_remus(borhaug.c_hat, ...
            nu_meas, tau_thr, psi, M_remus, params.borhaug, dt);
        borhaug.c_hist(i, :) = borhaug.c_hat';
    end

    % 3) PCRCO (使用remus_drag+remus_coriolis_gravity)
    if obs_use(3)
        [ucco.c_hat, ~] = ucco_observer_remus(ucco.c_hat, ...
            nu_meas, tau_thr, psi, M_remus, params.ucco, dt);
        ucco.c_hist(i, :) = ucco.c_hat';
    end

    % 状态更新
    x_state = rk4(@remus100, dt, x_state, ui, Vc_step, betaVc_step, wc);
    x_state(12) = ssa(x_state(12));

    if cfg.verbose, timebar; end
end
if cfg.verbose, fprintf('\n'); end

%% 结果统计
burn_in = round(N * 0.2); valid_idx = burn_in:N;
results = struct(); results.cfg = cfg;

observer_names = {'KIN','BORHAUG','PCRCO'};
hist_fields = {kin, borhaug, ucco};

for o = 1:3
    if ~obs_use(o), continue; end
    c_err = hist_fields{o}.c_hist - c_true_hist;
    rmse = sqrt(mean(sum(c_err(valid_idx, :).^2, 2)));
    mae = mean(sqrt(sum(c_err(valid_idx, :).^2, 2)));
    results.(observer_names{o}).RMSE = rmse;
    results.(observer_names{o}).MAE = mae;
    if cfg.verbose
        fprintf('  %-8s  RMSE=%.4f  MAE=%.4f\n', observer_names{o}, rmse, mae);
    end
end
end

%% REMUS 辅助函数
function delta_r = smc_yaw_remus(psi, r, psi_d, r_d, ~, dt)
persistent psi_int
if isempty(psi_int), psi_int = 0; end
e_psi = ssa(psi - psi_d);
sigma = (r - r_d) + 0.5 * e_psi;
sat_s = tanh(sigma / 0.1);
Kd = 2; Ks = 1; Iz_eff = 5.0;
delta = Iz_eff * (0 - Kd*sigma - Ks*sat_s);
psi_int = psi_int + dt * e_psi;
psi_int = max(-1.0, min(1.0, psi_int));
delta_r = sat(delta, deg2rad(20));
end

function delta_s = smc_pitch_remus(theta, q, ~, ~, dt)
persistent theta_int
if isempty(theta_int), theta_int = 0; end
sigma = q + 0.5 * theta;
sat_s = tanh(sigma / 0.05);
Kd = 2; Ks = 1; Iy_eff = 5.0;
delta = -Iy_eff * (Kd*sigma + Ks*sat_s);
theta_int = theta_int + dt * theta;
theta_int = max(-1.0, min(1.0, theta_int));
delta_s = sat(delta, deg2rad(20));
end

%% Børhaug REMUS版
function [c_hat, aux] = borhaug_current_observer_remus(c_hat, nu_meas, tau, psi, M, params, dt)
persistent nu_prev is_init
if isempty(is_init), is_init = true; end
if isempty(nu_prev), nu_prev = nu_meas; end
if isempty(c_hat), c_hat = [0; 0]; end

u_c =  c_hat(1)*cos(psi) + c_hat(2)*sin(psi);
v_c = -c_hat(1)*sin(psi) + c_hat(2)*cos(psi);
nu_c = [u_c; v_c; 0; 0; 0; 0];
nu_r = nu_prev - nu_c;
[tau_drag, ~] = remus_drag(nu_r);
[C_nu, g_nu] = remus_coriolis_gravity(nu_r, psi);
r = nu_prev(6);
Dnu_c = [r*v_c; -r*u_c; 0; 0; 0; 0];
a_model = Dnu_c + M \ (tau + tau_drag - C_nu*nu_r - g_nu);
nu_pred = nu_prev + a_model * dt;
e_vel = nu_meas(1:2) - nu_pred(1:2);

if norm(e_vel) < 1e-5
    dc = [0; 0];
else
    dc = params.K_c .* e_vel * dt;
    dc = max(-params.max_dc*dt, min(params.max_dc*dt, dc));
end
c_hat = c_hat + dc;
alpha = exp(-dt / params.tau_c);
c_hat = alpha * c_hat + (1-alpha) * params.c_mean;
c_hat = max(-params.c_max, min(params.c_max, c_hat));
nu_prev = nu_meas;
aux.c_hat = c_hat; aux.e_vel = e_vel;
end

%% PCRCO REMUS版
function [c_hat, aux] = ucco_observer_remus(c_hat, nu_meas, tau, psi, M, params, dt)
persistent nu_prev is_init
if isempty(is_init), is_init = true; end
if isempty(nu_prev), nu_prev = nu_meas; end
if isempty(c_hat), c_hat = [0; 0]; end

u_c =  c_hat(1)*cos(psi) + c_hat(2)*sin(psi);
v_c = -c_hat(1)*sin(psi) + c_hat(2)*cos(psi);
nu_c = [u_c; v_c; 0; 0; 0; 0];
nu_r = nu_prev - nu_c;
[tau_drag, ~] = remus_drag(nu_r);
[C_nu, g_nu] = remus_coriolis_gravity(nu_r, psi);
r = nu_prev(6);
Dnu_c = [r*v_c; -r*u_c; 0; 0; 0; 0];
a_model = Dnu_c + M \ (tau + tau_drag - C_nu*nu_r - g_nu);
nu_pred = nu_prev + a_model * dt;
e_vel = nu_meas(1:2) - nu_pred(1:2);

% 数值灵敏度
delta_c = params.sens_pert;
Phi = zeros(2, 2);
for j = 1:2
    cp = c_hat; cp(j) = cp(j) + delta_c;
    u_c_p =  cp(1)*cos(psi) + cp(2)*sin(psi);
    v_c_p = -cp(1)*sin(psi) + cp(2)*cos(psi);
    nu_c_p = [u_c_p; v_c_p; 0; 0; 0; 0];
    nu_r_p = nu_prev - nu_c_p;
    [td_p, ~] = remus_drag(nu_r_p);
    [Cp, gp] = remus_coriolis_gravity(nu_r_p, psi);
    Dnc_p = [nu_prev(6)*v_c_p; -nu_prev(6)*u_c_p; 0; 0; 0; 0];
    a_p = Dnc_p + M \ (tau + td_p - Cp*nu_r_p - gp);
    nu_p = nu_prev + a_p * dt;
    Phi(:, j) = (nu_p(1:2) - nu_pred(1:2)) / delta_c;
end

% Gramian + 自适应更新
Wc = Phi' * Phi;
lambda_min = min(eig(Wc));
gamma_eff = params.K_obs * 100;
gain = gamma_eff / (1 + lambda_min * 1e6);
dc = gain * Phi' * e_vel;
dc = max(-params.max_dc, min(params.max_dc, dc));
c_hat = c_hat + dc;
alpha = exp(-dt / params.tau_c);
c_hat = alpha * c_hat + (1-alpha) * params.c_mean;
c_hat = max(-params.c_max, min(params.c_max, c_hat));
nu_prev = nu_meas;
aux.c_hat = c_hat; aux.e_vel = e_vel; aux.lambda_min = lambda_min;
end
