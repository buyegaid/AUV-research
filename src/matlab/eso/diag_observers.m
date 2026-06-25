%% 诊断：各观测器时程输出
% 检查KIN是否真的收敛、Børhaug为何崩塌
% 2026-06-10

project_root = setup_paths();

%% KIN 诊断
fprintf('===== KIN 诊断 =====\n');
cfg = struct('scenario', 'circle', 'mismatch_pct', 0, ...
             'observers', {{'KIN'}}, ...
             'noise_level', 'none', 'seed', 1, 'T_end', 100, 'verbose', false);

% 修改 run_observer_comparison 返回时程
% 这里手动跑一个简化版
dt = 0.01; T_end = 100; N = T_end/dt + 1;
Vc = 0.3; betaVc = deg2rad(45);
c_true = [Vc*cos(betaVc); Vc*sin(betaVc)];

params = get_params();
thr_params = get_thr_params();
M_const = compute_M_constant();

% 圆形轨迹
pts = traj(20, 50);
x_state = [1.0; 0; 0; 0; 0; 0; 300; 0; 10; 0; 0; pi/2];
ui = zeros(5,1);
psi_d = pi/2; r_d = 0; u_d = 1.0;

% KIN 初始化
kin.Vc_hat = 0; kin.beta_hat = 0; kin.x_hat = [300; 0; 10];

c_hist = zeros(N,2);
e_hist = zeros(N,2);
x_hist = zeros(N,3);

for i = 1:N
    nu = x_state(1:6); xn = x_state(7); yn = x_state(8); zn = x_state(9); psi = x_state(12);

    [~, ~, ~, ~, ~, ~, tau_thr] = xhy(x_state, ui, Vc, betaVc, 0);

    [psi_ref, ~, ~, ~, ~, ~, ~] = my_ALOS3D(xn, yn, zn, dt, pts, params.alos);
    [psi_d, r_d] = LOSobserver(psi_d, r_d, psi_ref, dt, params.alos.K_f);
    r_d = sat(r_d, 0.5);

    X_cmd = smc_surge_xhy(nu(1), u_d, 0, dt, params.xhy.surge);
    N_cmd = smc_yaw_xhy(psi, nu(6), psi_d, r_d, 0, dt, params.xhy.yaw);
    Z_cmd = smc_heave_xhy(zn, nu(3), 10, 0, 0, dt, params.xhy.heave);
    tau_cmd = [X_cmd; 0; Z_cmd; 0; 0; N_cmd];
    [ui, ~] = thrust_allocation_xhy(tau_cmd, thr_params);

    [kin.Vc_hat, kin.beta_hat, kin.x_hat, kin_aux] = ...
        kin_current_observer(kin.Vc_hat, kin.beta_hat, kin.x_hat, ...
            nu, [xn;yn;zn], psi, 0, params.kin, dt);

    c_hist(i,:) = [kin.Vc_hat*cos(kin.beta_hat), kin.Vc_hat*sin(kin.beta_hat)];
    e_hist(i,:) = kin_aux.eta_err(1:2)';
    x_hist(i,:) = kin_aux.x_hat';

    x_state = rk4(@xhy, dt, x_state, ui, Vc, betaVc, 0);
    x_state(12) = ssa(x_state(12));
end

fprintf('KIN 最终估计: Vc=%.4f, beta=%.1f°, c=[%.4f,%.4f]\n', ...
    kin.Vc_hat, rad2deg(kin.beta_hat), c_hist(end,1), c_hist(end,2));
fprintf('KIN 位置误差终值: [%.4f, %.4f] m\n', e_hist(end,1), e_hist(end,2));
fprintf('KIN 位置估计终值: [%.1f, %.1f, %.1f]\n', x_hist(end,:));
fprintf('KIN 真实位置终值: [%.1f, %.1f, %.1f]\n', xn, yn, zn);

%% 计算RMSE
burn = round(N*0.2);
c_err = c_hist - [c_true'; c_true'];
c_err = c_err .* [c_true'; c_true'];  % Hmm, this is wrong
c_err = c_hist - repmat(c_true', N, 1);
rmse = sqrt(mean(sum(c_err(burn:end,:).^2, 2)));
fprintf('KIN RMSE: %.4f (K3=%.4f, K4=%.4f, lpf_alpha=%.4f)\n', ...
    rmse, params.kin.K3, params.kin.K4, params.kin.lpf_alpha);

%% 结论建议
fprintf('\n===== 建议 =====\n');
fprintf('KIN K3=%.4f → 建议增大到0.5-1.0加快收敛\n', params.kin.K3);
fprintf('Børhaug 直线崩塌 → 需要加入死区或门控防止错误更新\n');

function thr_params = get_thr_params()
thr_params.rho = 1026;
thr_params.D_prop_main = 0.08; thr_params.D_prop_aux = 0.06;
thr_params.KT_main_fwd = 0.1489; thr_params.KT_main_rev = 0.0506;
thr_params.KT_aux_fwd = 0.53; thr_params.KT_aux_rev = 0.71;  % 0622分段PWM
thr_params.n_max = 2500;
thr_params.x_vert_f = +0.344; thr_params.x_vert_r = -0.293;
thr_params.x_side_f = +0.424; thr_params.x_side_r = -0.376;
end

function M = compute_M_constant()
m = 85.832;
Ix = 0.553864787 + 0.865274; Iy = 2.162341935 + 5.011187; Iz = 1.849137 + 4.541468;
MRB = diag([m, m, m, Ix, Iy, Iz]);
MA = diag([15.81, 124.73, 42.87, 0.014, 0.041, 0.123]);
M = MRB + MA;
end
