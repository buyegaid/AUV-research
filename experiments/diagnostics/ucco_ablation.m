%% UCCO 补偿消融诊断
% 测试不同补偿配置对航向控制的影响
% 2026-06-04

function ucco_ablation()
project_root = setup_paths();
params = get_params;
h = 0.01; T = 100; t = 0:h:T; N = length(t);

% 初始状态
xn=0; yn=0; zn=10; psi0=0;
x0 = [1;0;0;0;0;0;xn;yn;zn;0;0;psi0];

ablations = {'全补偿','无补偿','仅Surge','仅Yaw','符号翻转'};
comp_flags = [1 1 1 1 0; ...  % [surge, sway, heave, roll, pitch, yaw] 全补偿
               0 0 0 0 0 0; ...  % 无补偿
               1 0 0 0 0 0; ...  % 仅Surge
               0 0 0 0 0 1; ...  % 仅Yaw
               1 0 0 0 0 -1];    % 符号翻转
results = cell(size(ablations));

gm_params = struct('Vc_mean',0.3,'betaVc_mean',pi/4,'sigma_Vc',0.1,'tau_c',100);
[Vc_seq,beta_seq,wc_seq] = gauss_markov_current(2, t, gm_params);
thr = get_thruster_params_diag();

for ab = 1:length(ablations)
    fprintf('\n=== 消融: %s ===\n', ablations{ab});
    comp_flags_ab = comp_flags(ab, :);

    x = x0; c_hat = [0;0]; Z = zeros(6,3);
    ui = zeros(5,1);
    pts = traj(50, 50);

    psi_d = psi0; r_d = 0; theta_d = 0; q_d = 0;
    u_d = 1; u_d_dot = 0; z_d = zn; w_d = 0; w_d_dot = 0;

    hist_Vc = zeros(N,1); hist_beta = zeros(N,1);
    hist_psi_err = zeros(N,1); hist_hat = zeros(N,6);

    for i = 1:N
        u=x(1); v=x(2); w=x(3); r=x(6); xn=x(7); yn=x(8); zn=x(9);
        theta=x(11); psi=x(12);

        Vc = Vc_seq(i); beta_c = beta_seq(i); wc = wc_seq(i);
        [~,~,M,~,~,~,tau_thr] = xhy(x, ui, Vc, beta_c, wc);

        % UCCO估计
        [c_hat, aux] = eg_ucco_simple(c_hat, x(1:6), tau_thr, psi, M, params.ucco, h);
        Vc_est = norm(c_hat); beta_est = atan2(c_hat(2), c_hat(1));

        % 补偿计算
        hat_d = compute_current_compensation(Vc_est, beta_est, x(1:6), psi, tau_thr, M);
        hat_d = hat_d .* comp_flags_ab';  % 消融控制

        % 制导+控制
        [psi_ref,theta_ref,~,~,~,~,~] = my_ALOS3D(xn,yn,zn,h,pts,params.alos);
        [psi_d,r_d] = LOSobserver(psi_d,r_d,psi_ref,h,params.alos.K_f); r_d=sat(r_d,0.5);
        Z_cmd = smc_heave_xhy(zn,w,z_d,w_d,w_d_dot,h,params.xhy.heave);
        X_cmd = smc_surge_xhy(u,u_d,u_d_dot,h,params.xhy.surge) - hat_d(1);
        N_cmd = smc_yaw_xhy(psi,r,psi_d,r_d,0,h,params.xhy.yaw) - hat_d(6);
        tau_cmd = [X_cmd;0;Z_cmd;0;0;N_cmd];
        [ui,~] = thrust_allocation_xhy(tau_cmd, thr);

        % 存储
        hist_Vc(i) = Vc_est; hist_beta(i) = beta_est;
        hist_psi_err(i) = psi - psi_d;
        hist_hat(i,:) = hat_d';

        % 更新
        x = rk4(@xhy, h, x, ui, Vc, beta_c, wc);
        x(12) = ssa(x(12));
    end

    cN_true = Vc_seq .* cos(beta_seq);
    cE_true = Vc_seq .* sin(beta_seq);
    cN_est = hist_Vc .* cos(hist_beta);
    cE_est = hist_Vc .* sin(hist_beta);

    results{ab}.rmse_Vc = sqrt(mean((sqrt((cN_est-cN_true).^2+(cE_est-cE_true).^2)).^2));
    results{ab}.rmse_psi = sqrt(mean(hist_psi_err.^2));
    results{ab}.rms_hat_yaw = sqrt(mean(hist_hat(:,6).^2));
    results{ab}.name = ablations{ab};

    fprintf('  RMSE_Vc=%.4f  RMSE_psi=%.4f  RMS_hat_yaw=%.4f\n', ...
        results{ab}.rmse_Vc, results{ab}.rmse_psi, results{ab}.rms_hat_yaw);
end

% 汇总
fprintf('\n===== 补偿消融结果 =====\n');
fprintf('%-12s %10s %10s %10s\n', '配置','RMSE_Vc','RMSE_psi','RMS_yaw');
for ab = 1:length(results)
    fprintf('%-12s %10.4f %10.4f %10.4f\n', ...
        results{ab}.name, results{ab}.rmse_Vc, results{ab}.rmse_psi, results{ab}.rms_hat_yaw);
end
end

function thr = get_thruster_params_diag()
thr.rho=1026; thr.D_prop_main=0.10; thr.D_prop_aux=0.06;
thr.KT_main_fwd=0.0293; thr.KT_main_rev=0.0201;
thr.KT_aux_fwd=0.327; thr.KT_aux_rev=0.327;
thr.n_max=2500; thr.x_vert_f=+0.344; thr.x_vert_r=-0.293;
thr.x_side_f=+0.424; thr.x_side_r=-0.376;
end
