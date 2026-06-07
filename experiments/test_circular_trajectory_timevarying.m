%% 圆形轨迹跟踪测试 - 阶段5（时变海流）
% 验证PI-ESO在时变海流+圆形轨迹下的最终性能
clear; close all; clc;
project_root = setup_paths();

%% 测试配置
test_cases = {
    % {name, R, tau_c_true, tau_c_pieso, useESO, usePIESO, T}
    {'R100-GM30s-SMC',    100, 30,  50,  0, 0, 400};
    {'R100-GM30s-LESO',   100, 30,  50,  1, 0, 400};
    {'R100-GM30s-PIESO',  100, 30,  30,  1, 1, 400};
    {'R100-GM50s-SMC',    100, 50,  50,  0, 0, 400};
    {'R100-GM50s-LESO',   100, 50,  50,  1, 0, 400};
    {'R100-GM50s-PIESO',  100, 50,  50,  1, 1, 400};
    {'R100-GM100s-SMC',   100, 100, 100, 0, 0, 400};
    {'R100-GM100s-LESO',  100, 100, 100, 1, 0, 400};
    {'R100-GM100s-PIESO', 100, 100, 100, 1, 1, 400};
};

Vc_mean = 0.3;
sigma_Vc = 0.1;
betaVc_mean = deg2rad(45);

results = cell(length(test_cases), 1);

for tc = 1:length(test_cases)
    fprintf('\n=== 测试 %d/%d: %s ===\n', tc, length(test_cases), test_cases{tc}{1});

    name      = test_cases{tc}{1};
    R         = test_cases{tc}{2};
    tau_c_true  = test_cases{tc}{3};
    tau_c_pieso = test_cases{tc}{4};
    useESO    = test_cases{tc}{5};
    usePIESO  = test_cases{tc}{6};
    T         = test_cases{tc}{7};

    results{tc} = run_circular_gm(name, R, Vc_mean, sigma_Vc, betaVc_mean, tau_c_true, tau_c_pieso, useESO, usePIESO, T);
end

%% 结果对比
fprintf('\n=== 时变海流圆形轨迹性能对比 ===\n');
fprintf('%-22s | 横向误差(m) | 航向误差(°) | ESO估计(N)\n', '测试');
fprintf('%s\n', repmat('-', 1, 70));

for tc = 1:length(test_cases)
    r = results{tc};
    fprintf('%-22s | %8.3f | %10.2f | %10.3f\n', ...
        test_cases{tc}{1}, r.cross_track_rmse, rad2deg(r.heading_rmse), r.eso_mean);
end

% 计算改善百分比
fprintf('\n=== PI-ESO vs LESO 改善百分比 ===\n');
for tau_idx = [1 4 7]
    smc_e  = results{tau_idx}.cross_track_rmse;
    leso_e = results{tau_idx+1}.cross_track_rmse;
    pieso_e = results{tau_idx+2}.cross_track_rmse;
    tau_c = test_cases{tau_idx}{3};
    fprintf('tau_c=%ds: SMC=%.3f, LESO=%.3f (%.1f%%), PIESO=%.3f (%.1f%% vs SMC, %.1f%% vs LESO)\n', ...
        tau_c, smc_e, leso_e, (smc_e-leso_e)/smc_e*100, ...
        pieso_e, (smc_e-pieso_e)/smc_e*100, (leso_e-pieso_e)/leso_e*100);
end

save('results/circular_timevarying_results.mat', 'results', 'test_cases');
fprintf('\n结果已保存到 results/circular_timevarying_results.mat\n');

%% 辅助函数
function result = run_circular_gm(name, R, Vc_mean, sigma_Vc, betaVc_mean, tau_c_true, tau_c_pieso, useESO, usePIESO, T)
    clear smc_yaw_xhy smc_pitch_xhy smc_surge_xhy smc_heave_xhy vec_leso_update_adv vec_pieso_update my_ALOS3D

    params = get_params;
    if usePIESO
        params.pieso.tau_c = tau_c_pieso;
    end

    h = 0.01;
    t = 0:h:T;
    N = length(t);

    % 预生成Gauss-Markov海流序列
    gm_params.Vc_mean     = Vc_mean;
    gm_params.betaVc_mean = betaVc_mean;
    gm_params.sigma_Vc    = sigma_Vc;
    gm_params.tau_c       = tau_c_true;
    [Vc_series, betaVc_series, ~] = gauss_markov_current(2, t, gm_params);

    % 圆形轨迹航点（12点）
    n_wpts = 12;
    theta_wpts = linspace(0, 2*pi, n_wpts+1);
    wpt.pos.x = R * cos(theta_wpts)';
    wpt.pos.y = R * sin(theta_wpts)';
    wpt.pos.z = 10 * ones(n_wpts+1, 1);

    % 初始状态
    x = [0; 0; 0; 0; 0; 0; wpt.pos.x(1); wpt.pos.y(1); wpt.pos.z(1); 0; 0; pi/2];
    Z = zeros(6, 3);

    % 推力分配参数
    thr_params.rho = 1026;
    thr_params.D_prop = 0.10;
    thr_params.KT = 0.22;
    thr_params.n_max = 2500;
    thr_params.x_vert_f = 0.344;
    thr_params.x_vert_r = -0.293;
    thr_params.x_side_f = 0.424;
    thr_params.x_side_r = -0.376;

    hist.x = zeros(N, 12);
    hist.hat_d = zeros(N, 6);
    hist.cross_track = zeros(N, 1);
    hist.psi_d = zeros(N, 1);

    ui = zeros(5,1);

    for i = 1:N
        u = x(1); v = x(2); w = x(3); r = x(6);
        xn = x(7); yn = x(8); zn = x(9);
        psi = x(12);

        Vc_now     = Vc_series(i);
        betaVc_now = betaVc_series(i);

        [~, ~, M, C, ~, g_vec, tau_thr] = xhy(x, ui, Vc_now, betaVc_now, 0);

        % 相对速度
        u_c_x = Vc_now * cos(betaVc_now - psi);
        u_c_y = Vc_now * sin(betaVc_now - psi);
        nu_c  = [u_c_x; u_c_y; 0; 0; 0; 0];
        nu_r  = x(1:6) - nu_c;

        % ESO更新
        a_known = M \ (tau_thr - C*nu_r - g_vec);
        if usePIESO
            [Z, ~] = vec_pieso_update(Z, x(1:6), a_known, params.pieso, h);
        else
            [Z, ~] = vec_leso_update_adv(Z, x(1:6), a_known, params.eso, h);
        end

        if useESO
            hat_d = M * Z(:, 3);
        else
            hat_d = zeros(6, 1);
        end

        % ALOS制导
        [psi_d, ~, y_e, ~] = my_ALOS3D(xn, yn, zn, h, wpt, params.alos);
        u_d = 1.0;
        cross_track = y_e;

        % SMC控制律
        X_cmd = smc_surge_xhy(u, u_d, 0, h, params.xhy.surge) - hat_d(1);
        N_cmd = smc_yaw_xhy(psi, r, psi_d, 0, 0, h, params.xhy.yaw) - hat_d(6);
        Z_cmd = smc_heave_xhy(zn, w, 10, 0, 0, h, params.xhy.heave) - hat_d(3);

        tau_cmd = [X_cmd; 0; Z_cmd; 0; 0; N_cmd];
        ui = thrust_allocation_xhy(tau_cmd, thr_params);

        x = rk4(@xhy, h, x, ui, Vc_now, betaVc_now, 0);
        x(12) = ssa(x(12));

        hist.x(i,:) = x';
        hist.hat_d(i,:) = hat_d';
        hist.cross_track(i) = cross_track;
        hist.psi_d(i) = psi_d;
    end

    result.cross_track_rmse = sqrt(mean(hist.cross_track.^2));
    e_psi = wrapToPi(hist.x(:, 12) - hist.psi_d);
    result.heading_rmse = sqrt(mean(e_psi.^2));
    result.eso_mean = mean(abs(hist.hat_d(:, 1)));
    result.hist = hist;
    result.name = name;
end
