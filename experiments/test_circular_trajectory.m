%% 圆形轨迹跟踪测试 - 阶段4（恒定海流）
% 验证ESO在圆形轨迹下的效果
clear; close all; clc;
project_root = setup_paths();

%% 测试配置
test_cases = {
    % {name, R, Vc, useESO, usePIESO, T}
    {'R50-无海流', 50, 0, 0, 0, 300};
    {'R50-SMC', 50, 0.3, 0, 0, 300};
    {'R50-LESO', 50, 0.3, 1, 0, 300};
    {'R50-PIESO', 50, 0.3, 1, 1, 300};
    {'R100-无海流', 100, 0, 0, 0, 400};
    {'R100-SMC', 100, 0.3, 0, 0, 400};
    {'R100-LESO', 100, 0.3, 1, 0, 400};
    {'R100-PIESO', 100, 0.3, 1, 1, 400};
};

betaVc = deg2rad(45);

results = cell(length(test_cases), 1);

for tc = 1:length(test_cases)
    fprintf('\n=== 测试 %d/%d: %s ===\n', tc, length(test_cases), test_cases{tc}{1});

    name = test_cases{tc}{1};
    R = test_cases{tc}{2};
    Vc = test_cases{tc}{3};
    useESO = test_cases{tc}{4};
    usePIESO = test_cases{tc}{5};
    T = test_cases{tc}{6};

    results{tc} = run_circular(name, R, Vc, betaVc, useESO, usePIESO, T);
end

%% 结果对比
fprintf('\n=== 圆形轨迹跟踪性能对比 ===\n');
fprintf('%-18s | 横向误差(m) | 航向误差(°) | ESO估计(N)\n', '测试');
fprintf('%s\n', repmat('-', 1, 65));

for tc = 1:length(test_cases)
    r = results{tc};
    fprintf('%-18s | %8.3f | %10.2f | %10.3f\n', ...
        test_cases{tc}{1}, r.cross_track_rmse, rad2deg(r.heading_rmse), r.eso_mean);
end

save('results/circular_results.mat', 'results', 'test_cases');
fprintf('\n结果已保存到 results/circular_results.mat\n');

%% 辅助函数
function result = run_circular(name, R, Vc, betaVc, useESO, usePIESO, T)
    clear smc_yaw_xhy smc_pitch_xhy smc_surge_xhy smc_heave_xhy vec_leso_update_adv vec_pieso_update my_ALOS3D

    params = get_params;
    params.current.Vc = Vc;
    params.current.betaVc = betaVc;

    h = 0.01;
    t = 0:h:T;
    N = length(t);

    % 生成圆形轨迹航点（12个点，确保间距>R_switch=10m）
    n_wpts = 12;
    theta_wpts = linspace(0, 2*pi, n_wpts+1);
    wpt.pos.x = R * cos(theta_wpts)';
    wpt.pos.y = R * sin(theta_wpts)';
    wpt.pos.z = 10 * ones(n_wpts+1, 1);

    % 初始状态（圆形轨迹起点）
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
    hist.tau = zeros(N, 6);
    hist.hat_d = zeros(N, 6);
    hist.cross_track = zeros(N, 1);
    hist.psi_d = zeros(N, 1);

    ui = zeros(5,1);

    for i = 1:N
        u = x(1); v = x(2); w = x(3); r = x(6);
        xn = x(7); yn = x(8); zn = x(9);
        psi = x(12);

        [~, ~, M, C, D, g_vec, tau_thr] = xhy(x, ui, Vc, betaVc, 0);

        % 计算相对速度
        u_c_x = Vc * cos(betaVc - psi);
        u_c_y = Vc * sin(betaVc - psi);
        nu_c = [u_c_x; u_c_y; 0; 0; 0; 0];
        nu_r = x(1:6) - nu_c;

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
        z_d = 10;
        cross_track = y_e;

        % SMC控制律
        X_cmd = smc_surge_xhy(u, u_d, 0, h, params.xhy.surge) - hat_d(1);
        N_cmd = smc_yaw_xhy(psi, r, psi_d, 0, 0, h, params.xhy.yaw) - hat_d(6);
        Z_cmd = smc_heave_xhy(zn, w, z_d, 0, 0, h, params.xhy.heave) - hat_d(3);

        tau_cmd = [X_cmd; 0; Z_cmd; 0; 0; N_cmd];
        ui = thrust_allocation_xhy(tau_cmd, thr_params);

        x = rk4(@xhy, h, x, ui, Vc, betaVc, 0);
        x(12) = ssa(x(12));

        hist.x(i,:) = x';
        hist.tau(i,:) = tau_cmd';
        hist.hat_d(i,:) = hat_d';
        hist.cross_track(i) = cross_track;
        hist.psi_d(i) = psi_d;
    end

    % 计算性能指标（全程）
    result.cross_track_rmse = sqrt(mean(hist.cross_track.^2));
    e_psi = wrapToPi(hist.x(:, 12) - hist.psi_d);
    result.heading_rmse = sqrt(mean(e_psi.^2));
    result.eso_mean = mean(abs(hist.hat_d(:, 1)));
    result.hist = hist;
    result.name = name;
    result.R = R;
    result.wpt = wpt;
end
