%% 直线轨迹跟踪测试 - 阶段2
% 验证ESO在轨迹跟踪中的效果
clear; close all; clc;
project_root = setup_paths();

%% 测试配置
test_cases = {
    % {name, Vc, useESO, usePIESO, T}
    {'无海流基线', 0, 0, 0, 300};
    {'恒定流0.3-SMC', 0.3, 0, 0, 300};
    {'恒定流0.3-LESO', 0.3, 1, 0, 300};
    {'恒定流0.3-PIESO', 0.3, 1, 1, 300};
    {'恒定流0.5-SMC', 0.5, 0, 0, 300};
    {'恒定流0.5-LESO', 0.5, 1, 0, 300};
    {'恒定流0.5-PIESO', 0.5, 1, 1, 300};
};

% 直线轨迹: 从(0,0,10)到(100,0,10)
wpt.pos.x = [0; 100];
wpt.pos.y = [0; 0];
wpt.pos.z = [10; 10];
betaVc = deg2rad(45);

results = cell(length(test_cases), 1);

for tc = 1:length(test_cases)
    fprintf('\n=== 测试 %d/%d: %s ===\n', tc, length(test_cases), test_cases{tc}{1});

    name = test_cases{tc}{1};
    Vc = test_cases{tc}{2};
    useESO = test_cases{tc}{3};
    usePIESO = test_cases{tc}{4};
    T = test_cases{tc}{5};

    results{tc} = run_straight_line(name, wpt, Vc, betaVc, useESO, usePIESO, T);
end

%% 结果对比
fprintf('\n=== 直线轨迹跟踪性能对比 ===\n');
fprintf('%-20s | 横向误差(m) | 航向误差(°) | 到达时间(s) | ESO估计(N)\n', '测试');
fprintf('%s\n', repmat('-', 1, 80));

for tc = 1:length(test_cases)
    r = results{tc};
    fprintf('%-20s | %8.3f | %10.2f | %10.1f | %10.3f\n', ...
        test_cases{tc}{1}, r.cross_track_rmse, rad2deg(r.heading_rmse), r.arrival_time, r.eso_mean);
end

save('results/straight_line_results.mat', 'results', 'test_cases');
fprintf('\n结果已保存到 results/straight_line_results.mat\n');

%% 辅助函数
function result = run_straight_line(name, wpt, Vc, betaVc, useESO, usePIESO, T)
    clear smc_yaw_xhy smc_pitch_xhy smc_surge_xhy smc_heave_xhy vec_leso_update_adv vec_pieso_update my_ALOS3D

    params = get_params;
    params.current.Vc = Vc;
    params.current.betaVc = betaVc;

    h = 0.01;
    t = 0:h:T;
    N = length(t);

    % 初始状态（起点）
    x = [0; 0; 0; 0; 0; 0; wpt.pos.x(1); wpt.pos.y(1); wpt.pos.z(1); 0; 0; 0];
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
    wp_idx = 1;
    arrived = false;
    arrival_time = T;

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
        [psi_d, theta_d, y_e, z_e] = my_ALOS3D(xn, yn, zn, h, wpt, params.alos);

        % 期望速度（恒定）
        u_d = 1.0;

        % 期望深度（从theta_d计算或直接使用z_e反馈）
        z_d = wpt.pos.z(end);  % 目标深度

        cross_track = y_e;

        % 检查是否到达终点
        dist_to_end = sqrt((xn - wpt.pos.x(end))^2 + (yn - wpt.pos.y(end))^2 + (zn - wpt.pos.z(end))^2);
        if dist_to_end < params.alos.R_switch && ~arrived
            arrived = true;
            arrival_time = t(i);
        end

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

    % 计算性能指标
    result.cross_track_rmse = sqrt(mean(hist.cross_track.^2));
    e_psi = wrapToPi(hist.x(:, 12) - hist.psi_d);
    result.heading_rmse = sqrt(mean(e_psi.^2));
    result.arrival_time = arrival_time;
    result.eso_mean = mean(abs(hist.hat_d(:, 1)));
    result.hist = hist;
    result.name = name;
end
