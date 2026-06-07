%% 定点控制测试 - 验证ESO补偿效果
% 从最简单场景开始，逐步验证控制系统和ESO
clear; close all; clc;
project_root = setup_paths();

%% 测试配置
test_cases = {
    % {name, Vc, useESO, usePIESO, T}
    {'无海流基线', 0, 0, 0, 200};
    {'恒定流-SMC', 0.3, 0, 0, 200};
    {'恒定流-LESO', 0.3, 1, 0, 200};
    {'恒定流-PIESO', 0.3, 1, 1, 200};
};

target_pos = [0; 0; 10];  % 目标位置 (NED)
betaVc = deg2rad(45);

results = cell(length(test_cases), 1);

for tc = 1:length(test_cases)
    fprintf('\n=== 测试 %d/%d: %s ===\n', tc, length(test_cases), test_cases{tc}{1});

    name = test_cases{tc}{1};
    Vc = test_cases{tc}{2};
    useESO = test_cases{tc}{3};
    usePIESO = test_cases{tc}{4};
    T = test_cases{tc}{5};

    results{tc} = run_station_keeping(name, target_pos, Vc, betaVc, useESO, usePIESO, T);
end

%% 结果对比
fprintf('\n=== 定点控制性能对比 ===\n');
fprintf('%-20s | 位置误差(m) | 姿态误差(°) | ESO估计(N)\n', '测试');
fprintf('%s\n', repmat('-', 1, 70));

for tc = 1:length(test_cases)
    r = results{tc};
    fprintf('%-20s | %8.3f | %10.2f | %10.3f\n', ...
        test_cases{tc}{1}, r.pos_rmse, rad2deg(r.att_rmse), r.eso_mean);
end

save('results/station_keeping_results.mat', 'results', 'test_cases');
fprintf('\n结果已保存到 results/station_keeping_results.mat\n');

%% 辅助函数
function result = run_station_keeping(name, target_pos, Vc, betaVc, useESO, usePIESO, T)
    clear smc_yaw_xhy smc_pitch_xhy smc_surge_xhy smc_heave_xhy vec_leso_update_adv vec_pieso_update

    params = get_params;
    params.current.Vc = Vc;
    params.current.betaVc = betaVc;

    h = 0.01;
    t = 0:h:T;
    N = length(t);

    % 初始状态（远离目标）
    x = [0.5; 0; 0; 0; 0; 0; 5; 5; 12; 0; 0; 0];
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
    hist.psi_d = zeros(N, 1);

    ui = zeros(5,1);

    for i = 1:N
        u = x(1); v = x(2); w = x(3); r = x(6);
        xn = x(7); yn = x(8); zn = x(9);
        psi = x(12);

        [~, ~, M, C, D, g_vec, tau_thr] = xhy(x, ui, Vc, betaVc, 0);

        % 计算相对速度（与海流的相对速度）
        u_c_x = Vc * cos(betaVc - psi);
        u_c_y = Vc * sin(betaVc - psi);
        nu_c = [u_c_x; u_c_y; 0; 0; 0; 0];
        nu_r = x(1:6) - nu_c;

        % ESO更新（使用相对速度，匹配实际动力学）
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

        % 定点控制律：NED坐标系下的PD控制
        e_pos_NED = target_pos - [xn; yn; zn];

        % NED坐标系下的期望速度（PD控制）
        Kp = 0.3;
        Kd = 0.8;
        v_NED = [u*cos(psi) - v*sin(psi); u*sin(psi) + v*cos(psi); w];
        V_d_NED = Kp * e_pos_NED - Kd * v_NED;

        % 转换到体坐标系
        u_d = V_d_NED(1)*cos(psi) + V_d_NED(2)*sin(psi);
        v_d = -V_d_NED(1)*sin(psi) + V_d_NED(2)*cos(psi);
        u_d = max(-0.5, min(1.0, u_d));
        v_d = max(-0.3, min(0.3, v_d));

        % 期望航向：与期望速度方向对齐
        if sqrt(V_d_NED(1)^2 + V_d_NED(2)^2) > 0.1
            psi_d = atan2(V_d_NED(2), V_d_NED(1));
        else
            psi_d = psi;
        end

        X_cmd = smc_surge_xhy(u, u_d, 0, h, params.xhy.surge) - hat_d(1);
        N_cmd = smc_yaw_xhy(psi, r, psi_d, 0, 0, h, params.xhy.yaw) - hat_d(6);
        Z_cmd = smc_heave_xhy(zn, w, target_pos(3), 0, 0, h, params.xhy.heave) - hat_d(3);

        tau_cmd = [X_cmd; 0; Z_cmd; 0; 0; N_cmd];
        ui = thrust_allocation_xhy(tau_cmd, thr_params);

        x = rk4(@xhy, h, x, ui, Vc, betaVc, 0);
        x(12) = ssa(x(12));

        hist.x(i,:) = x';
        hist.tau(i,:) = tau_cmd';
        hist.hat_d(i,:) = hat_d';
        hist.psi_d(i) = psi_d;
    end

    % 计算性能指标（最后100s稳态）
    steady_idx = max(1, N-10000):N;
    e_pos = hist.x(steady_idx, 7:9) - target_pos';
    result.pos_rmse = sqrt(mean(sum(e_pos.^2, 2)));
    e_psi = wrapToPi(hist.x(steady_idx, 12) - hist.psi_d(steady_idx));
    result.att_rmse = sqrt(mean(e_psi.^2));
    result.eso_mean = mean(abs(hist.hat_d(steady_idx, 1)));
    result.hist = hist;
    result.name = name;
end
