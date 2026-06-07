%% 时变海流定点控制测试 - 阶段3
% 验证PI-ESO在时变海流下的优势
clear; close all; clc;
project_root = setup_paths();

%% 测试配置
test_cases = {
    % {name, tau_c_true, tau_c_pieso, useESO, usePIESO, T}
    {'GM30s-SMC', 30, 50, 0, 0, 300};
    {'GM30s-LESO', 30, 50, 1, 0, 300};
    {'GM30s-PIESO-30s', 30, 30, 1, 1, 300};
    {'GM30s-PIESO-50s', 30, 50, 1, 1, 300};
    {'GM50s-SMC', 50, 50, 0, 0, 300};
    {'GM50s-LESO', 50, 50, 1, 0, 300};
    {'GM50s-PIESO-50s', 50, 50, 1, 1, 300};
    {'GM100s-SMC', 100, 50, 0, 0, 300};
    {'GM100s-LESO', 100, 50, 1, 0, 300};
    {'GM100s-PIESO-100s', 100, 100, 1, 1, 300};
};

target_pos = [0; 0; 10];
Vc_mean = 0.3;  % 平均海流速度
sigma_Vc = 0.1;  % 海流速度标准差

results = cell(length(test_cases), 1);

for tc = 1:length(test_cases)
    fprintf('\n=== 测试 %d/%d: %s ===\n', tc, length(test_cases), test_cases{tc}{1});

    name = test_cases{tc}{1};
    tau_c_true = test_cases{tc}{2};
    tau_c_pieso = test_cases{tc}{3};
    useESO = test_cases{tc}{4};
    usePIESO = test_cases{tc}{5};
    T = test_cases{tc}{6};

    results{tc} = run_station_keeping_gm(name, target_pos, Vc_mean, sigma_Vc, tau_c_true, tau_c_pieso, useESO, usePIESO, T);
end

%% 结果对比
fprintf('\n=== 时变海流定点控制性能对比 ===\n');
fprintf('%-20s | 位置误差(m) | 姿态误差(°) | ESO估计(N)\n', '测试');
fprintf('%s\n', repmat('-', 1, 70));

for tc = 1:length(test_cases)
    r = results{tc};
    fprintf('%-20s | %8.3f | %10.2f | %10.3f\n', ...
        test_cases{tc}{1}, r.pos_rmse, rad2deg(r.att_rmse), r.eso_mean);
end

save('results/station_keeping_timevarying_results.mat', 'results', 'test_cases');
fprintf('\n结果已保存到 results/station_keeping_timevarying_results.mat\n');

%% 辅助函数
function result = run_station_keeping_gm(name, target_pos, Vc_mean, sigma_Vc, tau_c_true, tau_c_pieso, useESO, usePIESO, T)
    clear smc_yaw_xhy smc_pitch_xhy smc_surge_xhy smc_heave_xhy vec_leso_update_adv vec_pieso_update

    params = get_params;
    if usePIESO
        params.pieso.tau_c = tau_c_pieso;
    end

    h = 0.01;
    t = 0:h:T;
    N = length(t);

    % 预生成Gauss-Markov海流序列
    gm_params.Vc_mean = Vc_mean;
    gm_params.betaVc_mean = deg2rad(45);
    gm_params.sigma_Vc = sigma_Vc;
    gm_params.tau_c = tau_c_true;
    [Vc_series, betaVc_series, ~] = gauss_markov_current(2, t, gm_params);

    % 初始状态
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
    hist.Vc = zeros(N, 1);

    ui = zeros(5,1);

    for i = 1:N
        u = x(1); v = x(2); w = x(3); r = x(6);
        xn = x(7); yn = x(8); zn = x(9);
        psi = x(12);

        % 使用预生成的Gauss-Markov海流
        Vc_state = Vc_series(i);
        betaVc_state = betaVc_series(i);

        [~, ~, M, C, D, g_vec, tau_thr] = xhy(x, ui, Vc_state, betaVc_state, 0);

        % 计算相对速度
        u_c_x = Vc_state * cos(betaVc_state - psi);
        u_c_y = Vc_state * sin(betaVc_state - psi);
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

        % 定点控制律：NED坐标系PD控制
        e_pos_NED = target_pos - [xn; yn; zn];
        Kp = 0.3;
        Kd = 0.8;
        v_NED = [u*cos(psi) - v*sin(psi); u*sin(psi) + v*cos(psi); w];
        V_d_NED = Kp * e_pos_NED - Kd * v_NED;

        u_d = V_d_NED(1)*cos(psi) + V_d_NED(2)*sin(psi);
        v_d = -V_d_NED(1)*sin(psi) + V_d_NED(2)*cos(psi);
        u_d = max(-0.5, min(1.0, u_d));
        v_d = max(-0.3, min(0.3, v_d));

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

        x = rk4(@xhy, h, x, ui, Vc_state, betaVc_state, 0);
        x(12) = ssa(x(12));

        hist.x(i,:) = x';
        hist.tau(i,:) = tau_cmd';
        hist.hat_d(i,:) = hat_d';
        hist.psi_d(i) = psi_d;
        hist.Vc(i) = Vc_state;
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
