%% SMC vs LESO vs PI-ESO 对比实验
% 对比三种方法在海流扰动下的轨迹跟踪性能
clear; close all; clc;

% 添加路径
project_root = setup_paths();

%% 实验配置
T = 300;  % 仿真时间 (s)
Vc = 0.5; % 海流速度 (m/s) - 增大以使扰动更显著
betaVc = deg2rad(45); % 海流方向

fprintf('=== SMC vs LESO vs PI-ESO 对比实验 ===\n');
fprintf('仿真时间: %d s\n', T);
fprintf('海流速度: %.2f m/s, 方向: %.1f°\n\n', Vc, rad2deg(betaVc));

%% 实验1: 纯SMC (无ESO)
fprintf('[1/3] 运行纯SMC...\n');
hist_smc = run_xhy_experiment(0, 0, T, Vc, betaVc);

%% 实验2: SMC + LESO
fprintf('[2/3] 运行SMC+LESO...\n');
hist_leso = run_xhy_experiment(1, 0, T, Vc, betaVc);

%% 实验3: SMC + PI-ESO
fprintf('[3/3] 运行SMC+PI-ESO...\n');
hist_pieso = run_xhy_experiment(1, 1, T, Vc, betaVc);

%% 性能指标计算
fprintf('\n=== 性能指标 ===\n');
metrics_smc = compute_metrics(hist_smc);
metrics_leso = compute_metrics(hist_leso);
metrics_pieso = compute_metrics(hist_pieso);

fprintf('航向误差RMSE (rad):\n');
fprintf('  SMC:     %.4f\n', metrics_smc.psi_rmse);
fprintf('  LESO:    %.4f (%.1f%%)\n', metrics_leso.psi_rmse, ...
    (metrics_leso.psi_rmse - metrics_smc.psi_rmse)/metrics_smc.psi_rmse*100);
fprintf('  PI-ESO:  %.4f (%.1f%%)\n', metrics_pieso.psi_rmse, ...
    (metrics_pieso.psi_rmse - metrics_smc.psi_rmse)/metrics_smc.psi_rmse*100);

fprintf('\n横向误差RMSE (m):\n');
fprintf('  SMC:     %.4f\n', metrics_smc.y_rmse);
fprintf('  LESO:    %.4f (%.1f%%)\n', metrics_leso.y_rmse, ...
    (metrics_leso.y_rmse - metrics_smc.y_rmse)/metrics_smc.y_rmse*100);
fprintf('  PI-ESO:  %.4f (%.1f%%)\n', metrics_pieso.y_rmse, ...
    (metrics_pieso.y_rmse - metrics_smc.y_rmse)/metrics_smc.y_rmse*100);

%% 保存结果
save('results/comparison_smc_pieso.mat', 'hist_smc', 'hist_leso', 'hist_pieso', ...
    'metrics_smc', 'metrics_leso', 'metrics_pieso');
fprintf('\n结果已保存到 results/comparison_smc_pieso.mat\n');

%% 辅助函数
function hist = run_xhy_experiment(useESO, usePIESO, T, Vc, betaVc)
    % 运行单次XHY实验
    clear smc_yaw_xhy smc_pitch_xhy smc_surge_xhy smc_heave_xhy my_ALOS3D vec_leso_update_adv vec_pieso_update

    params = get_params;
    params.current.Vc = Vc;
    params.current.betaVc = betaVc;

    h = 0.01;
    t = 0:h:T;
    N = length(t);

    % 初始状态
    x = [1; 0; 0; 0; 0; 0; 0; 0; 10; 0; 0; 0];
    Z = zeros(6, 3);

    % 轨迹
    pts = traj(50, 50);

    % 推力分配参数
    thr_params.rho = 1026;
    thr_params.D_prop = 0.10;
    thr_params.KT = 0.22;
    thr_params.n_max = 2500;
    thr_params.x_vert_f = +0.344;
    thr_params.x_vert_r = -0.293;
    thr_params.x_side_f = +0.424;
    thr_params.x_side_r = -0.376;

    % 历史数据
    hist.x = zeros(N, 12);
    hist.ui = zeros(N, 5);
    hist.tau = zeros(N, 6);
    hist.Z = zeros(N, 18);  % ESO状态 [Z1(6) Z2(6) Z3(6)]
    hist.hat_d = zeros(N, 6);  % ESO扰动估计

    ui = zeros(5,1);
    psi_d = 0; r_d = 0;

    % 诊断输出
    fprintf('  useESO=%d, usePIESO=%d\n', useESO, usePIESO);

    for i = 1:N
        u = x(1); w = x(3); q = x(5); r = x(6);
        xn = x(7); yn = x(8); zn = x(9);
        theta = x(11); psi = x(12);

        [~, ~, M, C, D, g_vec, tau_thr] = xhy(x, ui, Vc, betaVc, 0);

        u_c_x = Vc * cos(betaVc - psi);
        u_c_y = Vc * sin(betaVc - psi);
        nu_c = [u_c_x; u_c_y; 0; 0; 0; 0];
        nu_r = x(1:6) - nu_c;

        % a_known使用绝对速度，让ESO估计海流扰动
        a_known = M \ (tau_thr - C*x(1:6) - D*x(1:6) - g_vec);
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

        [psi_ref, ~, ~, ~, ~, ~, ~] = my_ALOS3D(xn, yn, zn, h, pts, params.alos);
        [psi_d, r_d] = LOSobserver(psi_d, r_d, psi_ref, h, params.alos.K_f);

        X_cmd = smc_surge_xhy(u, 1, 0, h, params.xhy.surge) - hat_d(1);
        N_cmd = smc_yaw_xhy(psi, r, psi_d, r_d, 0, h, params.xhy.yaw) - hat_d(6);
        M_cmd = 0;
        Z_cmd = smc_heave_xhy(zn, w, 10, 0, 0, h, params.xhy.heave) - hat_d(3);

        tau_cmd = [X_cmd; 0; Z_cmd; 0; M_cmd; N_cmd];
        ui = thrust_allocation_xhy(tau_cmd, thr_params);

        x = rk4(@xhy, h, x, ui, Vc, betaVc, 0);
        x(12) = ssa(x(12));

        hist.x(i,:) = x';
        hist.ui(i,:) = ui';
        hist.tau(i,:) = tau_cmd';
        hist.Z(i,:) = [Z(:,1)', Z(:,2)', Z(:,3)'];
        hist.hat_d(i,:) = hat_d';
    end

    hist.t = t;
    hist.traj = pts;
end

function metrics = compute_metrics(hist)
    % 计算性能指标
    psi = hist.x(:, 12);
    xn = hist.x(:, 7);
    yn = hist.x(:, 8);
    N = length(xn);

    % 航向误差和横向误差
    e_psi = zeros(N, 1);
    e_y = zeros(N, 1);

    for i = 1:N
        % 找到轨迹上最近的点
        [~, idx] = min((hist.traj.pos.x - xn(i)).^2 + (hist.traj.pos.y - yn(i)).^2);

        % 航向误差
        psi_ref = atan2(hist.traj.pos.y(idx) - yn(i), hist.traj.pos.x(idx) - xn(i));
        e_psi(i) = wrapToPi(psi(i) - psi_ref);

        % 横向误差（垂直于路径方向）
        if idx < length(hist.traj.pos.x)
            path_angle = atan2(hist.traj.pos.y(idx+1) - hist.traj.pos.y(idx), ...
                               hist.traj.pos.x(idx+1) - hist.traj.pos.x(idx));
        else
            path_angle = atan2(hist.traj.pos.y(idx) - hist.traj.pos.y(idx-1), ...
                               hist.traj.pos.x(idx) - hist.traj.pos.x(idx-1));
        end
        e_y(i) = -(xn(i) - hist.traj.pos.x(idx)) * sin(path_angle) + ...
                  (yn(i) - hist.traj.pos.y(idx)) * cos(path_angle);
    end

    metrics.psi_rmse = sqrt(mean(e_psi.^2));
    metrics.y_rmse = sqrt(mean(e_y.^2));
end
