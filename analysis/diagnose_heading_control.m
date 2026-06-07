%% 航向控制诊断测试
% 检查XHY平台的航向控制是否正常工作
clear; close all; clc;
project_root = setup_paths();

fprintf('=== XHY航向控制诊断 ===\n\n');

%% 测试1: 保持航向（无外力）
fprintf('[测试1] 保持航向0° (无海流, 100s)\n');
clear smc_yaw_xhy
params = get_params;

h = 0.01; T = 100; t = 0:h:T; N = length(t);
x = [0; 0; 0; 0; 0; 0; 0; 0; 10; 0; 0; 0];  % 初始航向0°

thr_params.rho = 1026; thr_params.D_prop = 0.10; thr_params.KT = 0.22;
thr_params.n_max = 2500; thr_params.x_vert_f = 0.344; thr_params.x_vert_r = -0.293;
thr_params.x_side_f = 0.424; thr_params.x_side_r = -0.376;

hist1.psi = zeros(N,1);
hist1.r = zeros(N,1);
hist1.N_cmd = zeros(N,1);

ui = zeros(5,1);
for i = 1:N
    psi = x(12); r = x(6);

    [~, ~, M, ~, ~, ~, ~] = xhy(x, ui, 0, 0, 0);

    % 期望航向0°，期望角速度0
    N_cmd = smc_yaw_xhy(psi, r, 0, 0, 0, h, params.xhy.yaw);
    tau_cmd = [0; 0; 0; 0; 0; N_cmd];
    ui = thrust_allocation_xhy(tau_cmd, thr_params);

    x = rk4(@xhy, h, x, ui, 0, 0, 0);
    x(12) = ssa(x(12));

    hist1.psi(i) = psi;
    hist1.r(i) = r;
    hist1.N_cmd(i) = N_cmd;
end

psi_rmse1 = sqrt(mean(hist1.psi.^2));
fprintf('  航向RMSE: %.4f rad (%.2f°)\n', psi_rmse1, rad2deg(psi_rmse1));
fprintf('  最终航向: %.4f rad (%.2f°)\n', hist1.psi(end), rad2deg(hist1.psi(end)));
fprintf('  平均力矩指令: %.3f N·m\n\n', mean(abs(hist1.N_cmd)));

%% 测试2: 转向90°（无外力）
fprintf('[测试2] 从0°转向90° (无海流, 100s)\n');
clear smc_yaw_xhy
x = [0; 0; 0; 0; 0; 0; 0; 0; 10; 0; 0; 0];

hist2.psi = zeros(N,1);
hist2.r = zeros(N,1);
hist2.N_cmd = zeros(N,1);

ui = zeros(5,1);
for i = 1:N
    psi = x(12); r = x(6);

    [~, ~, M, ~, ~, ~, ~] = xhy(x, ui, 0, 0, 0);

    % 期望航向90°
    N_cmd = smc_yaw_xhy(psi, r, pi/2, 0, 0, h, params.xhy.yaw);
    tau_cmd = [0; 0; 0; 0; 0; N_cmd];
    ui = thrust_allocation_xhy(tau_cmd, thr_params);

    x = rk4(@xhy, h, x, ui, 0, 0, 0);
    x(12) = ssa(x(12));

    hist2.psi(i) = psi;
    hist2.r(i) = r;
    hist2.N_cmd(i) = N_cmd;
end

e_psi2 = wrapToPi(hist2.psi - pi/2);
psi_rmse2 = sqrt(mean(e_psi2.^2));
fprintf('  航向误差RMSE: %.4f rad (%.2f°)\n', psi_rmse2, rad2deg(psi_rmse2));
fprintf('  最终航向: %.4f rad (%.2f°), 目标90°\n', hist2.psi(end), rad2deg(hist2.psi(end)));
fprintf('  最终误差: %.4f rad (%.2f°)\n', e_psi2(end), rad2deg(e_psi2(end)));
fprintf('  平均力矩指令: %.3f N·m\n\n', mean(abs(hist2.N_cmd)));

%% 测试3: 检查控制增益
fprintf('[测试3] 控制器参数检查\n');
fprintf('  Iz_eff: %.3f kg·m²\n', params.xhy.yaw.Iz_eff);
fprintf('  lambda: %.3f\n', params.xhy.yaw.lambda);
fprintf('  Kd: %.3f\n', params.xhy.yaw.Kd);
fprintf('  Ks: %.3f\n', params.xhy.yaw.Ks);
fprintf('  phi_b: %.3f rad (%.2f°)\n\n', params.xhy.yaw.phi_b, rad2deg(params.xhy.yaw.phi_b));

%% 诊断结论
fprintf('=== 诊断结论 ===\n');
if psi_rmse1 < deg2rad(5)
    fprintf('✅ 测试1通过: 航向保持正常\n');
else
    fprintf('❌ 测试1失败: 航向保持误差过大 (%.2f°)\n', rad2deg(psi_rmse1));
end

if abs(e_psi2(end)) < deg2rad(5)
    fprintf('✅ 测试2通过: 航向转向正常\n');
else
    fprintf('❌ 测试2失败: 航向转向误差过大 (%.2f°)\n', rad2deg(abs(e_psi2(end))));
end
