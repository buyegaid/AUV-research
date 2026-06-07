%% 快速测试: SMC vs PI-ESO
clear; close all; clc;
project_root = setup_paths();

fprintf('快速测试 (30s仿真)\n');

% 测试PI-ESO
clear smc_yaw_xhy smc_pitch_xhy smc_surge_xhy smc_heave_xhy my_ALOS3D vec_pieso_update
params = get_params;
params.current.Vc = 0.3;
params.current.betaVc = deg2rad(45);

h = 0.01; T = 30; t = 0:h:T; N = length(t);
x = [1; 0; 0; 0; 0; 0; 0; 0; 10; 0; 0; 0];
Z = zeros(6, 3);
pts = traj(50, 50);

thr_params.rho = 1026; thr_params.D_prop = 0.10; thr_params.KT = 0.22;
thr_params.n_max = 2500; thr_params.x_vert_f = 0.344; thr_params.x_vert_r = -0.293;
thr_params.x_side_f = 0.424; thr_params.x_side_r = -0.376;

ui = zeros(5,1); psi_d = 0; r_d = 0;

for i = 1:N
    u = x(1); xn = x(7); yn = x(8); zn = x(9); psi = x(12); r = x(6); w = x(3);

    [~, ~, M, C, D, g_vec, tau_thr] = xhy(x, ui, 0.3, deg2rad(45), 0);
    nu_c = [0.3*cos(deg2rad(45)-psi); 0.3*sin(deg2rad(45)-psi); 0; 0; 0; 0];
    nu_r = x(1:6) - nu_c;
    a_known = M \ (tau_thr - C*nu_r - D*nu_r - g_vec);

    [Z, ~] = vec_pieso_update(Z, x(1:6), a_known, params.pieso, h);
    hat_d = M * Z(:, 3);

    [psi_ref, ~, ~, ~, ~, ~, ~] = my_ALOS3D(xn, yn, zn, h, pts, params.alos);
    [psi_d, r_d] = LOSobserver(psi_d, r_d, psi_ref, h, params.alos.K_f);

    X_cmd = smc_surge_xhy(u, 1, 0, h, params.xhy.surge) - hat_d(1);
    N_cmd = smc_yaw_xhy(psi, r, psi_d, r_d, 0, h, params.xhy.yaw) - hat_d(6);
    Z_cmd = smc_heave_xhy(zn, w, 10, 0, 0, h, params.xhy.heave) - hat_d(3);

    tau_cmd = [X_cmd; 0; Z_cmd; 0; 0; N_cmd];
    ui = thrust_allocation_xhy(tau_cmd, thr_params);
    x = rk4(@xhy, h, x, ui, 0.3, deg2rad(45), 0);
    x(12) = ssa(x(12));
end

fprintf('✅ PI-ESO测试完成\n');
fprintf('最终位置: (%.2f, %.2f, %.2f)\n', x(7), x(8), x(9));
fprintf('最终航向: %.2f°\n', rad2deg(x(12)));
