%% ESO诊断脚本 - 运行50步，打印关键量
addpath('.', './Lib', './guidance', './controller/xhy', './model', './eso', './post', './traj');
clear vec_leso_update_adv  % 清除persistent变量，避免上次运行残留

params = get_params;
params.pieso = params.eso;
params.pieso.tau_c = 50;

h = 0.01; Vc = 0.5; betaVc = pi/4; wc = 0;
R_traj = 300; zn = 10; psi0 = pi/2;
x = [1; 0; 0; 0; 0; 0; R_traj; 0; zn; 0; 0; psi0];
ui = zeros(5,1);
Z = zeros(6,3);
Z(:,1) = x(1:6);  % 初始化Z1为初始速度，消除初始瞬态

fprintf('step | e_surge | Z3_surge | hat_d1 | Z3_yaw | hat_d6 | N_smc | N_cmd\n');
fprintf('-----|---------|----------|--------|--------|--------|-------|------\n');

for i = 1:200
    psi = x(12);
    u_c_x = Vc * cos(betaVc - psi);
    u_c_y = Vc * sin(betaVc - psi);
    nu_c  = [u_c_x; u_c_y; wc; 0; 0; 0];
    nu_r  = x(1:6) - nu_c;

    [~, ~, M, C, D, g_vec, tau_thr] = xhy(x, ui, Vc, betaVc, wc);
    a_known = M \ (tau_thr - C*x(1:6) - D*x(1:6) - g_vec);

    e = x(1:6) - Z(:,1);
    [Z, ~] = vec_leso_update_adv(Z, x(1:6), a_known, params.eso, h);
    hat_d = M * Z(:,3);

    % 简单yaw SMC
    N_smc = params.xhy.yaw.Iz_eff * (-2*params.xhy.yaw.lambda*ssa(x(12)-psi0));
    N_cmd = N_smc - hat_d(6);

    if mod(i,10)==0 || i<=5
        fprintf('%4d | %7.4f | %8.4f | %6.3f | %6.4f | %6.3f | %5.3f | %5.3f\n', ...
            i, e(1), Z(1,3), hat_d(1), Z(6,3), hat_d(6), N_smc, N_cmd);
    end

    % 简单推力（只维持速度）
    tau_cmd = [params.xhy.surge.m_eff/params.xhy.surge.T1; 0; 0; 0; 0; N_cmd];
    [ui, ~] = thrust_allocation_xhy(tau_cmd, struct('rho',1026,'D_prop',0.10,'KT',0.22,'n_max',2500,...
        'x_vert_f',0.344,'x_vert_r',-0.293,'x_side_f',0.424,'x_side_r',-0.376));
    x = rk4(@xhy, h, x, ui, Vc, betaVc, wc);
    x(12) = ssa(x(12));
end
fprintf('\nFinal state: u=%.3f, r=%.4f, psi=%.3f deg, x=%.1f, y=%.1f\n', ...
    x(1), x(6), rad2deg(x(12)), x(7), x(8));
