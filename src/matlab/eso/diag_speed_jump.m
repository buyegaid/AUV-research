%% 速度异常诊断脚本
clear; close all; clc;
clear my_ALOS3D LOSobserver smc_surge_xhy smc_yaw_xhy smc_heave_xhy eg_ucco_simple

project_root = setup_paths();
dt = 0.01; T_end = 600; t_vec = 0:dt:T_end; N = length(t_vec);
Vc = 0.5; betaVc = deg2rad(45); wc = 0;
c_true_N = Vc * cos(betaVc); c_true_E = Vc * sin(betaVc);

pts = traj(50, 50);
params = get_params();
params.alos.K_f = 0.5; params.alos.R_switch = 20;

thr_params = struct('rho',1026,'D_prop_main',0.10,'D_prop_aux',0.06,...
    'KT_main_fwd',0.0310,'KT_main_rev',0.0213,...
    'KT_aux_fwd',0.444,'KT_aux_rev',0.166,'n_max',2500,...
    'x_vert_f',0.344,'x_vert_r',-0.293,'x_side_f',0.424,'x_side_r',-0.376);

x_state = [1.0;0;0;0;0;0;300;0;10;0;0;pi/2];
ui = zeros(5,1); c_hat = [0;0];
psi_d = pi/2; r_d = 0; u_d = 1.0; u_d_dot = 0; z_d = 10;

% 惯性矩阵
m = 85.832; Ix = 1.419; Iy = 7.174; Iz = 6.391;
MRB = diag([m,m,m,Ix,Iy,Iz]);
MA = diag([15.81,124.73,42.87,0.014,0.041,0.123]);
M_const = MRB + MA;

fprintf('===== 诊断运行 (t=0-600s) =====\n');
timebar(1, N, '诊断运行');

for i = 1:N
    nu = x_state(1:6); xn = x_state(7); yn = x_state(8); zn = x_state(9); psi = x_state(12);

    [~,~,M,C,D,g_vec,tau_thr] = xhy(x_state, ui, Vc, betaVc, wc);
    [psi_ref,~,y_e] = my_ALOS3D(xn, yn, zn, dt, pts, params.alos);
    [psi_d, r_d] = LOSobserver(psi_d, r_d, psi_ref, dt, params.alos.K_f);
    r_d = sat(r_d, 0.5);

    X_cmd = smc_surge_xhy(nu(1), u_d, u_d_dot, dt, params.xhy.surge);
    N_cmd = smc_yaw_xhy(psi, nu(6), psi_d, r_d, 0, dt, params.xhy.yaw);
    Z_cmd = smc_heave_xhy(zn, nu(3), z_d, 0, 0, dt, params.xhy.heave);
    tau_cmd = [X_cmd; 0; Z_cmd; 0; 0; N_cmd];

    [ui, info] = thrust_allocation_xhy(tau_cmd, thr_params);

    % 检测速度穿越0附近
    if i > 10 && abs(nu(1)) < 0.5
        u_c_x = Vc * cos(betaVc - psi);
        u_c_y = Vc * sin(betaVc - psi);
        fprintf('\nt=%.1f u=%.3f Xc=%.0f Nc=%.1f psi=%.0f° psid=%.0f° psiref=%.0f° y_e=%.1f\n',...
            t_vec(i), nu(1), X_cmd, N_cmd, rad2deg(psi), rad2deg(psi_d), rad2deg(psi_ref), y_e);
        fprintf('  RPM=[%.0f %.0f %.0f %.0f %.0f] tau_thr=[%.1f %.1f %.1f %.1f %.1f %.2f]\n',...
            ui(1),ui(2),ui(3),ui(4),ui(5),tau_thr(1),tau_thr(2),tau_thr(3),tau_thr(4),tau_thr(5),tau_thr(6));
        fprintf('  nu=[%.2f %.2f %.2f %.4f %.4f %.4f] uc=[%.3f %.3f]\n',...
            nu(1),nu(2),nu(3),nu(4),nu(5),nu(6),u_c_x,u_c_y);
    end

    x_state = rk4(@xhy, dt, x_state, ui, Vc, betaVc, wc);
    x_state(12) = ssa(x_state(12));
    timebar;
end
fprintf('\n完成\n');
