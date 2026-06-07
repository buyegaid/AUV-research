function hat_d = compute_current_compensation(Vc_est, beta_est, nu, psi, tau_thr, M)
% 计算海流前馈补偿力/力矩
% hat_d = M * (加速度含海流 - 加速度零海流)
% 即：估计海流对AUV动力学产生的净力效应
% 2026-06-04
if Vc_est < 1e-6
    hat_d = zeros(6, 1);
    return;
end

% 海流船体系分量
u_c = Vc_est * cos(beta_est - psi);
v_c = Vc_est * sin(beta_est - psi);
nu_c = [u_c; v_c; 0; 0; 0; 0];

% 含估计海流的动力学加速度
nu_r_est = nu - nu_c;
[tau_drag_est, ~] = xhy_drag_cfd(nu_r_est, M);
[C_est, g_est] = compute_cg_standalone(nu_r_est, psi, M);
Dnu_c = [nu(6)*v_c; -nu(6)*u_c; 0; 0; 0; 0];
nu_dot_c = Dnu_c + M \ (tau_thr + tau_drag_est - C_est*nu_r_est - g_est);

% 零海流动力学加速度
[tau_drag_0, ~] = xhy_drag_cfd(nu, M);
[C_0, g_0] = compute_cg_standalone(nu, psi, M);
nu_dot_0 = M \ (tau_thr + tau_drag_0 - C_0*nu - g_0);

% 海流净力效应
hat_d = M * (nu_dot_c - nu_dot_0);
end
