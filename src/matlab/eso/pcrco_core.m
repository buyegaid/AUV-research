function core = pcrco_core(c_hat, nu_meas, nu_prev, tau, psi, M, params, dt)
% PCRCO_CORE  PC-RCO 共享计算核心（纯函数，无 persistent 状态）
%   提取 eg_ucco_simple 中速度预测 + 灵敏度计算部分，
%   供 Full/NoGM/NoClamp/NoPredCorr 四个变体复用。
%
%   输入:
%     c_hat:   2×1 NED海流估计 [cN; cE] (m/s)
%     nu_meas: 6×1 当前测量速度 (DVL对地)
%     nu_prev: 6×1 上一时刻测量速度
%     tau:     6×1 控制力/力矩
%     psi:     航向角 (rad)
%     M:       6×6 惯性矩阵
%     params:  参数结构体 (params.ucco.*)
%     dt:      时间步长 (s)
%
%   输出结构体 core:
%     .nu_pred    — 6×1 预测速度
%     .e_vel      — 2×1 速度新息 (surge+sway)
%     .Phi        — 2×2 灵敏度矩阵 ∂(nu_pred_12)/∂c
%     .Wc         — 2×2 灵敏度 Gramian
%     .lambda_min — Gramian 最小特征值
%     .a_model    — 6×1 模型加速度（用于 NoPredCorr 对比）
%     .nu_r       — 6×1 相对速度（诊断用）
%     .tau_drag   — 6×1 CFD阻力（诊断用）
%
%   2026-06-21

%% 1. 海流船体系分量
u_c =  c_hat(1)*cos(psi) + c_hat(2)*sin(psi);
v_c = -c_hat(1)*sin(psi) + c_hat(2)*cos(psi);
nu_c = [u_c; v_c; 0; 0; 0; 0];

%% 2. 相对速度
nu_r = nu_prev - nu_c;

%% 3. CFD阻力
[tau_drag, ~] = xhy_drag_cfd(nu_r);

%% 4. Coriolis + 重力
[C_nu, g_nu] = compute_cg_standalone(nu_r, psi, M);

%% 5. Dnu_c 项（海流旋转耦合）
r_nu = nu_prev(6);
Dnu_c = [r_nu*v_c; -r_nu*u_c; 0; 0; 0; 0];

%% 6. 模型加速度
a_model = Dnu_c + M \ (tau + tau_drag - C_nu*nu_r - g_nu);

%% 7. 一步速度预测
nu_pred = nu_prev + a_model * dt;

%% 8. 速度新息（仅水平面）
e_vel = nu_meas(1:2) - nu_pred(1:2);

%% 9. 数值灵敏度 ∂(nu_pred_12)/∂c
delta_c = params.sens_pert;  % 0.01 m/s
Phi = zeros(2, 2);

for j = 1:2
    cp = c_hat;
    cp(j) = cp(j) + delta_c;

    % 扰动后的船体系海流
    u_c_p =  cp(1)*cos(psi) + cp(2)*sin(psi);
    v_c_p = -cp(1)*sin(psi) + cp(2)*cos(psi);
    nu_c_p = [u_c_p; v_c_p; 0; 0; 0; 0];
    nu_r_p = nu_prev - nu_c_p;

    % 扰动后的阻力
    [tau_drag_p, ~] = xhy_drag_cfd(nu_r_p);
    [C_p, g_p] = compute_cg_standalone(nu_r_p, psi, M);

    % 扰动后的 Dnu_c
    Dnu_c_p = [nu_prev(6)*v_c_p; -nu_prev(6)*u_c_p; 0; 0; 0; 0];

    % 扰动后的加速度和速度预测
    a_model_p = Dnu_c_p + M \ (tau + tau_drag_p - C_p*nu_r_p - g_p);
    nu_pred_p = nu_prev + a_model_p * dt;

    % 灵敏度列
    Phi(:, j) = (nu_pred_p(1:2) - nu_pred(1:2)) / delta_c;
end

%% 10. 灵敏度 Gramian
Wc = Phi' * Phi;
lambda_min = min(eig(Wc));

%% 输出
core.nu_pred    = nu_pred;
core.e_vel      = e_vel;
core.Phi        = Phi;
core.Wc         = Wc;
core.lambda_min = lambda_min;
core.a_model    = a_model;
core.nu_r       = nu_r;
core.tau_drag   = tau_drag;
core.nu_c       = nu_c;
end
