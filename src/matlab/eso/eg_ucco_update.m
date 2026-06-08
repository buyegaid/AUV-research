function [c_hat, P_c, nu_obs, aux] = eg_ucco_update(c_hat, P_c, nu_obs, nu_meas, tau, psi, M, params, dt)
% EG-UCCO: 激励门控不确定性标定海流观测器
% Excitation-Gated Uncertainty-Calibrated Current Observer
%
% 核心创新:
%   1. 灵敏度Gramian门控 — 仅在Fisher信息充足时更新
%   2. Set-membership更新 — 利用CFD模型误差已知界进行鲁棒估计
%   3. 不确定性分量化 — 分量化的模型误差界
%   4. GM时间传播 — 无激励时通过Gauss-Markov模型维持估计
%
% 输入:
%   c_hat:   2×1 惯性系水平海流估计 [cN; cE] (m/s)
%   P_c:     2×2 估计协方差矩阵
%   nu_obs:  6×1 观测器状态（对水速度估计）
%   nu_meas: 6×1 测量速度 (DVL, 对地)
%   tau:     6×1 控制力/力矩
%   psi:     航向角 (rad)
%   M:       6×6 惯性矩阵
%   params:  参数结构体 (params.ucco.*)
%   dt:      时间步长 (s)
%
% 输出:
%   c_hat:   更新后的海流估计
%   P_c:     更新后的协方差
%   nu_obs:  更新后的观测器状态
%   aux:     诊断信息结构体
%
% 参考:
%   Børhaug et al. (2007) — 动力学海流观测器
%   Kim et al. (2018) — 模型基高增益观测器
%   Codex GPT-5.5 审阅建议 (2026-06-04)

persistent nu_prev t_prev Wc_buf Phi_buf r_buf buf_idx buf_N is_init

% 初始化persistent状态
if isempty(is_init)
    nu_prev = nu_meas;
    t_prev = 0;
    buf_N = params.window_size;  % 滑动窗口大小
    Wc_buf = zeros(2, 2, buf_N);
    Phi_buf = zeros(6, 2, buf_N);
    r_buf = zeros(6, buf_N);
    buf_idx = 0;
    is_init = true;
end

%% 1. 海流在船体系中的分量
u_c =  c_hat(1)*cos(psi) + c_hat(2)*sin(psi);
v_c = -c_hat(1)*sin(psi) + c_hat(2)*cos(psi);
nu_c = [u_c; v_c; 0; 0; 0; 0];

%% 2. 观测器模型前向积分（含测量注入）
% nu_r_obs = nu_obs - nu_c
% 使用简化动力学: nu_dot ≈ M\(tau - D(nu_r)*nu_r - g) + K*(nu_meas - nu_obs)
[tau_drag_obs, D_obs] = xhy_drag_cfd(nu_obs - nu_c);
[C_obs, g_obs] = compute_coriolis_gravity(nu_obs - nu_c, psi, M);

nu_dot_obs = M \ (tau + tau_drag_obs - C_obs*(nu_obs - nu_c) - g_obs) ...
           + params.K_obs * (nu_meas - nu_obs);

nu_obs = nu_obs + nu_dot_obs * dt;

%% 3. 计算灵敏度矩阵 Φc = ∂(ν̇)/∂c（数值扰动）
delta_c = params.sens_pert;  % 扰动步长 (m/s)
Phi_c = zeros(6, 2);

for j = 1:2
    c_pert = c_hat;
    c_pert(j) = c_pert(j) + delta_c;

    % 扰动的船体系海流
    u_c_p =  c_pert(1)*cos(psi) + c_pert(2)*sin(psi);
    v_c_p = -c_pert(1)*sin(psi) + c_pert(2)*cos(psi);
    nu_c_p = [u_c_p; v_c_p; 0; 0; 0; 0];
    nu_r_p = nu_obs - nu_c_p;

    % 扰动动力学加速度
    [tau_drag_p, ~] = xhy_drag_cfd(nu_r_p);
    [C_p, g_p] = compute_coriolis_gravity(nu_r_p, psi, M);

    % Dνc项对海流的导数: Dνc = [r*v_c; -r*u_c; 0; 0; 0; 0]
    r = nu_meas(6);
    Dnu_c_p = [r*v_c_p; -r*u_c_p; 0; 0; 0; 0];

    nu_dot_p = Dnu_c_p + M \ (tau + tau_drag_p - C_p*nu_r_p - g_p);

    % 基准（当前估计的加速度）
    [tau_drag_0, ~] = xhy_drag_cfd(nu_obs - nu_c);
    Dnu_c_0 = [r*v_c; -r*u_c; 0; 0; 0; 0];
    nu_dot_0 = Dnu_c_0 + M \ (tau + tau_drag_0 - C_obs*(nu_obs - nu_c) - g_obs);

    Phi_c(:, j) = (nu_dot_p - nu_dot_0) / delta_c;
end

%% 4. 计算模型残差
% 用滤波后的加速度（ν差分+低通）vs 模型预测加速度
a_meas_filt = lowpass_diff(nu_meas, nu_prev, dt, params.accel_lpf_alpha);
a_model = nu_dot_obs;
r_accel = a_meas_filt - a_model;

%% 5. 更新滑动窗口
buf_idx = mod(buf_idx, buf_N) + 1;
Phi_buf(:, :, buf_idx) = Phi_c;
r_buf(:, buf_idx) = r_accel;

% 累积Gramian: Wc = Σ Φ_c' R^(-1) Φ_c
R_inv = diag(params.R_inv_diag);  % 测量噪声逆
Wc_buf(:, :, buf_idx) = Phi_c' * R_inv * Phi_c;
Wc = sum(Wc_buf, 3);

%% 6. 激励门控 — 基于灵敏度Gramian
lambda_min = min(eig(Wc));
excited = (lambda_min > params.gate_mu);

%% 7. 海流更新
Delta_model = compute_uncertainty_bound(nu_meas, nu_c, params);  % 模型不确定性界

if excited
    % 7a. Set-membership更新: 在可信窗口内求解最小二乘
    % min_δc ||r_buf - Φ_buf*δc||  subject to |δc| bounded
    active_N = min(buf_idx, buf_N);
    Phi_stack = reshape(Phi_buf(:, :, 1:active_N), 6*active_N, 2);
    r_stack = reshape(r_buf(:, 1:active_N), 6*active_N, 1);
    R_stack_inv = kron(eye(active_N), R_inv);

    % 加权最小二乘更新
    H = Phi_stack' * R_stack_inv * Phi_stack;
    g_vec = Phi_stack' * R_stack_inv * r_stack;

    % 正则化（处理病态）
    H_reg = H + params.reg_lambda * eye(2);
    delta_c = H_reg \ g_vec;

    % 步长限制（防止单步过大更新）
    delta_c = max(-params.max_dc, min(params.max_dc, delta_c));

    % 更新估计
    c_hat_new = c_hat + delta_c;

    % 检查残差是否在模型误差界内
    r_new = r_accel - Phi_c * delta_c;
    r_norm = norm(r_new(1:2));  % 主要关注surge/sway

    if r_norm < Delta_model
        % 残差在模型不确定性界内 → 归因于模型误差，不放行更新
        % （这是死区机制：residual < model_error_bound → no update）
        aux.update_blocked = true;
        aux.deadzone_active = true;
    else
        % 残差超出模型界 → 归因于海流，接受更新
        c_hat = c_hat_new;

        % 更新协方差（Kalman-like）
        S = H_reg + inv(P_c + params.Q_c * eye(2) * dt);
        K_c = (P_c + params.Q_c * eye(2) * dt) / S;
        P_c = (eye(2) - K_c) * (P_c + params.Q_c * eye(2) * dt);

        aux.update_blocked = false;
        aux.deadzone_active = false;
    end
    aux.updated = true;
else
    % 7b. 无激励 → GM时间传播
    alpha = exp(-dt / params.tau_c);
    c_hat = alpha * c_hat + (1 - alpha) * params.c_mean;

    % 协方差增长（不确定性随时间增大）
    P_c = P_c + params.Q_c * eye(2) * dt;

    aux.updated = false;
    aux.deadzone_active = false;
    aux.update_blocked = false;
end

%% 8. 海流约束（避免不合理的估计值）
c_hat = max(-params.c_max, min(params.c_max, c_hat));

%% 9. 置信度计算
aux.confidence = 1 / (1 + trace(P_c) / params.conf_scale);

%% 10. 诊断信息
aux.c_hat = c_hat;
aux.P_c = P_c;
aux.lambda_min = lambda_min;
aux.excited = excited;
aux.r_norm = norm(r_accel(1:2));
aux.Delta_model = Delta_model;
aux.nu_obs = nu_obs;
aux.Phi_c = Phi_c;
aux.Wc = Wc;
aux.gate_mu = params.gate_mu;

% 更新persistent
nu_prev = nu_meas;
end

%% ========== 辅助函数 ==========

function [C, g] = compute_coriolis_gravity(nu_r, psi, M)
% 计算 Coriolis矩阵和重力/浮力向量（简化版，与xhy.m一致）
% nu_r: 6×1 相对速度

% 刚体 Coriolis
m = 33;
Ix = 0.540804; Iy = 2.107488; Iz = 1.849137;
Ig = diag([Ix Iy Iz]);
nu2 = nu_r(4:6);
O3 = zeros(3,3);
CRB = [m*Smtrx(nu2), O3; O3, -Smtrx(Ig*nu2)];

% 附加质量 Coriolis
MA = diag([11.2773, 132.1086, 47.104, 0.006, 0.043, 0.138]);
CA = m2c(MA, nu_r);
CA(5,3)=0; CA(3,5)=0; CA(5,1)=0; CA(1,5)=0; CA(6,1)=0; CA(1,6)=0;

C = CRB + CA;

% 重力/浮力
mu = 63.446827;
g_mu = gravity(mu);
W = m * g_mu;
B = W * 1.01;
[~, R] = eulerang(0, 0, psi);  % 简化：仅用航向角
r_bG = [0;0;0]; r_bB = [0;0;-0.03];
g = gRvect(W, B, R, r_bG, r_bB);
end

function a_filt = lowpass_diff(nu_curr, nu_prev, dt, alpha)
% 低通滤波 + 差分求加速度
a_raw = (nu_curr - nu_prev) / dt;
persistent a_filt_mem
if isempty(a_filt_mem)
    a_filt_mem = zeros(6,1);
end
a_filt = alpha * a_raw + (1-alpha) * a_filt_mem;
a_filt_mem = a_filt;
end

function Delta = compute_uncertainty_bound(nu, nu_c, params)
% 计算状态相关的分量式模型不确定性界
% Δ_i = Δ_single-DOF,i + Δ_coupling,i + Δ_thruster,i + Δ_sensor,i
u_r = nu(1) - nu_c(1);
v_r = nu(2) - nu_c(2);
w_r = nu(3) - nu_c(3);
r = nu(6);
q = nu(5);

% 单自由度阻尼不确定性（基于CFD R²残差 × 保守系数）
Delta_single = params.delta_single .* [abs(u_r); abs(v_r); abs(w_r); 1; abs(q); abs(r)];

% 耦合项不确定性（以速度和角速度的乘积估计）
Delta_couple = params.delta_coupling .* ...
    [abs(v_r*r) + abs(w_r*q);   % Surge: sway-yaw + heave-pitch耦合
     abs(u_r*r);                 % Sway: surge-yaw耦合
     abs(u_r*q);                 % Heave: surge-pitch耦合
     0;                          % Roll: 不可控
     abs(u_r*w_r);               % Pitch: surge-heave耦合
     abs(u_r*v_r)];              % Yaw: surge-sway耦合

% 推进器不确定性
Delta_thrust = params.delta_thruster * ones(6,1);

% 传感器不确定性（DVL噪声 + INS偏置）
Delta_sensor = params.delta_sensor * ones(6,1);

Delta = Delta_single + Delta_couple + Delta_thrust + Delta_sensor;
Delta = norm(Delta(1:2));  % 重点关注水平面
end
