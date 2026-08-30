function [x_nom, P, aux] = eskf_current_estimator(x_nom, P, nu_meas, tau, psi, M, params, dt)
% ESKF海流估计器 — 误差状态卡尔曼滤波 (Error-State Kalman Filter)
%
% 与EKF的关键区别:
%   - 名义状态 x_nom 按确定性动力学传播（无噪声注入）
%   - 误差状态 δx 始终均值零，仅协方差 P 被维护
%   - 每次更新后 δx̂ → 注入名义状态 → δx̂ 重置为零
%   - 线性化在名义轨迹上进行（更平滑的线性化点）
%
% 状态: x = [u v w p q r cN cE]' (8状态, 速度+海流)
%
% 参考: Sola (2017), "Quaternion kinematics for the error-state Kalman filter"
%       Bar-Shalom et al. (2001), "Estimation with Applications to Tracking and Navigation"
%
% 输入/输出接口与 ekf_current_estimator.m 兼容
%
% 2026-07-09

%% ===== 维度配置 =====
n_nu = 6;
n_c = 2;
has_bias = params.use_bias;

if has_bias
    n_b = 6;
    n_x = n_nu + n_c + n_b;  % 14
else
    n_b = 0;
    n_x = n_nu + n_c;        % 8
end

%% ===== 初始化 =====
if isempty(x_nom) || length(x_nom) ~= n_x
    x_nom = zeros(n_x, 1);
    x_nom(1:n_nu) = nu_meas;
end
if isempty(P) || size(P,1) ~= n_x
    P = params.P0 * eye(n_x);
end

%% ===== Step 1: 名义状态传播 (Nominal Propagation) =====
% 确定性传播，无噪声注入（与EKF预测步的唯一区别是语义分离）

nu_hat = x_nom(1:6);
cN = x_nom(7);
cE = x_nom(8);

% 海流 NED → 船体
u_c =  cN*cos(psi) + cE*sin(psi);
v_c = -cN*sin(psi) + cE*cos(psi);
nu_c = [u_c; v_c; 0; 0; 0; 0];
nu_r = nu_hat - nu_c;

% 力偏置
if has_bias
    b_tau = x_nom(9:14);
else
    b_tau = zeros(6,1);
end

% CFD动力学预测（名义轨迹）
[tau_drag, ~] = xhy_drag_cfd(nu_r);
[C_nu, g_nu] = compute_coriolis_gravity_eskf(nu_r, psi, M);
r = nu_hat(6);
Dnu_c = [r*v_c; -r*u_c; 0; 0; 0; 0];
nu_dot_nom = Dnu_c + M \ (tau + tau_drag - C_nu*nu_r - g_nu + b_tau);

% 名义状态 Euler 积分
x_pred = x_nom;
x_pred(1:6) = nu_hat + nu_dot_nom * dt;

% 海流 GM 传播（确定性部分）
alpha_c = exp(-dt / params.tau_c);
x_pred(7) = alpha_c * cN + (1-alpha_c) * params.c_mean(1);
x_pred(8) = alpha_c * cE + (1-alpha_c) * params.c_mean(2);

% 力偏置慢变
if has_bias
    alpha_b = exp(-dt / params.tau_b);
    x_pred(9:14) = alpha_b * b_tau;
end

%% ===== Step 2: 误差状态预测 (Error-State Prediction) =====
% δx̂_{k|k-1} = F · 0 = 0  (误差状态均值恒为零)
% P_{k|k-1} = F · P · Fᵀ + Q

% 数值 Jacobian（与EKF相同，但在名义状态点线性化）
F = eye(n_x);
delta_x = params.jac_pert;

for j = 1:n_x
    xp = x_nom;
    xp(j) = xp(j) + delta_x;

    nup = xp(1:6); cNp = xp(7); cEp = xp(8);
    u_cp =  cNp*cos(psi) + cEp*sin(psi);
    v_cp = -cNp*sin(psi) + cEp*cos(psi);
    nu_cp = [u_cp; v_cp; 0; 0; 0; 0];
    nu_rp = nup - nu_cp;

    [tau_drag_p, ~] = xhy_drag_cfd(nu_rp);
    [Cp, gp] = compute_coriolis_gravity_eskf(nu_rp, psi, M);
    Dnu_c_p = [nup(6)*v_cp; -nup(6)*u_cp; 0; 0; 0; 0];

    if has_bias
        bp = xp(9:14);
    else
        bp = zeros(6,1);
    end

    nu_dot_p = Dnu_c_p + M \ (tau + tau_drag_p - Cp*nu_rp - gp + bp);

    xp_pred = xp;
    xp_pred(1:6) = nup + nu_dot_p * dt;
    xp_pred(7) = alpha_c*cNp + (1-alpha_c)*params.c_mean(1);
    xp_pred(8) = alpha_c*cEp + (1-alpha_c)*params.c_mean(2);
    if has_bias
        xp_pred(9:14) = alpha_b * xp(9:14);
    end

    F(:, j) = (xp_pred - x_pred) / delta_x;
end

% 误差状态协方差预测
Q = params.Q0 * eye(n_x);
Q(1:6, 1:6) = params.Q_nu * eye(6) * dt;
Q(7:8, 7:8) = params.Q_c * eye(2) * dt;
if has_bias
    Q(9:14, 9:14) = params.Q_b * eye(6) * dt;
end
P_pred = F * P * F' + Q;

%% ===== Step 3: Kalman 增益 =====
H = [eye(6), zeros(6, n_x-6)];  % DVL 直接速度测量
R = params.R0 * eye(6);

% 新息: 使用名义状态计算（ESKF关键差异）
y_innov = nu_meas - x_pred(1:6);
S = H * P_pred * H' + R;
K = P_pred * H' / S;

%% ===== Step 4: 误差状态更新 =====
% δx̂_k = K · y  (误差状态的校正量)
dx_hat = K * y_innov;

% 误差协方差更新
P = (eye(n_x) - K * H) * P_pred;
% 对称化（数值稳定性）
P = (P + P') / 2;

%% ===== Step 5: 名义状态注入 + 误差状态重置 =====
% 将估计的误差注入名义状态
x_nom = x_pred + dx_hat;

% 海流约束
x_nom(7) = max(-params.c_max, min(params.c_max, x_nom(7)));
x_nom(8) = max(-params.c_max, min(params.c_max, x_nom(8)));

% 误差状态重置为零（ESKF 标志性步骤）
% dx_hat 已在注入后被消耗，下次迭代从零开始

%% ===== 诊断输出 =====
aux.c_hat = x_nom(7:8);
aux.P_c = P(7:8, 7:8);
aux.innovation = y_innov(1:2);
aux.nu_hat = x_nom(1:6);
aux.dx_hat = dx_hat;  % 记录误差状态校正量（ESKF特有）
aux.nominal_pred = x_pred;  % 记录名义预测（ESKF特有）
if has_bias
    aux.b_hat = x_nom(9:14);
end
end

%% ===== 辅助函数 =====
function [C, g] = compute_coriolis_gravity_eskf(nu_r, psi, M)
% XHY Coriolis+重力计算 (与 ekf_current_estimator.m 完全一致)
m = 85.832;
Ix = 1.419139; Iy = 7.173529; Iz = 6.390605;
Ig = diag([Ix Iy Iz]);
nu2 = nu_r(4:6);
O3 = zeros(3,3);
CRB = [m*Smtrx(nu2), O3; O3, -Smtrx(Ig*nu2)];

MA = diag([15.81, 124.73, 42.87, 0.014, 0.041, 0.123]);
CA = m2c(MA, nu_r);
CA(5,3)=0; CA(3,5)=0; CA(5,1)=0; CA(1,5)=0; CA(6,1)=0; CA(1,6)=0;

C = CRB + CA;

mu = 63.446827;
g_mu = gravity(mu);
W = m * g_mu;
B = W;
[~, R] = eulerang(0, 0, psi);
r_bG = [0;0;0]; r_bB = [0;0;-0.03];
g = gRvect(W, B, R, r_bG, r_bB);
end
