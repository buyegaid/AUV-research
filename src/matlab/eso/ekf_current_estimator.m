function [x_hat, P, aux] = ekf_current_estimator(x_hat, P, nu_meas, tau, psi, M, params, dt)
% CFD增广EKF海流估计器（强基线）
%
% 两个模式:
%   mode=1: x = [u v w p q r cN cE]' (8状态, 仅增广海流)
%   mode=2: x = [u v w p q r cN cE b1 b2 b3 b4 b5 b6]' (14状态, 增广海流+力偏置)
%
% 过程模型: CFD 6-DOF动力学 + GM海流模型
% 测量模型: ν(1:6) 直接测量 (DVL + IMU)
%
% 输入:
%   x_hat:   状态估计向量
%   P:       协方差矩阵
%   nu_meas: 6×1 DVL对地速度测量
%   tau:     6×1 控制力/力矩
%   psi:     航向角 (rad)
%   M:       6×6 惯性矩阵
%   params:  params.* 参数
%   dt:      时间步长
%
% 输出:
%   x_hat:   更新后的状态
%   P:       更新后的协方差
%   aux:     诊断信息 (含海流估计 [cN;cE])

n_nu = 6;
n_c = 2;
has_bias = params.use_bias;

if has_bias
    n_b = 6;
    n_x = n_nu + n_c + n_b;  % 14
else
    n_b = 0;
    n_x = n_nu + n_c;  % 8
end

% 确保初始化
if isempty(x_hat) || length(x_hat) ~= n_x
    x_hat = zeros(n_x, 1);
    x_hat(1:n_nu) = nu_meas;
end
if isempty(P) || size(P,1) ~= n_x
    P = params.P0 * eye(n_x);
end

%% ==== 预测步 ====

% 提取状态
nu_hat = x_hat(1:6);
cN = x_hat(7);
cE = x_hat(8);

% 海流在船体系分量
u_c =  cN*cos(psi) + cE*sin(psi);
v_c = -cN*sin(psi) + cE*cos(psi);
nu_c = [u_c; v_c; 0; 0; 0; 0];
nu_r = nu_hat - nu_c;

% 力偏置
if has_bias
    b_tau = x_hat(9:14);
else
    b_tau = zeros(6,1);
end

% CFD动力学预测
[tau_drag, ~] = xhy_drag_cfd(nu_r);
[C_nu, g_nu] = compute_coriolis_gravity_ekf(nu_r, psi, M);
r = nu_hat(6);
Dnu_c = [r*v_c; -r*u_c; 0; 0; 0; 0];

nu_dot_pred = Dnu_c + M \ (tau + tau_drag - C_nu*nu_r - g_nu + b_tau);

% 状态预测（Euler积分）
x_pred = x_hat;
x_pred(1:6) = nu_hat + nu_dot_pred * dt;

% 海流GM传播
alpha_c = exp(-dt / params.tau_c);
x_pred(7) = alpha_c * cN + (1-alpha_c) * params.c_mean(1);
x_pred(8) = alpha_c * cE + (1-alpha_c) * params.c_mean(2);

% 力偏置慢变
if has_bias
    alpha_b = exp(-dt / params.tau_b);
    x_pred(9:14) = alpha_b * b_tau;
end

%% ==== Jacobian计算（数值）====
F = eye(n_x);
delta_x = params.jac_pert;

for j = 1:n_x
    xp = x_hat;
    xp(j) = xp(j) + delta_x;

    nup = xp(1:6); cNp = xp(7); cEp = xp(8);
    u_cp =  cNp*cos(psi) + cEp*sin(psi);
    v_cp = -cNp*sin(psi) + cEp*cos(psi);
    nu_cp = [u_cp; v_cp; 0; 0; 0; 0];
    nu_rp = nup - nu_cp;

    [tau_drag_p, ~] = xhy_drag_cfd(nu_rp);
    [Cp, gp] = compute_coriolis_gravity_ekf(nu_rp, psi, M);
    Dnu_c_p = [nup(6)*v_cp; -nup(6)*u_cp; 0; 0; 0; 0];

    if has_bias
        bp = xp(9:14);
    else
        bp = zeros(6,1);
    end

    nu_dot_p = Dnu_c_p + M \ (tau + tau_drag_p - Cp*nu_rp - gp + bp);

    % 前6个状态
    xp_pred = xp;
    xp_pred(1:6) = nup + nu_dot_p * dt;

    % 海流
    xp_pred(7) = alpha_c*cNp + (1-alpha_c)*params.c_mean(1);
    xp_pred(8) = alpha_c*cEp + (1-alpha_c)*params.c_mean(2);

    if has_bias
        xp_pred(9:14) = alpha_b * xp(9:14);
    end

    F(:, j) = (xp_pred - x_pred) / delta_x;
end

% 协方差预测
Q = params.Q0 * eye(n_x);
Q(1:6, 1:6) = params.Q_nu * eye(6) * dt;
Q(7:8, 7:8) = params.Q_c * eye(2) * dt;
if has_bias
    Q(9:14, 9:14) = params.Q_b * eye(6) * dt;
end
P_pred = F * P * F' + Q;

%% ==== 更新步 ====
H = [eye(6), zeros(6, n_x-6)];  % 测量 ν(1:6)
R = params.R0 * eye(6);

y = nu_meas - x_pred(1:6);  % 新息
S = H * P_pred * H' + R;
K = P_pred * H' / S;

x_hat = x_pred + K * y;
P = (eye(n_x) - K * H) * P_pred;

% 海流约束
x_hat(7) = max(-params.c_max, min(params.c_max, x_hat(7)));
x_hat(8) = max(-params.c_max, min(params.c_max, x_hat(8)));

%% ==== 诊断 ====
aux.c_hat = x_hat(7:8);
aux.P_c = P(7:8, 7:8);
aux.innovation = y(1:2);
aux.nu_hat = x_hat(1:6);
if has_bias
    aux.b_hat = x_hat(9:14);
end
end

function [C, g] = compute_coriolis_gravity_ekf(nu_r, psi, M)
% XHY Coriolis+重力计算 (与xhy.m一致, 2026-06-23修正)
% 之前使用REMUS 100参数(m=33)为错误值，现已修正为XHY参数
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
W = m * g_mu;  % 重力
B = W;         % 中性浮力 (与xhy.m一致)
[~, R] = eulerang(0, 0, psi);
r_bG = [0;0;0]; r_bB = [0;0;-0.03];
g = gRvect(W, B, R, r_bG, r_bB);
end
