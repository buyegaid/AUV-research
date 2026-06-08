function [c_hat, aux] = eg_ucco_simple(c_hat, nu, tau, psi, M, params, dt)
% EG-UCCO 简化版: 速度层面自适应海流观测器
% 使用速度预测误差驱动海流估计更新（避免加速度噪声放大问题）
%
% 核心:
%   1. 预测速度: nu_pred = nu_prev + dt*a_model(c_hat)
%   2. 速度新息: e = nu_meas - nu_pred
%   3. 灵敏度: Phi = ∂(nu_pred)/∂c (数值)
%   4. 梯度更新: c_hat += gamma * Phi' * R_inv * e
%
% 2026-06-04

persistent nu_prev
if isempty(nu_prev), nu_prev = nu; end

%% 1. 用当前海流估计预测速度（模型一步前向）
u_c =  c_hat(1)*cos(psi) + c_hat(2)*sin(psi);
v_c = -c_hat(1)*sin(psi) + c_hat(2)*cos(psi);
nu_c = [u_c; v_c; 0; 0; 0; 0];
nu_r = nu_prev - nu_c;

[tau_drag, ~] = xhy_drag_cfd(nu_r);
[C_nu, g_nu] = compute_cg_u(nu_r, psi, M);
Dnu_c = [nu_prev(6)*v_c; -nu_prev(6)*u_c; 0; 0; 0; 0];

a_model = Dnu_c + M \ (tau + tau_drag - C_nu*nu_r - g_nu);
nu_pred = nu_prev + a_model * dt;

%% 2. 速度新息（仅surge/sway）
e_vel = nu(1:2) - nu_pred(1:2);

%% 3. 数值灵敏度 ∂(nu_pred)/∂c
delta = params.sens_pert;
Phi = zeros(2, 2);
for j = 1:2
    cp = c_hat; cp(j) = cp(j) + delta;
    u_c_p =  cp(1)*cos(psi) + cp(2)*sin(psi);
    v_c_p = -cp(1)*sin(psi) + cp(2)*cos(psi);
    nu_c_p = [u_c_p; v_c_p; 0; 0; 0; 0];
    nu_r_p = nu_prev - nu_c_p;
    [td_p, ~] = xhy_drag_cfd(nu_r_p);
    [Cp, gp] = compute_cg_u(nu_r_p, psi, M);
    Dnc_p = [nu_prev(6)*v_c_p; -nu_prev(6)*u_c_p; 0; 0; 0; 0];
    a_p = Dnc_p + M \ (tau + td_p - Cp*nu_r_p - gp);
    nu_p = nu_prev + a_p * dt;
    Phi(:, j) = (nu_p(1:2) - nu_pred(1:2)) / delta;
end

%% 4. 激励门控 — 灵敏度Gramian
Wc = Phi' * Phi;  % 简化Gramian（不加权）
lambda_min = min(eig(Wc));

%% 5. 自适应更新
gamma_eff = params.K_obs * 100;  % 速度层面有效增益

if lambda_min > params.gate_mu
    % 有激励: 梯度更新
    gain = gamma_eff / (1 + lambda_min * 1e6);
    dc = gain * Phi' * e_vel;
    dc = max(-params.max_dc, min(params.max_dc, dc));
    c_hat = c_hat + dc;
    aux.updated = true;
else
    % 无激励: GM时间传播
    alpha = exp(-dt / params.tau_c);
    c_hat = alpha * c_hat + (1-alpha) * params.c_mean;
    aux.updated = false;
end

%% 6. 约束
c_hat = max(-params.c_max, min(params.c_max, c_hat));

%% 7. 更新persistent
nu_prev = nu;

%% 8. 诊断
aux.c_hat = c_hat;
aux.e_vel = e_vel;
aux.lambda_min = lambda_min;
aux.excited = (lambda_min > params.gate_mu);
aux.Phi = Phi;
end

function [C, g] = compute_cg_u(nu_r, psi, M)
m = 33; Ix = 0.540804; Iy = 2.107488; Iz = 1.849137;
Ig = diag([Ix Iy Iz]);
nu2 = nu_r(4:6); O3 = zeros(3,3);
CRB = [m*Smtrx(nu2), O3; O3, -Smtrx(Ig*nu2)];
MA = diag([11.2773, 132.1086, 47.104, 0.006, 0.043, 0.138]);
CA = m2c(MA, nu_r);
CA(5,3)=0; CA(3,5)=0; CA(5,1)=0; CA(1,5)=0; CA(6,1)=0; CA(1,6)=0;
C = CRB + CA;
mu = 63.446827; g_mu = gravity(mu);
W = m * g_mu; B = W * 1.01;
[~, R] = eulerang(0, 0, psi);
g = gRvect(W, B, R, [0;0;0], [0;0;-0.03]);
end
