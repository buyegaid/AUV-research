function [c_hat, nu_hat, aux] = borhaug_current_observer(c_hat, nu_hat, nu_meas, tau, psi, M, params, dt)
% Børhaug 2007 非线性Luenberger型海流观测器
%
% 增广状态 x̂ = [ν̂; ĉ] 的非线性观测器:
%   ν̂̇ = M⁻¹(τ + D(ν̂r)·ν̂r + C(ν̂r)·ν̂r + g) + Kν·(ν_meas - ν̂)
%   ċ̇ = Kc·(ν_meas - ν̂)
%
% 其中 ν̂r = ν̂ - ν̂c, ν̂c = [u_c; v_c; 0; 0; 0; 0] 为船体系海流分量
%
% 输入:
%   c_hat:   2×1 惯性系海流估计 [cN; cE] (m/s)
%   nu_hat:  6×1 速度估计 (m/s, rad/s)
%   nu_meas: 6×1 DVL测量速度 (对地)
%   tau:     6×1 控制力/力矩
%   psi:     航向角 (rad)
%   M:       6×6 惯性矩阵
%   params:  params.borhaug.* 参数
%   dt:      时间步长 (s)
%
% 输出:
%   c_hat:   更新后的海流估计
%   nu_hat:  更新后的速度估计
%   aux:     诊断信息
%
% 参考:
%   Børhaug, Pivano, Pettersen, Johansen (2007)
%   "A Model-Based Ocean Current Observer for 6DOF Underwater Vehicles"
%   IFAC CAMS, DOI: 10.3182/20070919-3-HR-3904.00031
%
% 2026-06-10

persistent is_init
if isempty(is_init)
    is_init = true;
end

% 默认初始化
if isempty(nu_hat)
    nu_hat = nu_meas;
end
if isempty(c_hat)
    c_hat = [0; 0];
end

%% 1. 海流船体系分量
u_c =  c_hat(1)*cos(psi) + c_hat(2)*sin(psi);
v_c = -c_hat(1)*sin(psi) + c_hat(2)*cos(psi);
nu_c = [u_c; v_c; 0; 0; 0; 0];

%% 2. 相对速度
nu_r = nu_hat - nu_c;

%% 3. 动力学计算（使用名义CFD模型）
[tau_drag, ~] = xhy_drag_cfd(nu_r);
[C_nu, g_nu] = compute_coriolis_gravity_borhaug(nu_r, psi, M);

% Coriolis类耦合项 Dνc = [r·v_c; -r·u_c; 0; 0; 0; 0]
r = nu_hat(6);
Dnu_c = [r*v_c; -r*u_c; 0; 0; 0; 0];

%% 4. 速度预测（名义动力学加速度）
nu_dot_nom = Dnu_c + M \ (tau + tau_drag - C_nu*nu_r - g_nu);

%% 5. 速度观测器更新（Luenberger注入）
% K_nu: 6×6 对角速度误差注入增益
e_nu = nu_meas - nu_hat;
nu_dot = nu_dot_nom + params.K_nu * e_nu;

%% 6. 海流观测器更新
% K_c: 2×6 海流误差注入增益（仅用surge/sway通道）
% 原文中 Kc 的设计使得海流估计比速度估计收敛更慢
c_dot = params.K_c * e_nu(1:2);  % 仅用水平面速度误差

%% 7. GM时间传播（衰减项，使海流估计不会漂移）
alpha = exp(-dt / params.tau_c);
c_gm_dot = -(1 - alpha) / dt * (c_hat - params.c_mean);

% 合并: 测量注入 + GM衰减
c_dot = c_dot + c_gm_dot;

%% 8. 欧拉积分
nu_hat = nu_hat + nu_dot * dt;
c_hat = c_hat + c_dot * dt;

%% 9. 海流约束
c_hat = max(-params.c_max, min(params.c_max, c_hat));

%% 10. 速度约束（防止发散）
nu_hat(1) = max(0, nu_hat(1));  % surge ≥ 0

%% 11. 诊断
aux.c_hat = c_hat;
aux.nu_hat = nu_hat;
aux.e_nu = e_nu;
aux.nu_dot_nom = nu_dot_nom;
end

function [C, g] = compute_coriolis_gravity_borhaug(nu_r, psi, M)
% 计算 Coriolis矩阵和重力/浮力向量（与xhy.m一致）
% 使用XHY实际质量参数
m = 85.832;
Ix = 0.553864787 + 0.865274;     % Ix + Kp_dot
Iy = 2.162341935 + 5.011187;     % Iy + Mq_dot
Iz = 1.849137 + 4.541468;        % Iz + Nr_dot
Ig = diag([Ix Iy Iz]);

nu2 = nu_r(4:6);
O3 = zeros(3,3);
CRB = [m*Smtrx(nu2), O3; O3, -Smtrx(Ig*nu2)];

% 附加质量 Coriolis（与xhy.m一致）
MA = diag([15.81, 124.73, 42.87, 0.014, 0.041, 0.123]);
CA = m2c(MA, nu_r);
CA(5,3)=0; CA(3,5)=0; CA(5,1)=0; CA(1,5)=0; CA(6,1)=0; CA(1,6)=0;

C = CRB + CA;

% 重力/浮力（中性浮力 B=W）
mu = 63.446827;
g_mu = gravity(mu);
W = m * g_mu;
B = W;  % 中性浮力
[~, R] = eulerang(0, 0, psi);
r_bG = [0;0;0]; r_bB = [0;0;-0.03];
g = gRvect(W, B, R, r_bG, r_bB);
end
