function [c_hat, aux] = borhaug_current_observer(c_hat, nu_meas, tau, psi, M, params, dt)
% Børhaug-型模型基海流观测器（简化速度预测型）
%
% 原理（Børhaug et al. 2007 模型基观测器的简化实现）：
%   利用已知动力学模型进行一步速度预测，预测误差归因于海流。
%   ν̂_pred = ν_meas + dt·a_model(ν_meas, τ, ĉ)    ← 从当前测量预测下一步
%   c_hat += K_c · (ν_meas_next - ν̂_pred)(1:2)      ← 预测误差积分
%
% 与UCCO的核心区别：
%   - 无灵敏度Gramian分析（使用固定增益K_c）
%   - 无激励门控（持续更新）
%   - 假设完美模型（Børhaug原文的核心假设）
%
% 参考: Børhaug et al. (2007), IFAC CAMS
% 2026-06-10

persistent nu_prev is_init
if isempty(is_init), is_init = true; end
if isempty(nu_prev), nu_prev = nu_meas; end
if isempty(c_hat), c_hat = [0; 0]; end

%% 1. 海流船体系分量
u_c =  c_hat(1)*cos(psi) + c_hat(2)*sin(psi);
v_c = -c_hat(1)*sin(psi) + c_hat(2)*cos(psi);
nu_c = [u_c; v_c; 0; 0; 0; 0];

%% 2. 模型预测（使用上一时刻测量速度 + 当前海流估计）
nu_r = nu_prev - nu_c;
[tau_drag, ~] = xhy_drag_cfd(nu_r);
[C_nu, g_nu] = compute_coriolis_gravity_borhaug(nu_r, psi, M);
r = nu_prev(6);
Dnu_c = [r*v_c; -r*u_c; 0; 0; 0; 0];
a_model = Dnu_c + M \ (tau + tau_drag - C_nu*nu_r - g_nu);
nu_pred = nu_prev + a_model * dt;

%% 3. 一步预测误差（仅水平面）
e_vel = nu_meas(1:2) - nu_pred(1:2);

%% 4. 海流自适应更新（带限幅的积分）
dc = params.K_c .* e_vel * dt;
dc = max(-params.max_dc*dt, min(params.max_dc*dt, dc));
c_hat = c_hat + dc;

%% 5. GM衰减
alpha = exp(-dt / params.tau_c);
c_hat = alpha * c_hat + (1-alpha) * params.c_mean;

%% 6. 约束
c_hat = max(-params.c_max, min(params.c_max, c_hat));

%% 7. 更新persistent
nu_prev = nu_meas;

%% 8. 诊断
aux.c_hat = c_hat;
aux.e_vel = e_vel;
aux.nu_pred = nu_pred;
end

function [C, g] = compute_coriolis_gravity_borhaug(nu_r, psi, M)
m = 85.832;
Ix = 0.553864787 + 0.865274;
Iy = 2.162341935 + 5.011187;
Iz = 1.849137 + 4.541468;
Ig = diag([Ix Iy Iz]);
nu2 = nu_r(4:6); O3 = zeros(3,3);
CRB = [m*Smtrx(nu2), O3; O3, -Smtrx(Ig*nu2)];
MA = diag([15.81, 124.73, 42.87, 0.014, 0.041, 0.123]);
CA = m2c(MA, nu_r);
CA(5,3)=0; CA(3,5)=0; CA(5,1)=0; CA(1,5)=0; CA(6,1)=0; CA(1,6)=0;
C = CRB + CA;
mu_val = 63.446827;
g_mu = gravity(mu_val);
W = m * g_mu; B = W;
[~, R] = eulerang(0, 0, psi);
g = gRvect(W, B, R, [0;0;0], [0;0;-0.03]);
end
