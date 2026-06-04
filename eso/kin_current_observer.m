function [Vc_hat, beta_hat, x_hat, aux] = kin_current_observer(Vc_hat, beta_hat, x_hat, nu_meas, eta_meas, psi, theta, params, dt)
% 运动学海流观测器（Liang et al. 2018 方法）
%
% 基于AUV运动学方程: η̇ = J(η)·ν + νc_I
% 其中 νc_I = [Vc·cos(βc), Vc·sin(βc), 0]' 是惯性系海流速度
%
% 观测器结构:
%   η̂̇ = J(η)·ν_meas + V̂c_I + K4·(η - η̂)
%   V̂̇c_I = K3·(η - η̂)
%
% 输入:
%   Vc_hat:    海流速度估计 (m/s)
%   beta_hat:  海流方向估计 (rad, 惯性系)
%   x_hat:     3×1 位置估计 [xn; yn; zn] (m, NED)
%   nu_meas:   6×1 测量速度 (对地, 体坐标系)
%   eta_meas:  3×1 测量位置 [xn; yn; zn]
%   psi:       航向角
%   theta:     俯仰角
%   params:    params.kin.* 参数
%   dt:        时间步长
%
% 输出:
%   Vc_hat, beta_hat: 更新后的海流估计
%   x_hat:  更新后的位置估计
%   aux:    诊断信息
%
% 参考:
%   Liang et al. (2018), "Three-dimensional trajectory tracking control
%   of an underactuated AUV based on ocean current observer"
%   Int. J. Advanced Robotic Systems, 15(5)

persistent is_init
if isempty(is_init)
    is_init = true;
end

%% 1. 旋转矩阵（体坐标系 → NED 惯性系）
[~, R_nb] = eulerang(theta, 0, psi);  % R_nb: 3×3 体坐标系→NED旋转矩阵

% 体坐标系速度转换到惯性系
nu_I = R_nb * nu_meas(1:3);

%% 2. 观测器动力学
% 位置误差
eta_err = eta_meas - x_hat;

% 海流观测器更新（惯性系）
Vc_I_dot = params.K3 * eta_err;

% 位置观测器
x_hat_dot = nu_I + [Vc_hat*cos(beta_hat); Vc_hat*sin(beta_hat); 0] ...
          + params.K4 * eta_err;

%% 3. 数值积分
Vc_I_est = [Vc_hat*cos(beta_hat); Vc_hat*sin(beta_hat); 0] + Vc_I_dot * dt;
x_hat = x_hat + x_hat_dot * dt;

%% 4. 提取 Vc 和 βc（从惯性系估计）
Vc_hat_new = norm(Vc_I_est(1:2));
beta_hat_new = atan2(Vc_I_est(2), Vc_I_est(1));

% 低通滤波（减少观测器噪声）
alpha = params.lpf_alpha;
Vc_hat = alpha * Vc_hat_new + (1-alpha) * Vc_hat;
beta_hat = alpha * beta_hat_new + (1-alpha) * beta_hat;

% 海流约束
Vc_hat = max(0, min(params.Vc_max, Vc_hat));

%% 5. 诊断
aux.Vc_I_est = Vc_I_est;
aux.eta_err = eta_err;
aux.x_hat = x_hat;
end
