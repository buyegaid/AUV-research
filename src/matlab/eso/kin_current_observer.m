function [c_hat, aux] = kin_current_observer(c_hat, nu_meas, psi, u_cmd, params, dt)
% 运动学海流观测器（速度残差型, 2026-06-10重写）
%
% 核心原理:
%   DVL测量 ν_meas = ν_water + ν_c (对地 = 对水 + 海流)
%   命令速度 u_cmd 近似对水速度（稳态时推力=阻力）
%   海流 ≈ 测量速度 - 命令速度
%
%   ν_I = R(ψ)·ν_meas  → 惯性系对地速度
%   ν̂_I = [u_cmd·cos(ψ); u_cmd·sin(ψ)] → 惯性系命令速度（无侧滑假设）
%   海流残差 = ν_I(1:2) - ν̂_I
%   ĉ̇ = K·(残差) − GM衰减
%
% 输入:
%   c_hat:   2×1 惯性系海流估计 [cN; cE]
%   nu_meas: 6×1 DVL对地速度
%   psi:     航向角
%   u_cmd:   纵荡命令速度 (m/s), 稳态时≈对水速度
%   params:  params.kin.*
%   dt:      时间步长
%
% 2026-06-10

persistent is_init
if isempty(is_init), is_init = true; end
if isempty(c_hat), c_hat = [0; 0]; end

%% 1. DVL对地速度 → 惯性系
% 惯性系速度 = R_nb(ψ) * 体坐标系速度
% ν_I = [cos(ψ)*u - sin(ψ)*v; sin(ψ)*u + cos(ψ)*v; w]
u = nu_meas(1); v = nu_meas(2);
nu_I_N =  cos(psi)*u - sin(psi)*v;
nu_I_E =  sin(psi)*u + cos(psi)*v;

%% 2. 命令速度 → 惯性系（假设无侧滑，对水速度≈命令速度）
nu_cmd_N = u_cmd * cos(psi);
nu_cmd_E = u_cmd * sin(psi);

%% 3. 速度残差 = 对地速度 − 命令对水速度 ≈ 海流
vel_res = [nu_I_N - nu_cmd_N; nu_I_E - nu_cmd_E];

%% 4. 海流积分更新
c_dot = params.K3 .* vel_res;

% GM衰减
alpha = exp(-dt / params.tau_c);
c_dot = c_dot - (1-alpha)/dt * (c_hat - params.c_mean);

%% 5. 欧拉积分
c_hat = c_hat + c_dot * dt;

%% 6. 约束
c_hat = max(-params.c_max, min(params.c_max, c_hat));

%% 7. 诊断
aux.c_hat = c_hat;
aux.vel_res = vel_res;
end
