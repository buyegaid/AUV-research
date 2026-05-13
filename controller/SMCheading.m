function delta = my_smcheading(...
    psi, r, psi_d, r_d, a_d, hat_dr, h, params)
% 航向模型的SMC控制法则（去掉积分项）
% Sliding Mode Controller (SMC) for heading control. The yaw dynamics is modelled by
%
%  psi_dot = r
%  r_dot + (1 / T_yaw) * r = (K_nomoto / T_nomoto) * delta
%
% where delta is the rudder angle. The input gain, K_nomoto, is found from
% the steady-state condition, r_max = K_nomoto * delta_max, while T_nomoto
% is the time constant in yaw observed during steady turning.
%
% The heading autopilot sliding surface and control law are:
%
%   sigma = e_r + lambda * e_psi
%   delta = (T_nomoto * r_r_dot + r_r - K_d * sigma
%       - K_sigma * sat(sigma/phi_b)) / K_nomoto
%
% where lambda > 0, K_d > 0 (PID control), and K_sigma > 0 (SMC).
%
% Inputs:
%   psi: yaw angle (rad) 偏航角
%   r: yaw rate (rad/s) 偏航角速度
%   psi_d: desired yaw angle (rad) 期望偏航角
%   r_d: desired yaw rate (rad/s) 期望偏航角速度
%   a_d: desired yaw acceleration (rad/s^2) 期望偏航角加速度
%   hat_dr: estimated disturbance (rad/s^2) 估计干扰
%   h: sampling time (s) 采样时间
%   params: structure containing controller parameters
%       K_d: PID control gain, multiplied with the sliding surface sigma
%       K_sigma: SMC gain
%       lambda: constant defining the dynamics on the sliding surface
%       phi_b: boundary layer thickness (rad)
%       K_nomoto: Nomoto gain constant (1/s)
%       T_nomoto: Nomoto time constant (s)
%       (optional) rho_eso: ESO gain
%       (optional) delta_eso_max: maximum ESO rudder compensation (rad)
%
% Outputs:
%    delta: rudder angle (rad) 舵角
%
% Author: Modified from Thor I. Fossen (2024-02-09)
% Date:   2025-04-01

% 获取参数
K_d = params.K_d;
K_sigma = params.K_sigma;
lambda = params.lambda;
phi_b = params.phi_b;
K_nomoto = params.K_nomoto;
T_nomoto = params.T_nomoto;
rho_eso = params.rho_eso;
delta_eso_max = params.delta_eso_max;

% 角度误差（考虑角度周期性）
e_psi = ssa(psi - psi_d);
e_r = r - r_d;

% 标准滑模面定义（无积分项）
r_r     = r_d - lambda * e_psi;           % 参考角速度
r_r_dot = a_d - lambda * e_r;             % 参考角加速度
sigma   = r - r_r;                        % 滑模面

% 饱和函数处理
if abs(sigma / phi_b) > 1
    delta = (T_nomoto * r_r_dot + r_r - K_d * sigma ...
        - K_sigma * sign(sigma)) / K_nomoto;
else
    delta = (T_nomoto * r_r_dot + r_r - K_d * sigma ...
        - K_sigma * (sigma / phi_b)) / K_nomoto;
end

% ESO补偿项
delta_eso = -(T_nomoto / K_nomoto) * rho_eso * hat_dr;
delta_eso = max(-delta_eso_max, min(delta_eso_max, delta_eso));

% 总舵角命令
delta = delta + delta_eso;

end

