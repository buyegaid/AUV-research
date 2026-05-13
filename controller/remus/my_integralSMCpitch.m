function delta = my_integralSMCpitch(...
    theta,q,theta_d,q_d,a_d,h,params)
% 俯仰模型的ISMC控制法则
% PID and integral sliding mode controller (SMC) for heading control. The
% yaw dynamics is modelled by
%
%  theta_dot = q
%  q_dot + (1 / T_pitch) * q = (K_nomoto / T_nomoto) * delta_s
%
% where delta is the rudder angle. The input gain, K_nomoto, is found from
% the steady-state condition, r_max = K_nomoto * delta_max, while T_nomoto
% is the time constant in yaw observered during steady turning.
%
% The heading autopilot (Equation 16.479 in Fossen 2021) sliding surface
% and control law are
%
%   sigma = q-q_d + 2*lambda*ssa(theta-theta_d) + lambda^2 * integral(ssa(theta-theta_d))
%
%   delta_s = (T_nomoto * q_q_dot + q_q - K_d * sigma
%       - K_sigma*(sigma/phi_b)) / K_nomoto
%
% where lambda > 0, K_d > 0 (PID control), and K_sigma > 0 (SMC).
% The integral state psi_int is a persistent variable that should be cleared
% by adding:
%
%    clear integralSMCheading
%
% on the top in the script calling integralSMCheading.m.
%
% Inputs:
%   theta: pitch angle (rad) 俯仰角
%   q: pitch rate(rad/s) 俯仰角速度
%   theta_d: desired pitch angle (rad) 期望俯仰角
%   q_d: desired pitch rate(rad/s) 期望俯仰角速度
%   a_d: desired pitch acceleration (rad/s^2) 期望俯仰角加速度
%   K_d: PID control gain, multiplied with the sliding surface sigma PID控制增益，乘以滑动面sigma
%   K_sigma: SMC gain SMC增益
%   lambda: constant defining the dynamics on the sliding surface 滑动面的动力学常数
%   phi_b: boundary layer thickness (rad) 边界层厚度
%   K_nomoto: Nomoto gain constant (1/s) Nomoto增益常数
%   T_nomoto: Nomoto time constant (s) Nomoto时间常数
%   h: sampling time (s) 采样时间
%
% Outputs:
%    delta_s: Stern plane angle (rad) 尾舵角
%
% Author:    Thor I. Fossen
% Date:      2024-02-09
% Revisions: 2025-12-23 重写为俯仰控制

persistent theta_int;               % integral state

% 第二组参数是俯仰轴
K_d      = params.K_d;
K_sigma  = params.K_sigma;
lambda   = params.lambda;
phi_b    = params.phi_b;
K_nomoto = params.K_nomoto;
T_nomoto = params.T_nomoto;

% Initialization of desired state theta_d and integral state theta_int 初始化积分状态
if isempty(theta_int)
    theta_int = 0;
end

% PID and integral SMC (Equation 16.479 in Fossen 2021)
e_theta = ssa( theta - theta_d );  % 角度误差
e_q = q - q_d; % 角速度误差

q_r_dot = a_d - 2 * lambda * e_q - lambda^2 * e_theta;
q_r     = q_d - 2 * lambda * e_theta - lambda^2 * theta_int;
sigma   = q - q_r; % 滑模面

% 饱和
if abs(sigma / phi_b) > 1
    delta = ( T_nomoto * q_r_dot + q_r - K_d * sigma...
        - K_sigma * sign(sigma) ) / K_nomoto;
else
    delta = ( T_nomoto * q_r_dot + q_r - K_d * sigma...
        - K_sigma * (sigma / phi_b) ) / K_nomoto;
end

% Propagation of persistent integral state: psi_int[k+1]
theta_int = theta_int + h * e_theta;

end

