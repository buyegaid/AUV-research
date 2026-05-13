function delta = my_integralSMCheading(...
    psi,r,psi_d,r_d,a_d,h,params)
% 航向模型的ISMC控制法则
% PID and integral sliding mode controller (SMC) for heading control. The
% yaw dynamics is modelled by
%
%  psi_dot = r
%  r_dot + (1 / T_yaw) * r = (K_nomoto / T_nomoto) * delta
%
% where delta is the rudder angle. The input gain, K_nomoto, is found from
% the steady-state condition, r_max = K_nomoto * delta_max, while T_nomoto
% is the time constant in yaw observered during steady turning.
%
% The heading autopilot (Equation 16.479 in Fossen 2021) sliding surface
% and control law are
%
%   sigma = r-r_d + 2*lambda*ssa(psi-psi_d) + lambda^2 * integral(ssa(psi-psi_d))
%
%   delta = (T_nomoto * r_r_dot + r_r - K_d * sigma
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
%   psi: yaw angle (rad) 偏航角
%   r: yaw rate(rad/s) 偏航角速度
%   psi_d: desired yaw angle (rad) 期望偏航角
%   r_d: desired yaw rate(rad/s) 期望偏航角速度
%   a_d: desired yaw acceleration (rad/s^2) 期望偏航角加速度
%   K_d: PID control gain, multiplied with the sliding surface sigma PID控制增益，乘以滑动面sigma
%   K_sigma: SMC gain SMC增益
%   lambda: constant defining the dynamics on the sliding surface 滑动面的动力学常数
%   phi_b: boundary layer thickness (rad) 边界层厚度
%   K_nomoto: Nomoto gain constant (1/s) Nomoto增益常数
%   T_nomoto: Nomoto time constant (s) Nomoto时间常数
%   h: sampling time (s) 采样时间
%
% Outputs:
%    delta: rudder angle (rad) 舵角
%
% Author:    Thor I. Fossen
% Date:      2024-02-09

% 第一组参数是航向轴
K_d=params.K_d;
K_sigma=params.K_sigma;
lambda=params.lambda;
phi_b=params.phi_b;
K_nomoto=params.K_nomoto;
T_nomoto=params.T_nomoto;

persistent psi_int;               % integral state
% Initialization of desired state psi_d and integral state z_int 初始化积分状态
if isempty(psi_int)
    psi_int = 0;
end

% PID and integral SMC (Equation 16.479 in Fossen 2021)
e_psi = ssa( psi - psi_d );  % 角度误差
e_r = r - r_d; % 角速度误差

r_r_dot = a_d - 2 * lambda * e_r - lambda^2 * e_psi;
r_r     = r_d - 2 * lambda * e_psi - lambda^2 * psi_int;
sigma   = r - r_r; % 滑模面

% 饱和
if abs(sigma / phi_b) > 1
    delta = ( T_nomoto * r_r_dot + r_r - K_d * sigma...
        - K_sigma * sign(sigma) ) / K_nomoto;
else
    delta = ( T_nomoto * r_r_dot + r_r - K_d * sigma...
        - K_sigma * (sigma / phi_b) ) / K_nomoto;
end

% Propagation of persistent integral state: psi_int[k+1]
psi_int = psi_int + h * e_psi;

end

