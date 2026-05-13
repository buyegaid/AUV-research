function M_cmd = smc_pitch_xhy(theta, q, theta_d, q_d, a_d, h, params)
% SMC_PITCH_XHY 俯仰SMC控制器，输出俯仰力矩M (N·m)
%
%   基于XHY动力学的俯仰通道：
%     Iy_eff * q_dot = M + 扰动
%   其中 Iy_eff = Iy + Mq_dot
%
%   滑模面: sigma = q - q_r
%     q_r = q_d - 2*lambda*ssa(theta-theta_d) - lambda^2 * theta_int
%
%   控制律: M = Iy_eff*(q_r_dot - Kd*sigma - Ks*sat(sigma/phi_b))

persistent theta_int
if isempty(theta_int), theta_int = 0; end

Iy_eff  = params.Iy_eff;
lambda  = params.lambda;
Kd      = params.Kd;
Ks      = params.Ks;
phi_b   = params.phi_b;

e_theta = ssa(theta - theta_d);
e_q     = q - q_d;

q_r     = q_d - 2*lambda*e_theta - lambda^2 * theta_int;
q_r_dot = a_d - 2*lambda*e_q    - lambda^2 * e_theta;
sigma   = q - q_r;

if abs(sigma/phi_b) > 1
    sat_s = sign(sigma);
else
    sat_s = sigma / phi_b;
end

M_cmd = Iy_eff * (q_r_dot - Kd*sigma - Ks*sat_s);

theta_int = theta_int + h * e_theta;
end
