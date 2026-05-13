function N_cmd = smc_yaw_xhy(psi, r, psi_d, r_d, a_d, h, params)
% SMC_YAW_XHY 航向SMC控制器，输出偏航力矩N (N·m)
%
%   基于XHY动力学的偏航通道：
%     Iz_eff * r_dot = N + 扰动
%   其中 Iz_eff = Iz + Nr_dot (转动惯量 + 附加质量)
%
%   滑模面: sigma = (r - r_r)
%     r_r = r_d - 2*lambda*ssa(psi-psi_d) - lambda^2 * psi_int
%
%   控制律: N = Iz_eff*(r_r_dot - Kd*sigma - Ks*sat(sigma/phi_b))

persistent psi_int
if isempty(psi_int), psi_int = 0; end

Iz_eff  = params.Iz_eff;
lambda  = params.lambda;
Kd      = params.Kd;
Ks      = params.Ks;
phi_b   = params.phi_b;

e_psi = ssa(psi - psi_d);
e_r   = r - r_d;

r_r     = r_d - 2*lambda*e_psi - lambda^2 * psi_int;
r_r_dot = a_d - 2*lambda*e_r  - lambda^2 * e_psi;
sigma   = r - r_r;

if abs(sigma/phi_b) > 1
    sat_s = sign(sigma);
else
    sat_s = sigma / phi_b;
end

N_cmd = Iz_eff * (r_r_dot - Kd*sigma - Ks*sat_s);

psi_int = psi_int + h * e_psi;
end
