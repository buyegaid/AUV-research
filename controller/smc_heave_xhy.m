function Z_cmd = smc_heave_xhy(z, w, z_d, w_d, w_d_dot, h, params)
% SMC_HEAVE_XHY 深度SMC控制器，输出沉浮力Z (N)
%
%   动力学: m_eff_z * w_dot = Z - D_w - g_z + 扰动
%   滑模面: sigma = e_w + lambda * e_z  （二阶系统标准形式）
%   控制律: Z = m_eff_z*(w_d_dot - lambda*e_w - Kd*sigma - Ks*sat(sigma/phi_b))
%              + D_w + g_z

m_eff_z = params.m_eff_z;
d_w     = params.d_w;
g_z     = params.g_z;      % 净浮力 = B - W（正值表示正浮力，向上）
lambda  = params.lambda;
Kd      = params.Kd;
Ks      = params.Ks;
phi_b   = params.phi_b;

e_z   = z - z_d;
e_w   = w - w_d;
sigma = e_w + lambda * e_z;  % 二阶滑模面，无需积分状态

if abs(sigma/phi_b) > 1
    sat_s = sign(sigma);
else
    sat_s = sigma / phi_b;
end

% 前馈补偿阻尼和净浮力（NED坐标z向下，正浮力对应负Z力）
D_w   = d_w * w;
Z_cmd = m_eff_z*(w_d_dot - lambda*e_w - Kd*sigma - Ks*sat_s) + D_w - g_z;
end

