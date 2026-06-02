function X_cmd = smc_surge_xhy(u, u_d, u_d_dot, h, params)
% SMC_SURGE_XHY 纵荡SMC控制器，输出纵向推力X (N)
%
%   基于XHY动力学的纵荡通道：
%     m_eff * u_dot = X - D_linear*u + 扰动
%   其中 m_eff = m + Xu_dot
%   D_linear 由时间常数T1计算: D_linear = m_eff / T1
%
%   滑模面: sigma = e_u + lambda * u_int
%   控制律: X = m_eff*(u_d_dot - lambda*e_u - Kd*sigma - Ks*sat(sigma/phi_b)) + D_linear*u

persistent u_int
if isempty(u_int), u_int = 0; end

m_eff  = params.m_eff;   % m + Xu_dot
T1     = params.T1;      % 纵荡时间常数
lambda = params.lambda;
Kd     = params.Kd;
Ks     = params.Ks;
phi_b  = params.phi_b;

e_u   = u - u_d;
sigma = e_u + lambda * u_int;

if abs(sigma/phi_b) > 1
    sat_s = sign(sigma);
else
    sat_s = sigma / phi_b;
end

D_linear = m_eff / T1;
X_cmd = m_eff*(u_d_dot - lambda*e_u - Kd*sigma - Ks*sat_s) + D_linear*u;

u_int = u_int + h * e_u;
end
