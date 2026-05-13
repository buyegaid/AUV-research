function T = thrust_main(n, rho)
% 主推进器推力计算
% 输入:
%   n   - 转速 (RPM)
%   rho - 水密度 (kg/m^3)
% 输出:
%   T   - 推力 (N)

D_prop = 0.10;  % 主推直径 10cm
KT = 0.22;      % 推力系数

n_rps = n / 60;  % 转换为 rps
T = rho * D_prop^4 * KT * abs(n_rps) * n_rps;

end
