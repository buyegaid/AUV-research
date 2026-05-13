function T = thrust_aux(n, rho)
% 辅助推进器（侧推、垂推）推力计算
% 输入:
%   n   - 转速 (RPM)
%   rho - 水密度 (kg/m^3)
% 输出:
%   T   - 推力 (N)

D_prop = 0.07;  % 辅助推直径 7cm
KT = 0.22;      % 推力系数

n_rps = n / 60;  % 转换为 rps
T = rho * D_prop^4 * KT * abs(n_rps) * n_rps;

end
