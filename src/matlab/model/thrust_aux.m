function T = thrust_aux(n, rho)
% 辅助推进器（侧推、垂推）推力计算（双向推力系数）
% 输入:
%   n   - 转速 (RPM)，正转=正向推力，反转=反向推力
%   rho - 水密度 (kg/m^3)
% 输出:
%   T   - 推力 (N)
%
% KT_fwd/KT_rev为CFD阻力校准值（2026-06-03更新）
%   校准依据: 水池试验稳态 2×2500RPM→0.34m/s, CFD阻力=15.11N
%   KT = F_drag / (2 * rho * D^4 * n^2)

D_prop = 0.06;      % 辅助推直径 6cm
KT_fwd = 0.327;     % 正向推力系数（CFD阻力校准, 原0.22）
KT_rev = 0.327;     % 反向推力系数（无反向数据, 暂取与正向相同）

n_rps = n / 60;     % rpm → rps
if n >= 0
    KT = KT_fwd;
else
    KT = KT_rev;
end
T = rho * D_prop^4 * KT * abs(n_rps) * n_rps;

end
