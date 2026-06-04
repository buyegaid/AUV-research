function T = thrust_main(n, rho)
% 主推进器推力计算（双向推力系数）
% 输入:
%   n   - 转速 (RPM)，正转=正向推力，反转=反向推力
%   rho - 水密度 (kg/m^3)
% 输出:
%   T   - 推力 (N)，正=前向推力
%
% KT_fwd/KT_rev为CFD阻力校准值（2026-06-03更新）
%   校准依据: 水池试验稳态 2500RPM→1.08m/s, CFD阻力=5.22N
%   KT = F_drag / (rho * D^4 * n^2)

D_prop = 0.1;       % 主推直径 10cm
KT_fwd = 0.0293;    % 正向推力系数（CFD阻力校准, 原0.019）
KT_rev = 0.0201;    % 反向推力系数（同比缩放, 原0.013）

n_rps = n / 60;     % rpm → rps
if n >= 0
    KT = KT_fwd;
else
    KT = KT_rev;
end
T = rho * D_prop^4 * KT * abs(n_rps) * n_rps;

end
