function T = thrust_aux(n, rho)
% 辅助推进器（侧推、垂推）推力计算（双向推力系数）
% 输入:
%   n   - 转速 (RPM)，正转=正向推力，反转=反向推力
%   rho - 水密度 (kg/m^3)
% 输出:
%   T   - 推力 (N)
%
% TODO: KT_fwd/KT_rev为模拟数值，需要通过水池推力试验标定实际KT曲线

D_prop = 0.06;      % 辅助推直径 6cm
KT_fwd = 0.22;      % 正向推力系数（正转，n>0）
KT_rev = 0.22;      % 反向推力系数（反转，n<0，TODO: 需试验测定）

n_rps = n / 60;     % rpm → rps
if n >= 0
    KT = KT_fwd;
else
    KT = KT_rev;
end
T = rho * D_prop^4 * KT * abs(n_rps) * n_rps;

end
