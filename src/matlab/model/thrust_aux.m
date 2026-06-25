function T = thrust_aux(n, rho)
% 辅助推进器（侧推、垂推）推力计算（双向推力系数）
% 输入:
%   n   - 转速 (RPM)，正转=正向推力，反转=反向推力
%   rho - 水密度 (kg/m^3)
% 输出:
%   T   - 推力 (N)
%
% KT_fwd/KT_rev为分段PWM映射反推值（0616水池阶跃实验, 2026-06-22）
%   校准方案: [[小黄鱼推进器模型]] §3.7
%   校准依据: 0616水池阶跃 TY右移/左移, CFD阻力(300W)反推
%   per-thruster CAN → 分段PWM → RPM → KT = T/(ρD⁴(n/60)²)
%   右移(perCAN负→反向PWM→RPM负→KT_rev): 100% 1681RPM, 7.24N → KT_rev=0.71
%   左移(perCAN正→正向PWM→RPM正→KT_fwd): 100% 1370RPM, 3.56N → KT_fwd=0.53
%   有效质量 m_y=210.6kg, τ_exp≈2.5s

D_prop = 0.06;      % 辅推直径 6cm
KT_fwd = 0.53;      % 正向推力系数 (0616 左移100% 分段PWM反推, 2026-06-22)
KT_rev = 0.71;      % 反向推力系数 (0616 右移100% 分段PWM反推, 2026-06-22)

n_rps = n / 60;     % rpm → rps
if n >= 0
    KT = KT_fwd;
else
    KT = KT_rev;
end
T = rho * D_prop^4 * KT * abs(n_rps) * n_rps;

end
