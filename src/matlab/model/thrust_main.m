function T = thrust_main(n, rho)
% 主推进器推力计算（双向推力系数）
% 输入:
%   n   - 转速 (RPM)，正转=正向推力，反转=反向推力
%   rho - 水密度 (kg/m^3)
% 输出:
%   T   - 推力 (N)，正=前向推力
%
% KT_fwd/KT_rev为分段PWM映射反推值（0616水池阶跃实验, 2026-06-22）
%   校准方案: [[小黄鱼推进器模型]] §3.7
%   校准依据: 0616水池阶跃 TX前进/后退, CFD阻力(300W)反推
%   per-thruster CAN → 分段PWM → RPM → KT = T/(ρD⁴(n/60)²)
%   前进(perCAN正→正向PWM→RPM正): 100% 2250RPM, 8.78N → KT_fwd=0.149
%   后退(perCAN负→反向PWM→RPM负): 80% 2150RPM, 2.90N → KT_rev=0.051
%   有效质量 m_x=101.6kg, τ_exp≈2.4s, τ_cfd(线性化)≈8.8s

D_prop = 0.08;      % 主推直径 8cm
KT_fwd = 0.1489;    % 正向推力系数 (0616 前进100% 分段PWM反推, 2026-06-22)
KT_rev = 0.0506;    % 反向推力系数 (0616 后退80% 分段PWM反推, 2026-06-22)

n_rps = n / 60;     % rpm → rps
if n >= 0
    KT = KT_fwd;
else
    KT = KT_rev;
end
T = rho * D_prop^4 * KT * abs(n_rps) * n_rps;

end
