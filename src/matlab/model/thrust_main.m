function T = thrust_main(n, rho)
% 主推进器推力计算（双向推力系数）
% 输入:
%   n   - 转速 (RPM)，正转=正向推力，反转=反向推力
%   rho - 水密度 (kg/m^3)
% 输出:
%   T   - 推力 (N)，正=前向推力
%
% KT_fwd/KT_rev为非饱和段映射校准值（2026-06-21更新）
%   校准方案: [[推进器推力校准方案_20260621]] §3.3, 0616水池阶跃实验
%   校准方法: 20-80%非饱和阶跃段 → CFD阻力(300W)反推k → 80%参考RPM转KT
%   有效质量 m_x=101.6kg (刚性85.83+附加15.81)
%   时间常数 τ_exp≈2.4s, τ_cfd(线性化,0.6m/s)≈8.8s (exp/CFD≈0.27)
%   最大转速 2500 RPM, KT非固定值(随RPM变化), 当前取80%工作点(2105RPM)
%   参考: k_X_fwd=0.000888 N/CAN-g, k_X_rev=0.000361 N/CAN-g

D_prop = 0.1;       % 主推直径 10cm
KT_fwd = 0.0567;    % 正向推力系数 (0616非饱和校准, 80%RPM_ref=2105)
KT_rev = 0.0235;    % 反向推力系数 (0616非饱和校准, 80%RPM_ref=2105)

n_rps = n / 60;     % rpm → rps
if n >= 0
    KT = KT_fwd;
else
    KT = KT_rev;
end
T = rho * D_prop^4 * KT * abs(n_rps) * n_rps;

end
