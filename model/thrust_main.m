function T = thrust_main(n, rho)
% 主推进器推力计算（双向推力系数）
% 输入:
%   n   - 转速 (RPM)，正转=正向推力，反转=反向推力
%   rho - 水密度 (kg/m^3)
% 输出:
%   T   - 推力 (N)，正=前向推力
%
% TODO: KT_fwd/KT_rev为模拟数值，需要通过水池推力试验标定实际KT曲线

D_prop = 0.1;       % 主推直径 10cm
KT_fwd = 0.019;     % 有效正向推力系数（时域辨识校准, 原0.33）
KT_rev = 0.013;     % 有效反向推力系数（同比缩放, 原0.22）

n_rps = n / 60;     % rpm → rps
if n >= 0
    KT = KT_fwd;
else
    KT = KT_rev;
end
T = rho * D_prop^4 * KT * abs(n_rps) * n_rps;

end
