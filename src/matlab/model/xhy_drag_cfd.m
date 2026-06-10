function [tau_drag, D] = xhy_drag_cfd(nu_r, mismatch_pct, mismatch_seed)
% XHY_DRAG_CFD 阻力计算（CFD RANS 计算, 2026-06-03 更新）
%   平动阻力: Fluent 直航/斜航 CFD, k-ω SST
%   转动阻尼: Fluent MRF Wall Motion CFD, k-ω SST
%   数据来源: Obsidian [[CFD阻尼计算结果]]
%   CFD坐标系(右上前) → 体坐标系(前右下): Z→X, X→Y, Y→Z, 绕X→q, 绕Y→r
%
%   可选参数:
%     mismatch_pct:  0-100, 阻力系数扰动百分比 (默认0=名义模型)
%     mismatch_seed: 随机种子 (默认1), 确保可复现
%
%   Plant调用: xhy_drag_cfd(nu_r, mismatch_pct, seed)  → 扰动后阻力
%   Observer调用: xhy_drag_cfd(nu_r)  → 名义阻力

if nargin < 2, mismatch_pct = 0; end
if nargin < 3, mismatch_seed = 1; end

u = nu_r(1); v = nu_r(2); w = nu_r(3);
p = nu_r(4); q = nu_r(5); r = nu_r(6);

% 名义阻力系数
d_X1 = 0.7580;    d_X2 = 3.7729;
d_Y1 = 0.0;       d_Y2 = 130.7023;
d_Z1 = 1.4485;    d_Z2 = 45.0442;
d_M1 = 0.001345;  d_M2 = 0.00453;
d_N1 = 0.027322;  d_N2 = 0.067903;

% 模型失配: 每个系数独立随机扰动
if mismatch_pct > 0
    rng_state = rng;
    rng(mismatch_seed);
    scale = @() 1 + mismatch_pct/100 * (2*rand - 1);
    d_X1 = d_X1 * scale(); d_X2 = d_X2 * scale();
    d_Y2 = d_Y2 * scale();
    d_Z1 = d_Z1 * scale(); d_Z2 = d_Z2 * scale();
    d_M1 = d_M1 * scale(); d_M2 = d_M2 * scale();
    d_N1 = d_N1 * scale(); d_N2 = d_N2 * scale();
    rng(rng_state);
end

% 平动阻力
Fx = -(d_X1 * u + d_X2 * u * abs(u));
Fy = -(d_Y1 * v + d_Y2 * v * abs(v));
Fz = -(d_Z1 * w + d_Z2 * w * abs(w));

% 转动阻尼
K  = 0;
My = -(d_M1 * q + d_M2 * q * abs(q));
Nz = -(d_N1 * r + d_N2 * r * abs(r));

tau_drag = [Fx; Fy; Fz; K; My; Nz];

D = diag([
    d_X1 + d_X2 * abs(u);
    d_Y1 + d_Y2 * abs(v);
    d_Z1 + d_Z2 * abs(w);
    0;
    d_M1 + d_M2 * abs(q);
    d_N1 + d_N2 * abs(r)
    ]);
end
