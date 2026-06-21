function [tau_drag, D] = xhy_drag_cfd(nu_r, mismatch_pct, mismatch_seed)
% XHY_DRAG_CFD 阻力计算（CFD RANS 计算, 2026-06-21 更新）
%   平动阻力: Fluent 直航 CFD, k-ω SST, 300W网格收敛结果
%   转动阻尼: Fluent MRF + 滑移网格 CFD, k-ω SST, 中网格（2026-06-18）
%   平动数据来源: Obsidian [[260617 小黄鱼 CFD 直航阻尼 新]]
%   转动数据来源: Obsidian [[260618 小黄鱼 CFD 旋转阻尼 新]]
%   新CFD坐标系(前右下) → 体坐标系(前右下): 直接对应, 无需转换
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

% 名义阻力系数（CFD RANS 计算结果, 2026-06-17/18, 300W/中网格）
% 新CFD坐标系(前右下)与体坐标系一致, 直接对应
% 平动数据来源: Obsidian [[260617 小黄鱼 CFD 直航阻尼 新]]
% 转动数据来源: Obsidian [[260618 小黄鱼 CFD 旋转阻尼 新]]
d_X1 = 0.65;       d_X2 = 10.85;    % Surge: CFD X轴(前), 300W网格
d_Y1 = 1.67;       d_Y2 = 178.59;   % Sway: CFD Y轴(右), 300W网格
d_Z1 = 2.95;       d_Z2 = 40.83;    % Heave: CFD Z轴(下), 300W网格
d_M1 = 0.5941;     d_M2 = 4.088;    % Pitch: CFD MRF 中网格 (210W)
d_N1 = 0.2046;     d_N2 = 18.200;   % Yaw: CFD MRF 中网格 (276W)

% 模型失配: 每个系数独立随机扰动（Monte Carlo鲁棒性测试用）
if mismatch_pct > 0
    rng_state = rng;
    rng(mismatch_seed);
    scale = @() 1 + mismatch_pct/100 * (2*rand - 1);
    d_X1 = d_X1 * scale(); d_X2 = d_X2 * scale();
    d_Y1 = d_Y1 * scale(); d_Y2 = d_Y2 * scale();
    d_Z1 = d_Z1 * scale(); d_Z2 = d_Z2 * scale();
    d_M1 = d_M1 * scale(); d_M2 = d_M2 * scale();
    d_N1 = d_N1 * scale(); d_N2 = d_N2 * scale();
    rng(rng_state);
end

% 平动阻力
Fx = -(d_X1 * u + d_X2 * u * abs(u));
Fy = -(d_Y1 * v + d_Y2 * v * abs(v));
Fz = -(d_Z1 * w + d_Z2 * w * abs(w));

% 转动阻尼（Roll为不可控自由度, 阻尼系数取0）
K  = 0;
My = -(d_M1 * q + d_M2 * q * abs(q));
Nz = -(d_N1 * r + d_N2 * r * abs(r));

tau_drag = [Fx; Fy; Fz; K; My; Nz];

D = diag([
    d_X1 + d_X2 * abs(u);   % Surge
    d_Y1 + d_Y2 * abs(v);   % Sway
    d_Z1 + d_Z2 * abs(w);   % Heave
    0;                       % Roll: 不可控
    d_M1 + d_M2 * abs(q);   % Pitch
    d_N1 + d_N2 * abs(r)    % Yaw
    ]);
end
