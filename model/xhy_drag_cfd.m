function [tau_drag, D] = xhy_drag_cfd(nu_r)
% XHY_DRAG_CFD 阻力计算（CFD RANS 计算, 2026-06-03 更新）
%   平动阻力: Fluent 直航/斜航 CFD, k-ω SST
%   转动阻尼: Fluent MRF Wall Motion CFD, k-ω SST
%   数据来源: Obsidian [[CFD阻尼计算结果]]
%   CFD坐标系(右上前) → 体坐标系(前右下): Z→X, X→Y, Y→Z, 绕X→q, 绕Y→r
%
u = nu_r(1); v = nu_r(2); w = nu_r(3);
p = nu_r(4); q = nu_r(5); r = nu_r(6);

% 平动阻力 (CFD RANS 计算结果, 2026-06-03)
% CFD坐标系(右上前) → 体坐标系(前右下):
%   CFD Z轴(前) → Body Surge, CFD X轴(右) → Body Sway, CFD Y轴(上) → Body Heave
% 数据来源: Obsidian [[CFD阻尼计算结果]]

% Surge: CFD Z轴 d1=0.7580, d2=3.7729, R²=0.99991
Fx = -(0.7580 * u + 3.7729 * u * abs(u));

% Sway: CFD X轴 纯二次 d2=130.7023, R²=0.99757
% 注：线性项为0是CFD拟合结果（二次+一次退化为纯二次，保证物理一致性）
Fy = -(0.0 * v + 130.7023 * v * abs(v));

% Heave: CFD Y轴 d1=1.4485, d2=45.0442, R²=0.99865
Fz = -(1.4485 * w + 45.0442 * w * abs(w));

% --- 原水池校准值（已替换为CFD结果）---
% Fx = -(1.3451 * u + 1.6172 * u * abs(u));  % 0530时域辨识
% Fy = -(0.0 * v + 87.9 * v * abs(v));        % 0530最大速度校准
% Fz = -(1.4500 * w + 67.3000 * w * abs(w));  % 0530最大速度+浮力校准
% --- 原水池校准值结束 ---

% 转动阻尼 (CFD MRF 计算结果, 2026-06-03)
% CFD坐标系(右上前) → 体坐标系(前右下):
%   CFD绕X轴 → Body q (Pitch), CFD绕Y轴 → Body r (Yaw)
%   Roll(p)为不可控自由度，阻尼系数取0
% 数据来源: Obsidian [[CFD阻尼计算结果]]

% Pitch (q): d1=0.001345, d2=0.00453, R²=0.99997
My = -(0.001345 * q + 0.00453 * q * abs(q));

% Yaw (r): d1=0.027322, d2=0.067903, R²=0.99986
Nz = -(0.027322 * r + 0.067903 * r * abs(r));

% Roll (p): 不可控自由度
K = 0;

% --- 原时间常数法（已替换为CFD MRF结果）---
% zeta4 = 0.3;
% zeta5 = 0.8;
% W = 33 * 9.81;
% r_bg_z = 0;
% r_bb_z = -0.03;
% w4 = sqrt(W * (r_bg_z - r_bb_z) / M(4,4));
% w5 = sqrt(W * (r_bg_z - r_bb_z) / M(5,5));
% T6 = 1;
% K  = -M(4,4) * 2 * zeta4 * w4 * p;
% My = -M(5,5) * 2 * zeta5 * w5 * q;
% Nz = -M(6,6) / T6 * r * 2.4;  % ×2.4: 水池校准缩放因子
% --- 原时间常数法结束 ---

tau_drag = [Fx; Fy; Fz; K; My; Nz];

D = diag([
    0.7580 + 3.7729 * abs(u);          % Surge: CFD Z轴(前)
    0.0 + 130.7023 * abs(v);           % Sway: CFD X轴(右), 纯二次
    1.4485 + 45.0442 * abs(w);         % Heave: CFD Y轴(上)
    0;                                  % Roll: 不可控
    0.001345 + 0.00453 * abs(q);       % Pitch: CFD MRF 绕X轴
    0.027322 + 0.067903 * abs(r)       % Yaw: CFD MRF 绕Y轴
    ]);
end
