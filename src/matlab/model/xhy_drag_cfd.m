function [tau_drag, D] = xhy_drag_cfd(nu_r)
% XHY_DRAG_CFD 阻力计算（CFD RANS 计算, 2026-06-21 更新）
%   平动阻力: Fluent 直航 CFD, k-ω SST, 300W网格收敛结果
%   转动阻尼: Fluent MRF + 滑移网格 CFD, k-ω SST, 中网格（2026-06-18）
%   平动数据来源: Obsidian [[260617 小黄鱼 CFD 直航阻尼 新]]
%   转动数据来源: Obsidian [[260618 小黄鱼 CFD 旋转阻尼 新]]
%   新CFD坐标系(前右下) → 体坐标系(前右下): 直接对应, 无需转换
%
u = nu_r(1); v = nu_r(2); w = nu_r(3);
p = nu_r(4); q = nu_r(5); r = nu_r(6);

% 平动阻力 (CFD RANS 计算结果, 2026-06-17, 300W网格)
% 新CFD坐标系(前右下)与体坐标系一致, 直接对应:
%   CFD X轴(前) → Body Surge, CFD Y轴(右) → Body Sway, CFD Z轴(下) → Body Heave
% 网格独立性验证: 100W/200W/300W三套网格, 300W收敛
% 数据来源: Obsidian [[260617 小黄鱼 CFD 直航阻尼 新]]

% Surge: CFD X轴(前) d1=0.65, d2=10.85, 1m/s阻力11.5N
Fx = -(0.65 * u + 10.85 * u * abs(u));

% Sway: CFD Y轴(右) d1=1.67, d2=178.59, 1m/s阻力180.2N
Fy = -(1.67 * v + 178.59 * v * abs(v));

% Heave: CFD Z轴(下) d1=2.95, d2=40.83, 1m/s阻力43.7N
Fz = -(2.95 * w + 40.83 * w * abs(w));

% --- 旧CFD值（2026-06-03, 不同坐标系, 已替换）---
% 旧CFD坐标系(右上前) → 体坐标系(前右下):
% Fx = -(0.7580 * u + 3.7729 * u * abs(u));  % Surge: 旧CFD Z轴
% Fy = -(0.0 * v + 130.7023 * v * abs(v));   % Sway: 旧CFD X轴, 纯二次
% Fz = -(1.4485 * w + 45.0442 * w * abs(w)); % Heave: 旧CFD Y轴
% --- 旧CFD值结束 ---

% --- 原水池校准值（已替换为CFD结果）---
% Fx = -(1.3451 * u + 1.6172 * u * abs(u));  % 0530时域辨识
% Fy = -(0.0 * v + 87.9 * v * abs(v));        % 0530最大速度校准
% Fz = -(1.4500 * w + 67.3000 * w * abs(w));  % 0530最大速度+浮力校准
% --- 原水池校准值结束 ---

% 转动阻尼 (CFD MRF 计算结果, 2026-06-18, 中网格)
% 新CFD坐标系(前右下)与体坐标系一致, 直接对应:
%   绕CFD X轴 = Body Roll(p), 绕CFD Y轴 = Body Pitch(q), 绕CFD Z轴 = Body Yaw(r)
%   Roll(p)为不可控自由度，阻尼系数取0
% 方法: MRF（多参考系法）+ sliding mesh（滑移网格）
% 数据来源: Obsidian [[260618 小黄鱼 CFD 旋转阻尼 新]]

% Pitch (q): C1=0.5941, C2=4.088, R²≈1.000（中网格 210W）
My = -(0.5941 * q + 4.088 * q * abs(q));

% Yaw (r): C1=0.2046, C2=18.200, R²=0.99999（中网格 276W）
Nz = -(0.2046 * r + 18.200 * r * abs(r));

% Roll (p): 不可控自由度
K = 0;

% --- 旧CFD MRF值（2026-06-03, 不同坐标系, 已替换）---
% My = -(0.001345 * q + 0.00453 * q * abs(q));  % Pitch 旧值
% Nz = -(0.027322 * r + 0.067903 * r * abs(r)); % Yaw 旧值
% --- 旧CFD MRF值结束 ---

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
    0.65 + 10.85 * abs(u);             % Surge: CFD X轴(前), 300W网格
    1.67 + 178.59 * abs(v);            % Sway: CFD Y轴(右), 300W网格
    2.95 + 40.83 * abs(w);             % Heave: CFD Z轴(下), 300W网格
    0;                                  % Roll: 不可控
    0.5941 + 4.088 * abs(q);           % Pitch: CFD MRF 中网格 (2026-06-18)
    0.2046 + 18.200 * abs(r)           % Yaw: CFD MRF 中网格 (2026-06-18)
    ]);
end
