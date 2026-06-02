function [tau_drag, D] = xhy_drag_cfd(nu_r, M)
% XHY_DRAG_CFD 阻力计算（实测数据校准 2026-05-31）
%   校准数据: debug_data260530-1.csv
%   Surge: 时域系统辨识 (R²=0.30, 正向仿真R²=0.64)
%   Sway/Heave/Yaw: 最大速度匹配校准
%
u = nu_r(1); v = nu_r(2); w = nu_r(3);
p = nu_r(4); q = nu_r(5); r = nu_r(6);

% 平动阻力
% Surge: 时域辨识 d1=1.35, d2=1.62 (原CFD: d1=0.76, d2=3.77)
Fx = -(1.3451 * u + 1.6172 * u * abs(u));

% Sway: 最大速度校准 d2=87.9 (原CFD: 142.85, 实测v_max≈0.34m/s)
% 注：线性项为0是校准结果，横荡阻力在AUV上以形状阻力（二次项）为主，极低速时摆荡小可忽略
Fy = -(0.0 * v + 87.9 * v * abs(v));

% Heave: 最大速度+浮力校准 d2=67.3 (原CFD: 45.04, 实测w_max≈0.31m/s)
Fz = -(1.4500 * w + 67.3000 * w * abs(w));

% 转动阻尼
zeta4 = 0.3;
zeta5 = 0.8;
W = 33 * 9.81;
r_bg_z = 0;
r_bb_z = -0.03;
w4 = sqrt(W * (r_bg_z - r_bb_z) / M(4,4));
w5 = sqrt(W * (r_bg_z - r_bb_z) / M(5,5));
T6 = 1;

K  = -M(4,4) * 2 * zeta4 * w4 * p;
My = -M(5,5) * 2 * zeta5 * w5 * q;
% Yaw: 最大速度校准 ×2.4 (实测r_max≈52°/s, Fossen原始T6=1低估)
%  校准依据：水池试验偏航角速度数据，缩放因子匹配实测最大r
Nz = -M(6,6) / T6 * r * 2.4;

tau_drag = [Fx; Fy; Fz; K; My; Nz];

D = diag([
    1.3451 + 1.6172 * abs(u);
    0.0 + 87.9 * abs(v);
    1.45 + 67.30 * abs(w);
    M(4,4) * 2 * zeta4 * w4;
    M(5,5) * 2 * zeta5 * w5;
    M(6,6) / T6 * 2.4
    ]);
end
