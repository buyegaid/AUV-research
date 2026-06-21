function [tau_drag, D_out] = remus_drag(nu_r)
% REMUS_DRAG REMUS 100 阻尼力/力矩计算（包装器）
%   接口与 xhy_drag_cfd 一致: [tau_drag, D] = remus_drag(nu_r)
%   2026-06-11

% REMUS 100 几何参数（常量）
L_auv = 1.6;             % AUV长度 (m)
D_auv = 0.19;            % AUV直径 (m)
S = 0.7 * L_auv * D_auv; % 投影面积
a_sp = 1.0096 * L_auv/2; % 椭球体半轴
b_sp = 1.0096 * D_auv/2;

u = nu_r(1); v = nu_r(2); w = nu_r(3);
U_r = sqrt(u^2 + v^2 + w^2);
alpha = atan2(w, u);

% 寄生阻力系数
Cd = 0.42;
CD_0 = Cd * pi * b_sp^2 / S;

% 线性阻尼矩阵
T1 = 20; T2 = 20; T6 = 1;
zeta4 = 0.3; zeta5 = 0.8;
r44 = 0.3;
r_bG = [0 0 0.02]';
r_bB = [0 0 0]';

nu_zero = zeros(6,1);
[MRB, ~] = spheroid(a_sp, b_sp, nu_zero(4:6), r_bG);
[MA, ~] = imlay61(a_sp, b_sp, nu_zero, r44);
m = MRB(1,1);
mu = 63.446827; g_mu = gravity(mu);
W = m * g_mu;

D_linear = Dmtrx([T1 T2 T6], [zeta4 zeta5], MRB, MA, [W r_bG' r_bB']);
D_linear(1,1) = D_linear(1,1) * exp(-3*U_r);
D_linear(2,2) = D_linear(2,2) * exp(-3*U_r);

% 攻角相关升力/阻力 + 横流阻力
tau_liftdrag = forceLiftDrag(D_auv, S, CD_0, alpha, U_r);
tau_crossflow = crossFlowDrag(L_auv, D_auv, D_auv, nu_r, 'cylinder');

% 总阻尼力/力矩
tau_drag = -D_linear * nu_r + tau_liftdrag + tau_crossflow;
D_out = D_linear;
end
