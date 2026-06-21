function [C, g] = remus_coriolis_gravity(nu_r, psi)
% REMUS_CORIOLIS_GRAVITY REMUS 100 Coriolis矩阵+恢复力
%   接口: [C, g] = remus_coriolis_gravity(nu_r, psi)
%   2026-06-11

L_auv = 1.6; D_auv = 0.19;
a_sp = 1.0096 * L_auv/2;
b_sp = 1.0096 * D_auv/2;
r44 = 0.3;
r_bG = [0 0 0.02]';
r_bB = [0 0 0]';

[MRB, CRB] = spheroid(a_sp, b_sp, nu_r(4:6), r_bG);
[MA, CA] = imlay61(a_sp, b_sp, nu_r, r44);

% 二次旋转阻尼缺失项置零（与remus100.m一致）
CA(5,3) = 0; CA(3,5) = 0;
CA(5,1) = 0; CA(1,5) = 0;
CA(6,1) = 0; CA(1,6) = 0;
CA(6,2) = 0; CA(2,6) = 0;

C = CRB + CA;

% 恢复力
m = MRB(1,1);
mu = 63.446827; g_mu = gravity(mu);
W = m * g_mu; B = W;
[~, R] = eulerang(0, 0, psi);
g = gRvect(W, B, R, r_bG, r_bB);
end
