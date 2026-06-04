function [C, g] = compute_cg_standalone(nu_r, psi, M)
% 计算Coriolis矩阵和重力/浮力向量（独立函数版本）
m = 33; Ix = 0.540804; Iy = 2.107488; Iz = 1.849137;
Ig = diag([Ix Iy Iz]);
nu2 = nu_r(4:6); O3 = zeros(3,3);
CRB = [m*Smtrx(nu2), O3; O3, -Smtrx(Ig*nu2)];
MA = diag([11.2773, 132.1086, 47.104, 0.006, 0.043, 0.138]);
CA = m2c(MA, nu_r);
CA(5,3)=0; CA(3,5)=0; CA(5,1)=0; CA(1,5)=0; CA(6,1)=0; CA(1,6)=0;
C = CRB + CA;
mu = 63.446827; g_mu = gravity(mu);
W = m * g_mu; B = W * 1.01;
[~, R] = eulerang(0, 0, psi);
g = gRvect(W, B, R, [0;0;0], [0;0;-0.03]);
end
