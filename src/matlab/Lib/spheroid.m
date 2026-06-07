function [MRB,CRB] = spheroid(a,b,nu2,r_bg)
% 计算长椭球体的刚性质量矩阵和科里奥利向心矩阵（机体系）
% [MRB,CRB] = spheroid(a,b,nu2,r_bg) computes the 6x6 rigid-body mass
% and Coriolis-centripetal matrices of a prolate spheroid of length
% L = 2 * a and diameter D = 2 * b. The spheroid can be used to approximate
% a cylinder-shaped autonomous underwater vehicle (AUV). In general
% nu = [u,v,w,p,q,r]', while linear and angular velocities are denoted by
% nu1 = [u, v, w]' and nu2 = [p, q, r]'. The CRB matrix is computed using
% the linear velocity-independent representation (Fossen 2021, Chapter 3.3.1)
% according to:
%
%  [MRB,CRB] = spheroid(a, b,[p, q, r]',[xg, yg, zg]')
%
% This is particular useful for the relative equations of motion where 对于相对运动方程尤其有用
% nu_r = nu - nu_c and nu_c = [u_c, v_c, w_c, 0, 0, 0]' is the irrotational
% current velocity. This implies that the AUV equations of motion
% expressed in the CO satisfies:
%
%  MRB * nudot + CRB(nu) * nu = MRB * nudot_r + CRB(nu_r) * nu_r = tau
%
% Outputs:
%  MRB:                 Rigid-body mass matrix 刚体质量矩阵
%  CRB = CRB(nu2):      Coriolis-centripetal matrix, independent of nu1 科里奥利-离心矩阵，与线速度无关
%
% Inputs:
%  a, b:                Semiaxes a > b 椭球长短轴
%  nu2 = [p, q, r]':    Angular velocity vector 角速度向量
%  r_bg:                r_bg = [xg, yg, zg]' vector from the CO to the CG 从CO到CG的向量
%
% Author:    Thor I. Fossen
% Date:      24 April 2021

O3 = zeros(3,3);

% Mass of spheriod  计算质量
rho = 1025;
m = 4/3 * pi * rho * a * b^2;

% Moment of inertia 计算转动惯量
Ix = (2/5) * m * b^2;
Iy = (1/5) * m * (a^2 + b^2);
Iz = Iy;
Ig = diag([Ix Iy Iz]);

% Rigid-body matrices expressed in the CG 在质心CG坐标系中构造刚体质量矩阵
MRB_CG = diag([ m m m Ix Iy Iz ]);

% 科氏-离心矩阵
CRB_CG = [ m * Smtrx(nu2)    O3
    O3               -Smtrx(Ig*nu2) ];

% Transform MRB and CRB from the CG to the CO 从质心转换到体心
H = Hmtrx(r_bg);
MRB = H' * MRB_CG * H;
CRB = H' * CRB_CG * H;








