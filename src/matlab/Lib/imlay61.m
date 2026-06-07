function [MA,CA] = imlay61(a,b,nu,r44)
% 计算长椭球体的6×6水动力附加质量矩阵MA和科里奥利附加质量及向心附加质量矩阵CA（机体系）
% [MA,CA] = imlay61(a,b,nu,r44) computes the 6x6 hydrodynamic added mass
% system matrix MA and the 6x6 added mass Coriolis and centripetal matrix
% CA for a prolate spheroid with semiaxes a > b using the Lamb's
% k-factors k1, k2 and k_prime (Fossen 2021, Section 8.4.2). The matrix MA
% is assumed to  be diagonal when the CO is chosen on the centerline
% midtships. The length of the AUV is L = 2*a while the diamater is D = 2*b.
%
% Inputs: a, b: spheroid semiaxes a > b 椭球长短轴
%         nu = [u, v, w, p, q, r]': generalized velocity vector 速度向量（机体系）
%         r44: hydrodynamic added moment MA(4,4) = r44 * Ix in roll. ；流体动力附加力矩
%              If r44 is not specified, MA(4,4) = 0.
%              Typicaly values for r44 are 0.2-0.4. 典型值是0.2-0.4之间
%
% Output: MA: 6x6 diagonal hydrodynamic added mass system matrix 水动力附加质量矩阵
%         CA: 6x6 hydrodynamic added Coriolis and centripetal matrix 科里奥利附加质量及向心附加质量矩阵
%
% Example: [MA,CA] = imlay61(a, b, [u,v,w,p,q,r]')
%          [MA,CA] = imlay61(a, b, [u,v,w,p,q,r]', r44)
%
% Refs: Lamb, H. (1932). Hydrodynamics. Cambridge University Press. London.
%       Imlay, F. H. (1961). The Complete Expressions for Added Mass of a
%          Rigid Body Moving in an Ideal Fluid. Technical Report DTMB 1528.
%          David Taylor Model Basin. Washington D.C.
%
% Author:     Thor I. Fossen
% Date:       24 Apr 2021
% Revisions:

% prolate spheroid formulas 椭球公式
rho = 1026;
m = 4/3 * pi * rho * a * b^2;
Ix = (2/5) * m * b^2;
Iy = (1/5) * m * (a^2 + b^2);

% Imlay (1961) gives a zero added moment in roll for a spheroid. This is
% compensated by manually specifying a nonzero r44 for other hull effects.
% MA(4,4) will in practise be nonzero due to control surfaces, propellers etc.
% Imlay（1961）给出球体在滚动中增加的力矩为零。通过手动指定非零的 r44 来补偿其他船体效果。由于控制面、螺旋桨等原因，MA（4,4）实际上将保持非零。
if (a < 0), error('a must be larger than 0'); end
if (b < 0), error('b must be larger than 0'); end
if (a <= b), error('a must be larger than b'); end

if (nargin == 3)
    r44 = 0;
end
MA_44 = r44 * Ix;

% Lamb's k-factors
e = sqrt(1-(b/a)^2);
alpha_0 = ( 2 * (1-e^2)/e^3 ) * ( 0.5 * log((1+e)/(1-e)) - e );
beta_0  = 1/e^2 - (1-e^2)/(2*e^3) * log((1+e)/(1-e));

k1 = alpha_0 / (2 - alpha_0);
k2 = beta_0  / (2 - beta_0);
k_prime = e^4*(beta_0-alpha_0) / ((2-e^2)*(2*e^2-(2-e^2)*(beta_0-alpha_0)));

% Added mass system matrix expressed in the CO
MA = diag([m*k1 m*k2 m*k2 MA_44 k_prime*Iy k_prime*Iy]);

% Added mass Coriolis and centripetal matrix expressed in the CO
CA = m2c(MA,nu);

