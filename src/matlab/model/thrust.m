function [n, tau_out] = thrust(tau)
%THRUST Convert desired force/torque vector to thruster speeds.
%   [n, tau_out] = thrust(tau) computes the thruster speeds n (RPM) and
%   the actual force/torque vector tau_out for the given desired tau.
%
%   tau: 6x1 desired force/torque vector [X Y Z K M N]'
%   n: 5x1 thruster speeds [T1 T2 T3 T4 T5]' (RPM)
%   tau_out: 6x1 actual force/torque vector

% Thruster parameters
rho = 1026;             % Water density (kg/m^3)
D_prop = 0.10;          % Thruster diameter (m)
KT = 0.22;              % Thrust coefficient

% 推进器几何矩阵与 xhy.m 保持一致。
[B_thr, ~] = xhy_thruster_geometry();

% Solve for thrust vector using pseudoinverse (least squares)
T_vec = pinv(B_thr) * tau;

% Compute thruster speeds from thrusts
% T = rho * D_prop^4 * KT * abs(n_rps) * n_rps
% n_rps = sign(T) * sqrt(abs(T) / (rho * D_prop^4 * KT))
n_rps = zeros(5,1);
for i = 1:5
    T = T_vec(i);
    if T ~= 0
        n_rps(i) = sign(T) * sqrt(abs(T) / (rho * D_prop^4 * KT));
    else
        n_rps(i) = 0;
    end
end

% Convert to RPM
n = n_rps * 60;

% Actual force/torque vector
tau_out = B_thr * T_vec;
end
