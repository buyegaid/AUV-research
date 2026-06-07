function [n, tau_out] = thrust(tau)
%THRUST Convert desired force/torque vector to thruster speeds.
%   [n, tau_out] = thrust(tau) computes the thruster speeds n (RPM) and
%   the actual force/torque vector tau_out for the given desired tau.
%
%   tau: 6x1 desired force/torque vector [X Y Z K M N]'
%   n: 5x1 thruster speeds [n_main n_vert1 n_vert2 n_side1 n_side2]' (RPM)
%   tau_out: 6x1 actual force/torque vector

% Thruster parameters
rho = 1026;             % Water density (kg/m^3)
D_prop = 0.10;          % Thruster diameter (m)
KT = 0.22;              % Thrust coefficient

% Thruster geometry
x_vert_f = +0.344;      % Fore vertical thruster x-position (m)
x_vert_r = -0.293;      % Aft vertical thruster x-position (m)
x_side_f = +0.424;      % Fore side thruster x-position (m)
x_side_r = -0.376;      % Aft side thruster x-position (m)

% Thruster allocation matrix (tau = B_thr * T_vec)
B_thr = [
    1,  0,           0,           0,           0;
    0,  0,           0,           1,           1;
    0,  1,           1,           0,           0;
    0,  0,           0,           0,           0;
    0, -x_vert_f,   -x_vert_r,    0,           0;
    0,  0,           0,           x_side_f,    x_side_r
    ];

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