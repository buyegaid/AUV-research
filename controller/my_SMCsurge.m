function n = my_SMCsurge(u, u_d, u_d_dot,h, params)
% -----------------------------------------------------------
% Surge SMC based directly on REMUS100 dynamics
% 速度控制器，基于REMUS100动力学的SMC
% Control input: propeller speed n (rps)
% -----------------------------------------------------------
persistent z   % integral of surge speed error

% ===== Initialization =====
if isempty(z)
    z = 0;
end
% ===== Parameters =====
m       = params.m;
Xu_dot  = params.Xu_dot;
d1      = params.d1;
d2      = params.d2;
K_T     = params.K_T;

k_d     = params.K_d;
k_sigma = params.K_sigma;
lambda  = params.lambda;
phi_b   = params.phi_b;

% ===== Errors =====
e_u = u - u_d;
% if e_u < 0.1
z   = z + h * e_u;
% end
% ===== Sliding surface =====
sigma = e_u + lambda * z;

% ===== Saturation =====
if abs(sigma/phi_b) > 1
    sat_sigma = sign(sigma);
else
    sat_sigma = sigma / phi_b;
end

% ===== Hydrodynamic drag =====
X_D = d1 * u + d2 * abs(u) * u;

% ===== Commanded thrust =====
X_cmd = (m - Xu_dot) * ...
    ( u_d_dot ...
    - lambda * e_u ...
    - k_d * sigma ...
    - k_sigma * sat_sigma ) ...
    + X_D;

% ===== Inverse propeller model =====
n = sign(X_cmd) * sqrt( abs(X_cmd) / max(K_T, eps) );

end
