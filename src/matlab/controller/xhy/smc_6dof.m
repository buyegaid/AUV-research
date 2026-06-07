function [tau_cmd, debug] = smc_6dof(nu, nu_d, nu_d_dot, M, C_nu, D_nu, g_vec, hat_d, params)
%SMC_6DOF 6-DOF sliding mode controller for XHY AUV.
%
%   tau_cmd = smc_6dof(nu, nu_d, nu_d_dot, M, C_nu, D_nu, g_vec, hat_d, params)
%
%   Dynamics: M*nu_dot + C(nu)*nu_r + D(nu)*nu_r + g = tau
%   Control law (model-based SMC):
%     tau = M*(nu_d_dot - Lambda*e - Kd*sigma - Ks*sat(sigma/phi_b))
%           + C*nu_r + D*nu_r + g - rho_eso * hat_d_force
%
%   Inputs:
%     nu:        6x1 body velocities [u v w p q r]'
%     nu_d:      6x1 desired velocities
%     nu_d_dot:  6x1 desired accelerations
%     M:         6x6 inertia matrix (MRB + MA)
%     C_nu:      6x1 Coriolis+centripetal force C(nu_r)*nu_r
%     D_nu:      6x1 damping force D(nu_r)*nu_r
%     g_vec:     6x1 restoring force vector
%     hat_d:     6x1 ESO disturbance estimate (acceleration units)
%     params:    struct with fields:
%       .lambda  6x1 sliding surface slope
%       .Kd      6x1 linear gain on sigma
%       .Ks      6x1 switching gain
%       .phi_b   6x1 boundary layer thickness
%       .rho_eso 6x1 ESO compensation ratio (0~1)
%
%   Outputs:
%     tau_cmd: 6x1 commanded force/moment vector
%     debug:   struct with sigma, e

persistent e_int
if isempty(e_int)
    e_int = zeros(6,1);
end

lambda = params.lambda(:);
Kd     = params.Kd(:);
Ks     = params.Ks(:);
phi_b  = params.phi_b(:);
rho    = params.rho_eso(:);

% velocity error
e = nu - nu_d;

% sliding surface: sigma = e + lambda .* e_int
sigma = e + lambda .* e_int;

% saturation
sat_sigma = zeros(6,1);
for k = 1:6
    ratio = sigma(k) / phi_b(k);
    if abs(ratio) > 1
        sat_sigma(k) = sign(sigma(k));
    else
        sat_sigma(k) = ratio;
    end
end

% equivalent + switching control in acceleration space
nu_cmd_ddot = nu_d_dot - lambda .* e - Kd .* sigma - Ks .* sat_sigma;

% ESO feedforward compensation (in force space)
tau_eso = -M * (rho .* hat_d);

% total commanded force
tau_cmd = M * nu_cmd_ddot + C_nu + D_nu + g_vec + tau_eso;

% update integral
e_int = e_int + params.dt * e;

% debug output
debug.sigma = sigma;
debug.e = e;
debug.tau_eso = tau_eso;
debug.e_int = e_int;

end
