function [D, tau_Dv] = drag(nu_r, U_r, MRB, MA, W, r_bG, r_bB)
%DRAG Compute linear damping matrix and D(v)*v drag term.
%   [D, tau_Dv] = drag(nu_r, U_r, MRB, MA, W, r_bG, r_bB)
%   Returns the linear damping matrix D and the generalized drag
%   force vector D * nu_r.

% Low-speed linear damping matrix parameters
T1 = 20;
T2 = 20;
T6 = 1;
zeta4 = 0.3;
zeta5 = 0.8;

D = Dmtrx([T1 T2 T6],[zeta4 zeta5],MRB,MA,[W r_bG' r_bB']);
D(1,1) = D(1,1) * exp(-3 * U_r);
D(2,2) = D(2,2) * exp(-3 * U_r);

tau_Dv = D * nu_r;
end
