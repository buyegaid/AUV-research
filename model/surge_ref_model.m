function [u_d, u_d_dot] = surge_ref_model(u_c, u_d,u_d_dot, omega,zeta, dt)
% 二阶参考模型（surge）
% xr = [u_d; u_d_dot]

u_d_ddot = ...
    -2*zeta*omega*u_d_dot ...
    -omega^2*u_d ...
    +omega^2*u_c;

% Euler / RK2 / RK4 均可
u_d = u_d     + dt*u_d_dot;
u_d_dot = u_d_dot + dt*u_d_ddot;
end
