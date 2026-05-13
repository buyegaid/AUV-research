% Test script for XHY AUV dynamics model
% Simulates 100 seconds with fixed thruster inputs
% Plots trajectory, attitude, and process variables

clear all; close all; clc;

% Add path to access xhy function
addpath('..');
addpath('../Lib');

% Simulation parameters
tspan = [0 100];  % 100 seconds
dt = 0.1;         % time step
t = tspan(1):dt:tspan(2);

% Initial state: [u v w p q r x y z phi theta psi]'
x0 = zeros(12,1);
x0(12) = 0;  % initial yaw angle (rad)

% Fixed thruster inputs (RPM)
% ui = [n_main, n_vert1, n_vert2, n_side1, n_side2]'
ui_fixed = [1000; 500; 500; 200; 200];  % example fixed inputs

% Ocean currents (optional, set to zero)
Vc = 0;
betaVc = 0;
w_c = 0;

% ODE function handle
ode_fun = @(t, x) xhy_ode(t, x, ui_fixed, Vc, betaVc, w_c);

% Simulate using ode45
[t_sim, x_sim] = ode45(ode_fun, tspan, x0);

% Extract states
u = x_sim(:,1);
v = x_sim(:,2);
w = x_sim(:,3);
p = x_sim(:,4);
q = x_sim(:,5);
r = x_sim(:,6);
x_pos = x_sim(:,7);
y_pos = x_sim(:,8);
z_pos = x_sim(:,9);
phi = x_sim(:,10);
theta = x_sim(:,11);
psi = x_sim(:,12);

% Plot trajectory
figure(1);
plot3(x_pos, y_pos, z_pos);
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title('AUV Trajectory');
grid on;

% Plot attitude
figure(2);
subplot(3,1,1);
plot(t_sim, rad2deg(phi));
ylabel('Roll (\phi) (deg)');
title('Attitude Angles');
grid on;

subplot(3,1,2);
plot(t_sim, rad2deg(theta));
ylabel('Pitch (\theta) (deg)');
grid on;

subplot(3,1,3);
plot(t_sim, rad2deg(psi));
ylabel('Yaw (\psi) (deg)');
xlabel('Time (s)');
grid on;

% Plot velocities
figure(3);
subplot(3,1,1);
plot(t_sim, u);
ylabel('Surge (u) (m/s)');
title('Linear Velocities');
grid on;

subplot(3,1,2);
plot(t_sim, v);
ylabel('Sway (v) (m/s)');
grid on;

subplot(3,1,3);
plot(t_sim, w);
ylabel('Heave (w) (m/s)');
xlabel('Time (s)');
grid on;

% Plot angular velocities
figure(4);
subplot(3,1,1);
plot(t_sim, rad2deg(p));
ylabel('Roll rate (p) (deg/s)');
title('Angular Velocities');
grid on;

subplot(3,1,2);
plot(t_sim, rad2deg(q));
ylabel('Pitch rate (q) (deg/s)');
grid on;

subplot(3,1,3);
plot(t_sim, rad2deg(r));
ylabel('Yaw rate (r) (deg/s)');
xlabel('Time (s)');
grid on;

% ODE wrapper function
function xdot = xhy_ode(t, x, ui, Vc, betaVc, w_c)
[xdot, ~, ~, ~, ~, ~, ~] = xhy(x, ui, Vc, betaVc, w_c);
end
