% XHY AUV 动力学模型测试
% 使用实物推进器配置: 主推 M080, 辅推 M060, 输入为 PWM 脉宽和供电电压。
% 通过 m080_thruster_model / m060_thruster_model 计算推力后驱动 XHY 动力学。

clear all; close all; clc;

script_dir = fileparts(mfilename('fullpath'));
project_root = fullfile(script_dir, '..', '..');
addpath(project_root, fullfile(project_root, 'Lib'), fullfile(project_root, 'guidance'), ...
    fullfile(project_root, 'controller', 'xhy'), fullfile(project_root, 'controller', 'remus'), ...
    fullfile(project_root, 'model'), fullfile(project_root, 'model', 'params'), ...
    fullfile(project_root, 'model', 'test'), fullfile(project_root, 'eso'), ...
    fullfile(project_root, 'post'), fullfile(project_root, 'traj'));

% Simulation parameters
tspan = [0 100];  % 100 seconds
dt = 0.1;         % time step
t = tspan(1):dt:tspan(2);
voltage_v = 24;

% Initial state: [u v w p q r x y z phi theta psi]'
x0 = zeros(12,1);
x0(12) = 0;  % initial yaw angle (rad)

% 固定推进器输入 PWM(us)
% 顺序: [M080主推, M060前垂推, M060后垂推, M060前侧推, M060后侧推]'
pwm_fixed_us = [1650; 1500; 1500; 1600; 1600];

% Ocean currents (optional, set to zero)
Vc = 0;
betaVc = 0;
w_c = 0;

% ODE function handle
ode_fun = @(t, x) xhy_ode(t, x, pwm_fixed_us, voltage_v, Vc, betaVc, w_c);

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

fprintf('XHY PWM动力学测试完成: 主推 M080, 辅推 M060, 输入电压 %.1f V\n', voltage_v);
fprintf('固定PWM = [%.0f %.0f %.0f %.0f %.0f] us\n', pwm_fixed_us);

% ODE wrapper function
function xdot = xhy_ode(t, x, pwm_us, voltage_v, Vc, betaVc, w_c)
xdot = xhy_pwm_test_step(x, pwm_us, voltage_v, Vc, betaVc, w_c);
end
