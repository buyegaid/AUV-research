% M060 推进器模型示例

clear; clc;

model_dir = fileparts(mfilename('fullpath'));
project_root = fullfile(model_dir, '..', '..');
project_root = setup_paths();

params = m060_thruster_params();
pwm_us = linspace(params.pwm_min_us, params.pwm_max_us, 301);
voltage_v = 24.0;

state = m060_thruster_model(pwm_us, voltage_v, params);
zero_state = m060_thruster_model(1500, voltage_v, params);

fprintf('PWM=1150 us, 24 V: T = %.2f N, Q_shaft = %.4f N*m\n', ...
    state.thrust_n(1), state.shaft_torque_nm(1));
fprintf('PWM=1500 us, 24 V: T = %.2f N, Q_shaft = %.4f N*m\n', ...
    zero_state.thrust_n, zero_state.shaft_torque_nm);
fprintf('PWM=1850 us, 24 V: T = %.2f N, Q_shaft = %.4f N*m\n', ...
    state.thrust_n(end), state.shaft_torque_nm(end));

figure('Name', 'M060 Thruster Model');

subplot(2, 1, 1);
plot(pwm_us, state.thrust_n, 'LineWidth', 1.5);
grid on;
xlabel('PWM / us');
ylabel('Thrust / N');
title('M060 PWM 到轴向推力');

subplot(2, 1, 2);
plot(pwm_us, state.shaft_torque_nm, 'LineWidth', 1.5);
grid on;
xlabel('PWM / us');
ylabel('Shaft torque / N*m');
title('M060 PWM 到桨轴扭矩');
