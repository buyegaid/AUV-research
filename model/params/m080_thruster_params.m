function params = m080_thruster_params()
%M080_THRUSTER_PARAMS M080 推进器模型默认参数
%
% 推力表来自公开 M080 静水推力参数:
%   正推: 12/16/24 V -> 2.8/4.0/6.8 kgf
%   反推: 12/16/24 V -> 1.7/2.8/4.7 kgf
%
% 扭矩相关参数为工程估计值。若有实测电流、转速或扭矩数据, 应在这里替换。

kgf_to_n = 9.80665;

params = struct();

params.pwm_min_us = 1150.0;
params.pwm_dead_low_us = 1485.0;
params.pwm_mid_us = 1500.0;
params.pwm_dead_high_us = 1535.0;
params.pwm_max_us = 1850.0;

params.kv_rpm_per_v = 270.0;
params.max_power_w = 360.0;
params.max_voltage_v = 24.0;
params.shaft_efficiency = 0.70;
params.speed_load_factor = 0.75;
params.power_exponent = 3.0;
params.min_abs_command_for_torque = 0.03;

% 推力标定增益（默认 1.0 = 不缩放，由水池标定参数覆盖）
params.thrust_gain_forward = 1.0;
params.thrust_gain_reverse = 1.0;

params.forward_voltage_v = [12.0, 16.0, 24.0];
params.forward_thrust_n = [2.8, 4.0, 6.8] .* kgf_to_n;

params.reverse_voltage_v = [12.0, 16.0, 24.0];
params.reverse_thrust_n = [1.7, 2.8, 4.7] .* kgf_to_n;
end
