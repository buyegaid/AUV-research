function params = m060_thruster_params()
%M060_THRUSTER_PARAMS M060 推进器模型默认参数
%
% 默认推力表采用 FullDepth M060 公开静水推力参数:
%   正推: 12/16/24 V -> 1.0/1.5/3.0 kgf
%   反推: 12/16/24 V -> 0.7/1.1/2.2 kgf
%
% 扭矩相关参数采用工程估计:
%   KV 参考常见 M060 转售规格 350 rpm/V。
%   最大功率采用 150 W。FullDepth 表中 24 V 最大电流 6 A, 即约 144 W,
%   与 150 W 量级一致。
%
% 若有实测电流、转速或扭矩数据, 应在这里替换。

kgf_to_n = 9.80665;

params = struct();

params.pwm_min_us = 1150.0;
params.pwm_dead_low_us = 1485.0;
params.pwm_mid_us = 1500.0;
params.pwm_dead_high_us = 1535.0;
params.pwm_max_us = 1850.0;

params.kv_rpm_per_v = 350.0;
params.max_power_w = 150.0;
params.max_voltage_v = 24.0;
params.shaft_efficiency = 0.70;
params.speed_load_factor = 0.75;
params.power_exponent = 3.0;
params.min_abs_command_for_torque = 0.03;

% 推力标定增益（默认 1.0 = 不缩放，由水池标定参数覆盖）
params.thrust_gain_forward = 1.0;
params.thrust_gain_reverse = 1.0;

params.forward_voltage_v = [12.0, 16.0, 24.0];
params.forward_thrust_n = [1.0, 1.5, 3.0] .* kgf_to_n;

params.reverse_voltage_v = [12.0, 16.0, 24.0];
params.reverse_thrust_n = [0.7, 1.1, 2.2] .* kgf_to_n;
end
