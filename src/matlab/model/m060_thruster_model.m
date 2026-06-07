function state = m060_thruster_model(pwm_us, voltage_v, params)
%M060_THRUSTER_MODEL M060 深水推进器 PWM 到推力/扭矩的准静态模型
%
% 输入:
%   pwm_us    - PWM 脉宽, 单位 us, 可为标量或数组
%   voltage_v - 供电电压, 单位 V, 默认 24 V, 可为标量或与 pwm_us 同尺寸
%   params    - 可选参数结构体, 可由 m060_thruster_params() 生成
%
% 输出 state 结构体:
%   command             - 归一化油门, 范围 [-1, 1]
%   thrust_n            - 轴向推力, 单位 N
%   shaft_torque_nm     - 桨轴输出扭矩, 单位 N*m
%   reaction_torque_nm  - 载体受到的反扭矩, 单位 N*m
%   electrical_power_w  - 估计输入电功率, 单位 W
%   shaft_power_w       - 估计轴功率, 单位 W
%   rpm                 - 估计负载转速, 单位 rpm
%   omega_rad_s         - 估计负载角速度, 单位 rad/s
%
% 建模假设:
%   1. 最大静推力来自公开 M060 参数表, 并按供电电压线性插值。
%   2. PWM 死区内输出为 0。
%   3. 静推力随归一化油门平方变化: T = Tmax * u^2。
%   4. 轴扭矩由轴功率和估计负载转速计算: Q = P_shaft / omega。
%   5. 公开资料没有完整效率、转速、电流、扭矩曲线, 因此扭矩是工程估计值。

if nargin < 2 || isempty(voltage_v)
    voltage_v = 24.0;
end

if nargin < 3 || isempty(params)
    params = m060_thruster_params();
end

pwm_us = double(pwm_us);
voltage_v = double(voltage_v);

command = pwm_to_command(pwm_us, params);

if isscalar(command) && ~isscalar(voltage_v)
    command = command + zeros(size(voltage_v));
    pwm_us = pwm_us + zeros(size(voltage_v));
elseif isscalar(voltage_v) && ~isscalar(command)
    voltage_v = voltage_v + zeros(size(command));
elseif ~isequal(size(command), size(voltage_v))
    error('pwm_us 和 voltage_v 必须同尺寸, 或其中一个为标量。');
end

abs_command = abs(command);
direction = sign(command);

max_forward_n = interp_clamped(voltage_v, ...
    params.forward_voltage_v, params.forward_thrust_n);
max_reverse_n = interp_clamped(voltage_v, ...
    params.reverse_voltage_v, params.reverse_thrust_n);

if isscalar(max_forward_n)
    max_forward_n = max_forward_n + zeros(size(command));
end
if isscalar(max_reverse_n)
    max_reverse_n = max_reverse_n + zeros(size(command));
end

max_thrust_n = max_forward_n;
max_thrust_n(command < 0) = max_reverse_n(command < 0);

thrust_n = direction .* max_thrust_n .* abs_command.^2;

% 水动力负载功率通常近似随转速三次方变化, 这里作为初始工程模型。
voltage_scale = min((voltage_v ./ params.max_voltage_v).^2, 1.0);
voltage_scale = max(voltage_scale, 0.0);
electrical_power_w = params.max_power_w .* abs_command.^params.power_exponent .* voltage_scale;
electrical_power_w = min(electrical_power_w, params.max_power_w);
shaft_power_w = electrical_power_w .* params.shaft_efficiency;

rpm = params.kv_rpm_per_v .* voltage_v .* params.speed_load_factor .* abs_command;
omega_rad_s = rpm .* 2.0 .* pi ./ 60.0;

shaft_torque_nm = zeros(size(command));
valid_torque = abs_command >= params.min_abs_command_for_torque & omega_rad_s > 0;
shaft_torque_nm(valid_torque) = shaft_power_w(valid_torque) ./ omega_rad_s(valid_torque);
shaft_torque_nm = direction .* shaft_torque_nm;

% 推进器对载体的反扭矩方向与桨轴输出扭矩相反。
reaction_torque_nm = -shaft_torque_nm;

state = struct();
state.pwm_us = pwm_us;
state.voltage_v = voltage_v;
state.command = command;
state.thrust_n = thrust_n;
state.shaft_torque_nm = shaft_torque_nm;
state.reaction_torque_nm = reaction_torque_nm;
state.electrical_power_w = electrical_power_w;
state.shaft_power_w = shaft_power_w;
state.rpm = rpm;
state.omega_rad_s = omega_rad_s;
end

function command = pwm_to_command(pwm_us, params)
%PWM_TO_COMMAND 将 PWM 脉宽映射为 [-1, 1] 归一化油门

command = zeros(size(pwm_us));

forward_idx = pwm_us > params.pwm_dead_high_us;
reverse_idx = pwm_us < params.pwm_dead_low_us;

forward_span = params.pwm_max_us - params.pwm_dead_high_us;
reverse_span = params.pwm_dead_low_us - params.pwm_min_us;

command(forward_idx) = ...
    (pwm_us(forward_idx) - params.pwm_dead_high_us) ./ forward_span;
command(reverse_idx) = ...
    -(params.pwm_dead_low_us - pwm_us(reverse_idx)) ./ reverse_span;

command = min(max(command, -1.0), 1.0);
end

function y = interp_clamped(x, xp, yp)
%INTERP_CLAMPED 线性插值, 超出表格范围时夹紧到边界值

y = interp1(xp, yp, x, 'linear', 'extrap');
y(x <= xp(1)) = yp(1);
y(x >= xp(end)) = yp(end);
end
