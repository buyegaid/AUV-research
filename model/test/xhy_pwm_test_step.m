function [xdot, tau_thr, thr] = xhy_pwm_test_step(x, pwm_us, voltage_v, Vc, betaVc, w_c)
%XHY_PWM_TEST_STEP 使用 M080/M060 PWM 推进器模型计算 XHY 动力学一步导数。
%
% 输入:
%   x         12x1 状态 [u v w p q r x y z phi theta psi]'
%   pwm_us    5x1 PWM 脉宽, 顺序 [M080主推 M060前垂推 M060后垂推 M060前侧推 M060后侧推]'
%   voltage_v 供电电压(V), 标量或 5x1
%   Vc,betaVc,w_c 海流参数, 与 xhy.m 保持一致
%
% 输出:
%   xdot      12x1 状态导数
%   tau_thr   6x1 由推进器产生的力/力矩
%   thr       推进器模型输出诊断结构体

if nargin < 3 || isempty(voltage_v), voltage_v = 24; end
if nargin < 4 || isempty(Vc), Vc = 0; end
if nargin < 5 || isempty(betaVc), betaVc = 0; end
if nargin < 6 || isempty(w_c), w_c = 0; end

pwm_us = pwm_us(:);
if numel(pwm_us) ~= 5
    error('xhy_pwm_test_step:InvalidPwm', 'pwm_us 必须为 5 维向量。');
end

if isscalar(voltage_v)
    voltage_v = voltage_v * ones(5,1);
else
    voltage_v = voltage_v(:);
    if numel(voltage_v) ~= 5
        error('xhy_pwm_test_step:InvalidVoltage', 'voltage_v 必须为标量或 5 维向量。');
    end
end

% 先用零推进器调用 xhy, 获取当前状态下的基础动力学项。
[xdot, ~, M, ~, ~, ~, ~] = xhy(x, zeros(5,1), Vc, betaVc, w_c);

main_state  = m080_thruster_model(pwm_us(1), voltage_v(1));
vert1_state = m060_thruster_model(pwm_us(2), voltage_v(2));
vert2_state = m060_thruster_model(pwm_us(3), voltage_v(3));
side1_state = m060_thruster_model(pwm_us(4), voltage_v(4));
side2_state = m060_thruster_model(pwm_us(5), voltage_v(5));

T_vec = [
    main_state.thrust_n;
    vert1_state.thrust_n;
    vert2_state.thrust_n;
    side1_state.thrust_n;
    side2_state.thrust_n
    ];

% 推进器几何与 xhy.m 保持一致。
x_vert_f = +0.344;
x_vert_r = -0.293;
x_side_f = +0.424;
x_side_r = -0.376;

B_thr = [
    1,  0,         0,         0,        0;
    0,  0,         0,         1,        1;
    0,  1,         1,         0,        0;
    0,  0,         0,         0,        0;
    0, -x_vert_f, -x_vert_r,  0,        0;
    0,  0,         0,         x_side_f, x_side_r
    ];

tau_thr = B_thr * T_vec;
xdot(1:6) = xdot(1:6) + M \ tau_thr;

thr = struct();
thr.pwm_us = pwm_us;
thr.voltage_v = voltage_v;
thr.T_vec = T_vec;
thr.main = main_state;
thr.vert1 = vert1_state;
thr.vert2 = vert2_state;
thr.side1 = side1_state;
thr.side2 = side2_state;
end
