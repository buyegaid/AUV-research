function [initialObs, loggedSignals] = reset_pitch_control()
clear my_integralSMCpitch   % 清除积分状态
clear my_SMCsurge           % 清除surge状态
clear my_ALOS3D

%% ===== 基本设置 =====
loggedSignals.h = 0.01;
loggedSignals.maxSteps = 1000;
loggedSignals.stepCount = 0;
params = get_params();
loggedSignals.params = params;
loggedSignals.successHoldCount = 0;
loggedSignals.doneReason = "";

%% ===== 初始状态 =====
% x = [u v w p q r xN yE z phi theta psi]'
x = zeros(12,1);
x(1)  = 1.0 + 0.2*rand;              % [0.9,1.1]
x(5)  = deg2rad(-1 + 2*rand);        % 小范围初始俯仰角速度
x(11) = deg2rad(-5 + 10*rand);       % 小范围初始俯仰角扰动
loggedSignals.x = x;

%% ===== 参考俯仰 =====
loggedSignals.theta_d = deg2rad(-15+30*rand); % 期望俯仰角
loggedSignals.q_d       = 0;              % 期望俯仰角速度
loggedSignals.a_d       = 0;              % 期望俯仰角加速度
loggedSignals.u_d       = 1.2;            % 期望航速 (m/s)
loggedSignals.u_d_dot   = 0;              % 期望加速度
loggedSignals.n_d       = 1000;           % 期望转速 (RPM)
% 初始控制输入
loggedSignals.ui = [0;0;900];           % 初始控制输入 3*1维
% [delta_r; delta_s; n] = [垂直舵角度; 水平舵角度; 推进器转速]
loggedSignals.prev_e_theta = wrapToPi(x(11) - loggedSignals.theta_d);
loggedSignals.prev_sigma   = 0;
% loggedSignals.prevDelta = 0;
% loggedSignals.prevAction = zeros(4,1);
loggedSignals.theta_int = 0; % 同步保存积分误差
%% ===== 初始观测 =====
theta = x(11);
q     = x(5);
u     = x(1);

e_theta = wrapToPi(ssa(theta - loggedSignals.theta_d));
e_q     = q;
sigma     = e_q + params.pitch.lambda*e_theta;

initialObs = [
    e_theta
    e_q
    sigma
    theta
    0
    0
    params.pitch.K_d
    params.pitch.K_sigma
    params.pitch.lambda
    params.pitch.phi_b
    ];
end