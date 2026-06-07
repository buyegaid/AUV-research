function [initialObs, loggedSignals] = reset_vel_control()
clear my_PIDsurge
clear my_integralSMCheading
clear my_ALOS3D

%% ===== 基本设置 =====
loggedSignals.h = 0.01;
loggedSignals.maxSteps = 1000;
loggedSignals.stepCount = 0;
params = get_params();
params.h = 0.01;
loggedSignals.params = params;
loggedSignals.smallErrCount = 0;
loggedSignals.doneReason = "";

%% ===== 初始状态 =====
% x = [u v w p q r xN yE z phi theta psi]'
x0 = zeros(12,1);
x0(1) = 1.5;     % 初始前向速度
x0(6) = 0;
x0(12) = 0;
loggedSignals.x = x0;

%% ===== 参考速度 =====
% loggedSignals.u_d = 1.5+1*rand;        % 期望航速 (m/s)
loggedSignals.u_d = 1.6;
loggedSignals.u_d_dot = 0;      % 期望加速度
loggedSignals.n_d = 1000;       % 名义期望转速 (RPM)

%% ===== 初始控制输入 =====
loggedSignals.ui = [0;0;900];   % [delta_r; delta_s; n]
loggedSignals.n_prev = loggedSignals.ui(3);

%% ===== 初始误差 =====
u = x0(1);
udot = 0;

e_u = loggedSignals.u_d - u;
e_udot = loggedSignals.u_d_dot - udot;

loggedSignals.eu0 = e_u;
loggedSignals.eudot0 = e_udot;
loggedSignals.u_prev = u;
loggedSignals.udot_prev = udot;
loggedSignals.e_u_prev = e_u;
%% ===== 初始观测 =====
initialObs = [
    e_u
    e_udot
    u
    udot
    loggedSignals.n_prev
    params.Kp(1)
    params.Ki(1)
    params.Kd(1)
    ];
end
