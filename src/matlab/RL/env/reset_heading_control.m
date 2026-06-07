function [initialObs, loggedSignals] = reset_heading_control()
clear my_integralSMCheading   % 清除积分状态
clear my_SMCsurge       % 清除surge状态
clear my_ALOS3D

%% ===== 基本设置 =====
loggedSignals.h = 0.01;
loggedSignals.maxSteps = 1000;
loggedSignals.stepCount = 0;
params = get_params();
loggedSignals.params = params;
loggedSignals.smallErrCount = 0;
loggedSignals.doneReason = "";
%% ===== 初始状态 =====
% x = [u v w p q r xN yE z phi theta psi]'
x0 = zeros(12,1);
% x0(1) = 1.5;                           % 初始前向速度
% x0(6) = 0;
% x0(12) = 0;                         % 初始航向设为0
x0(1)  = 1.5 + 0.2*rand;              % [0.9,1.1]
x0(6)  = deg2rad(-1 + 2*rand);        % 小范围初始偏航角速度
x0(12) = deg2rad(-5 + 10*rand);       % 小范围初始航向扰动
loggedSignals.x = x0;

%% ===== 参考航向 =====
loggedSignals.psi_ref = deg2rad(-30+60*rand);
loggedSignals.psi_d = 0;              % 期望航向角
loggedSignals.r_d   = 0;              % 期望偏航角速度
loggedSignals.a_d   = 0;              % 期望偏航角加速度
loggedSignals.u_d = 1.2;              % 期望航速 (m/s)
loggedSignals.u_d_dot = 0;            % 期望加速度
loggedSignals.n_d = 1000;             % 期望转速 (RPM)
% 初始控制输入
loggedSignals.ui = [0;0;900];       % 初始控制输入 3*1维
% [delta_r; delta_s; n] = [垂直舵角度; 水平舵角度; 推进器转速]

loggedSignals.epsi0 = wrapToPi(x0(12) - loggedSignals.psi_ref); % 初始航向误差
loggedSignals.delta_r_prev = 0;
loggedSignals.e_psi_prev   = loggedSignals.epsi0;
%% ===== 初始观测 =====
psi = x0(12);
r = x0(6);
u = x0(1);

e_psi = wrapToPi(ssa(psi - loggedSignals.psi_ref));
e_r = r;

initialObs = [
    e_psi
    e_r
    0
    psi
    r
    u
    params.K_d(1)
    params.K_sigma(1)
    params.lambda(1)
    params.phi_b(1)
    ];
end
