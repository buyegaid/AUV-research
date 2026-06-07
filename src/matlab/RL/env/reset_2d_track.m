function [initObs, loggedSignals] = reset_2d_track()
% 重置环境状态
clear my_integralSMCheading   % 清除积分状态
clear my_SMCsurge       % 清除surge状态
clear my_ALOS3D
loggedSignals.h = 0.01; % 时间步长
% auv状态变量
loggedSignals.x = [1;0;0;0;0;0;0;0;0;0;0;0]; % 初始状态变量 12*1维
%   u:       Surge velocity          (m/s)
%   v:       Sway velocity           (m/s)
%   w:       Heave velocity          (m/s)
%   p:       Roll rate               (rad/s)
%   q:       Pitch rate              (rad/s)
%   r:       Yaw rate                (rad/s)
%   x:       North position          (m)
%   y:       East position           (m)
%   z:       Downwards position      (m)
%   phi:     Roll angle              (rad)
%   theta:   Pitch angle             (rad)
%   psi:     Yaw angle               (rad)

% 初始控制输入
loggedSignals.ui = [0;0;600];       % 初始控制输入 3*1维
% [delta_r; delta_s; n] = [垂直舵角度; 水平舵角度; 推进器转速]

loggedSignals.psi_ref = 0;
loggedSignals.psi_d = 0;              % 期望航向角
loggedSignals.r_d   = 0;              % 期望偏航角速度
loggedSignals.a_d   = 0;              % 期望偏航角加速度
loggedSignals.u_d = 1.2;              % 期望航速 (m/s)
loggedSignals.u_d_dot = 0;            % 期望加速度
loggedSignals.n_d = 1300;             % 期望转速 (RPM)

% 初始参数
params.K_d(1) = 0; % 误差项增益
params.K_sigma(1) = 0.05; % 滑模面增益
params.lambda(1) = 0.1; % 滑模面权重
params.phi_b(1) = 0.1; % 边界层厚度

params.K_d(3)      = 0.35;    % 速度通道（第3通道）
params.K_sigma(3)  = 0.05;
params.lambda(3)   = 0.1;
params.phi_b(3)    = 0.1;
params.K_nomoto(1) = 5/20; % Nomoto 模型增益
params.T_nomoto(1) = 1; % Nomoto 模型时间常数
params.m      = 31.9;     % 示例值，你替换为真实参数
params.Xu_dot = -5.5;
params.d1     = 2.5;
params.d2     = 3.0;
params.K_T    = 0.012;


params.Vc = 0; % 海流速度
params.betaVc = 0; % 海流迎角
params.wc = 0; % 垂向海流速度

% 一些固定参数
params.delta_max = deg2rad(20);       % 舵机最大角度（rad）
% 初始化推进器参数
params.n_rate = 0.1;                  % 速率更新限制 (RPM per update)
params.n_max = 1525;                  % 最大推进器速度 (RPM)
params.delta_max = deg2rad(20);       % 舵机最大角度（rad）
params.r_max = deg2rad(5.0);          % 最大允许转弯速率 Maximum allowable rate of turn (rad/s)
params.q_max = deg2rad(5.0);          % 最大允许俯仰速率 Maximum allowable pitch rate (rad/s)
loggedSignals.params = params;

% 3DALOS路径跟踪参数
alos_params.delta_h = 20;            % 水平前视距离 (m)
alos_params.delta_v = 20;            % 垂直前视距离 (m)
alos_params.gamma_h = 0.001;         % 自适应增益，水平方向
alos_params.gamma_v = 0.001;         % 自适应增益，垂直方向
alos_params.M_theta = deg2rad(20);   % 最大估计角 (rad)
alos_params.R_switch = 10;           % 航点切换半径 (m)
alos_params.K_f = 0.5;               % LOS观测器的增益 LOS observer gain
loggedSignals.alos_params = alos_params;

% 航点
wpt.pos.x = [0  50  100  150]';
wpt.pos.y = [0  0   30   30 ]';
wpt.pos.z = [0  0    0    0 ]';  % 仅水平面示例
loggedSignals.wpt = wpt;

initObs = zeros(10,1);

end