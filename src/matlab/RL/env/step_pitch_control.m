function [nextObs, reward, isDone, loggedSignals] = step_pitch_control(action,loggedSignals)
% controller: my_integralSMCpitch
% Method: TD3
if iscell(action)
    action = action{1};
end
action = max(min(action(:),1),-1);
isDone = false;

%% 获取当前状态
x = loggedSignals.x;
params = loggedSignals.params;
current = loggedSignals.params.current;
h = loggedSignals.h;
prevN = loggedSignals.ui(3); % 当前转速
prevDeltar = loggedSignals.ui(1); % 当前垂直舵角
prevDeltas = loggedSignals.ui(2); % 上一步水平舵角
q = x(5);       % 当前俯仰角速度
theta = x(11);  % 当前俯仰角
u = x(1);       % 当前前向速度

%% 参考俯仰
theta_d = loggedSignals.theta_d;
q_d     = loggedSignals.q_d;

%% 参数更新
% 从参数中读取上一步的参数
prevKd     = params.pitch.K_d;
prevKsigma = params.pitch.K_sigma;
prevLambda = params.pitch.lambda;
prevPhib   = params.pitch.phi_b;

theta_target.K_d     = param2ismc(action(1), params.pitch.kd_range(1), params.pitch.kd_range(2));
theta_target.K_sigma = param2ismc(action(2), params.pitch.ksigma_range(1), params.pitch.ksigma_range(2));
theta_target.lambda  = param2ismc(action(3), params.pitch.lambda_range(1),  params.pitch.lambda_range(2));
theta_target.phi_b   = param2ismc(action(4), params.pitch.phi_b_range(1),   params.pitch.phi_b_range(2));

beta = 0.1;  % 一阶平滑系数
% 参数平滑更新
params.pitch.K_d     = (1-beta)*prevKd      + beta*theta_target.K_d;
params.pitch.K_sigma = (1-beta)*prevKsigma + beta*theta_target.K_sigma;
params.pitch.lambda  = (1-beta)*prevLambda  + beta*theta_target.lambda;
params.pitch.phi_b   = (1-beta)*prevPhib   + beta*theta_target.phi_b;

%% 控制
delta_s_d = my_integralSMCpitch(theta, q, theta_d, q_d, loggedSignals.a_d, h, params.pitch);
delta_s = sat(delta_s_d, params.auv.delta_max); % 水平舵控制（俯仰轴）
delta_r = prevDeltar; % 垂直舵控制（偏航轴）保持0
n = prevN;
ui = [delta_r; -delta_s; n];

%% 动力学更新
x = rk4(@remus100, h, x, ui, current.Vc, current.betaVc, current.wc);
u = x(1);
q = x(5);
theta = x(11);
%% 误差计算
e_theta = wrapToPi(ssa(theta - theta_d));
e_q     = q - q_d;
% 计算滑模面 TODO 如果reward中的sigma和控制器不一致，建议同步保存积分误差，并在此处计算一致的sigma
sigma = e_q + 2*params.pitch.lambda*e_theta;

% 上一步量
prev_e_theta = loggedSignals.prev_e_theta;
prev_sigma   = loggedSignals.prev_sigma;
delta_delta = -delta_s - prevDeltas;

%% 参数变化率
dKd     = params.pitch.K_d     - prevKd;
dKsigma = params.pitch.K_sigma - prevKsigma;
dLambda = params.pitch.lambda  - prevLambda;
dPhib   = params.pitch.phi_b   - prevPhib;
%% 奖励函数
% -------- 归一化 --------
e_theta_n = e_theta / (deg2rad(20));         % 20 deg
e_q_n     = e_q     / (deg2rad(30));         % 30 deg/s
sigma_n   = sigma   / 0.8;                   % 可后续按数据再调
delta_n   = delta_s / params.auv.delta_max;
ddelta_n  = delta_delta / params.auv.delta_max;

dKd_n = dKd / max(params.pitch.kd_range(2)     - params.pitch.kd_range(1),     1e-6);
dKs_n = dKsigma / max(params.pitch.ksigma_range(2) - params.pitch.ksigma_range(1), 1e-6);
dLm_n = dLambda / max(params.pitch.lambda_range(2) - params.pitch.lambda_range(1), 1e-6);
dPb_n = dPhib / max(params.pitch.phi_b_range(2) - params.pitch.phi_b_range(1), 1e-6);

dTheta2 = dKd_n^2 + dKs_n^2 + dLm_n^2 + dPb_n^2;

% -------- 基础惩罚项 --------
r_track  = -2.0  * e_theta_n^2;   % 姿态误差主项
r_rate   = -0.5  * e_q_n^2;       % 角速度误差
r_slide  = -0.8  * sigma_n^2;     % 滑模面惩罚
r_ctrl   = -0.05 * delta_n^2;     % 舵角大小
r_smooth = -0.10 * ddelta_n^2;    % 舵角变化率/抖振
r_param  = -0.05 * dTheta2;       % 参数变化平滑性

% -------- 进步奖励 --------
r_progress_e = 0.5 * (abs(prev_e_theta) - abs(e_theta));
r_progress_s = 0.2 * (abs(prev_sigma)   - abs(sigma));
% -------- 稳态成功奖励 --------
r_success = 0;
if abs(e_theta) < deg2rad(1) && abs(e_q) < deg2rad(1)
    loggedSignals.successHoldCount = loggedSignals.successHoldCount + 1;
    r_success = r_success + 0.5;
else
    loggedSignals.successHoldCount = 0;
end

% 连续保持0.5s（50步）
if loggedSignals.successHoldCount >= 50
    isDone = true;
    r_success = r_success + 2.0;
end

% -------- 约束惩罚/提前终止 --------
r_violation = 0;
if abs(theta) > deg2rad(40) || abs(q) > deg2rad(60)
    r_violation = -20;
    isDone = true;
end
loggedSignals.stepCount = loggedSignals.stepCount + 1;
if ~isDone && loggedSignals.stepCount >= loggedSignals.maxSteps
    isDone = true;
end

% 总奖励
reward = r_track + r_rate + r_slide + r_ctrl + r_smooth + r_param ...
    + r_progress_e + r_progress_s + r_success + r_violation;

%% 下一观测
% 跟踪误差，误差导数，积分误差，滑膜变量，上一时刻控制输入，控制变化率，上一时刻参数
% 归一化操作
nextObs = [
    e_theta
    e_q
    sigma
    theta
    delta_s
    delta_delta
    params.pitch.K_d
    params.pitch.K_sigma
    params.pitch.lambda
    params.pitch.phi_b
    ];

%% 回写
loggedSignals.x = x;
loggedSignals.params = params;
loggedSignals.ui = ui;
loggedSignals.prev_e_theta = e_theta;
loggedSignals.prev_sigma   = sigma;

end