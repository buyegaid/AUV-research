function [nextObs, reward, isDone, loggedSignals] = step_heading_control(action, loggedSignals)
% controller: my_integralSMCheading
% Method: TD3
if iscell(action)
    action = action{1};
end
% 动作限幅
action = max(min(action(:),1),-1);
isDone = false;
loggedSignals.doneReason = "";

%% ===== 1. 当前状态 =====
x = loggedSignals.x;
params = loggedSignals.params;
h = loggedSignals.h;
n = loggedSignals.ui(3); % 当前转速
r   = x(6);
psi = x(12);

%% ===== 2. 参考航向 =====
psi_ref = loggedSignals.psi_ref;
r_d   = loggedSignals.r_d;

%% ===== 3. 参数更新 =====

theta_old = [params.K_d(1); params.K_sigma(1); params.lambda(1); params.phi_b(1)];
% theta_old = [params.K_sigma(1); params.phi_b(1)];
theta_target.K_d     = param2ismc(action(1), params.kd_range(1), params.kd_range(2));
theta_target.K_sigma = param2ismc(action(1), params.hksigma_range(1), params.hksigma_range(2));
theta_target.lambda  = param2ismc(action(3), params.hlambda_range(1),  params.hlambda_range(2));
theta_target.phi_b   = param2ismc(action(2), params.hphi_b_range(1),   params.hphi_b_range(2));
beta = 0.1;  % 0.05 ~ 0.2 可调 一阶平滑系数

params.K_d(1)      = (1-beta)*params.K_d(1)      + beta*theta_target.K_d;
params.K_sigma(1)  = (1-beta)*params.K_sigma(1)  + beta*theta_target.K_sigma;
params.lambda(1)   = (1-beta)*params.lambda(1)   + beta*theta_target.lambda;
params.phi_b(1)    = (1-beta)*params.phi_b(1)    + beta*theta_target.phi_b;

%% ===== 4. ISMC 控制 =====
delta_r_d = my_integralSMCheading(psi, r, psi_ref, r_d, loggedSignals.a_d, h, params);
delta_r = sat(delta_r_d, params.delta_max); % 垂直舵控制（航向轴）
%% ===== 5. 固定转速 =====
ui = [delta_r; 0; n];

%% ===== 6. 动力学更新 =====
x = rk4(@remus100, h, x, ui, params.Vc, params.betaVc, params.wc);
x(12) = ssa(x(12));

%% ===== 7. 更新状态 =====
u   = x(1);
r   = x(6);
psi = x(12);

e_psi = wrapToPi(ssa(psi - psi_ref));
e_r   = r - r_d;
sigma     = e_r + params.lambda(1)*e_psi;


%% ===== 9. 奖励函数 =====
% 目标：
% 1) 远离目标时快速收敛
% 2) 接近目标时减小角速度、防止超调和振荡
% 3) 避免过大舵角和过快参数漂移

t = loggedSignals.stepCount * h;

theta_new = [params.K_d(1); params.K_sigma(1); params.lambda(1); params.phi_b(1)];
% theta_new = [params.K_sigma(1); params.phi_b(1)];
dtheta = theta_new - theta_old;

delta_r_rate = delta_r - loggedSignals.delta_r_prev;

abs_epsi = abs(e_psi);
abs_r = abs(r);

% ---------- 基础项 ----------
reward = 0;

% 每步生存代价：鼓励更快完成
reward = reward - 0.03;

% 参数变化惩罚：防止参数乱漂
reward = reward - 0.05 * sum(dtheta.^2);

% 控制能量和控制变化率惩罚
reward = reward - 0.015 * delta_r^2;
reward = reward - 0.06 * delta_r_rate^2;

% ---------- 分阶段奖励 ----------
if abs_epsi > deg2rad(8)
    % 远离目标：强调尽快拉回来
    reward = reward ...
        - 14.0 * e_psi^2 ...
        - 0.8  * r^2 ...
        - 0.2  * sigma^2 ...
        - 0.20 * t * abs_epsi;   % 时间加权误差，更强地区分快慢
elseif abs_epsi > deg2rad(3)
    % 中等误差：兼顾收敛速度和角速度约束
    reward = reward ...
        - 8.0  * e_psi^2 ...
        - 2.0  * r^2 ...
        - 0.25 * sigma^2 ...
        - 0.08 * t * abs_epsi;
else
    % 接近目标：重点压制角速度和抖动
    reward = reward ...
        - 4.0  * e_psi^2 ...
        - 6.0  * r^2 ...
        - 0.30 * sigma^2 ...
        - 0.12 * delta_r_rate^2;
end

% ---------- 超调惩罚 ----------
% 若误差符号相对初始误差发生反转，说明穿越目标
if sign(e_psi) ~= sign(loggedSignals.epsi0) && abs_epsi > deg2rad(0.3)
    reward = reward - 25.0 * e_psi^2 - 4.0 * r^2;
end

% ---------- 接近目标时，若角速度仍然过大，则额外惩罚 ----------
if abs_epsi < deg2rad(3)
    reward = reward - 8.0 * r^2;
end

% ---------- 误差带奖励 ----------
% 进入较小误差范围，持续给奖励
if abs_epsi < deg2rad(1.5)
    reward = reward + 0.8;
end

if abs_epsi < deg2rad(1) && abs_r < deg2rad(0.8)
    reward = reward + 2.0;
end

if abs_epsi < deg2rad(0.5) && abs_r < deg2rad(0.25)
    reward = reward + 4.0;
end

% 如果误差比上一步减小，给微弱正奖励，帮助 critic 学习“朝正确方向前进”
if isfield(loggedSignals, 'e_psi_prev')
    reward = reward + 0.6 * (abs(loggedSignals.e_psi_prev) - abs_epsi);
end

% 同时体现：误差小，收敛快，接近目标时不能冲太猛，不能超调，不能乱调参数
%% ===== 10. 下一观测 =====
% 观测可以包含误差、动态状态、以及上一时刻参数
nextObs = [
    e_psi
    e_r
    sigma
    psi
    r
    u
    params.K_d(1)
    params.K_sigma(1)
    params.lambda(1)
    params.phi_b(1)
    ];

%% ===== 11. 终止条件 =====
loggedSignals.stepCount = loggedSignals.stepCount + 1;
if ~isfield(loggedSignals, 'smallErrCount')
    loggedSignals.smallErrCount = 0;
end
if abs(e_psi) < deg2rad(1) && abs(r) < deg2rad(0.8)
    loggedSignals.smallErrCount = loggedSignals.smallErrCount + 1;
else
    loggedSignals.smallErrCount = 0;
end

if loggedSignals.smallErrCount > 100 % 1s保持
    isDone = true;
    reward = reward + 50 + 0.06 * (loggedSignals.maxSteps - loggedSignals.stepCount);
    loggedSignals.doneReason = "Finished";
end

% 失败终止：仅在尚未成功终止时检查
% if ~isDone && abs(r) > params.r_max
%     isDone = true;
%     reward = reward - 50;
%     loggedSignals.doneReason = "RateLimit";
% end

if ~isDone && abs(e_psi) > deg2rad(80)
    isDone = true;
    reward = reward - 100;
    loggedSignals.doneReason = "LargeHeadingError";
end

if ~isDone && (any(isnan(x)) || any(isinf(x)))
    isDone = true;
    reward = reward - 120;
    loggedSignals.doneReason = "InvalidState";
end

if ~isDone && loggedSignals.stepCount >= loggedSignals.maxSteps
    isDone = true;
    
    % 情况 A：超时但已经非常接近成功
    if abs(e_psi) < deg2rad(1) && abs(r) < deg2rad(0.5)
        % 说明就差持续保持时间不够，不应重罚
        reward = reward + 15;
        loggedSignals.doneReason = "MaxSteps_NearSuccess";
        
        % 情况 B：超时但已接近目标
    elseif abs(e_psi) < deg2rad(3.0) && abs(r) < deg2rad(1.5)
        reward = reward - 5;
        loggedSignals.doneReason = "MaxSteps_Close";
        
        % 情况 C：超时且中等偏差
    elseif abs(e_psi) < deg2rad(6.0)
        reward = reward - 20;
        loggedSignals.doneReason = "MaxSteps_MediumError";
        
        % 情况 D：超时且明显没做好
    else
        reward = reward - 50;
        loggedSignals.doneReason = "MaxSteps_Fail";
    end
end

if isDone
    fprintf(['Episode End | steps=%d | reason=%s | ' ...
        'Kd=%.4f | Ksigma=%.4f | lambda=%.4f | phi_b=%.4f | ' ...
        'epsi=%.3f deg | reward=%.3f | ui=%d \n'], ...
        loggedSignals.stepCount, ...
        loggedSignals.doneReason, ...
        params.K_d(1), ...
        params.K_sigma(1), ...
        params.lambda(1), ...
        params.phi_b(1), ...
        rad2deg(e_psi), ...
        reward, ...
        rad2deg(delta_r));
end

%% ===== 12. 回写 =====
loggedSignals.x = x;
loggedSignals.params = params;
loggedSignals.psi_ref = psi_ref;
% loggedSignals.psi_d = psi_d;
loggedSignals.r_d = r_d;
loggedSignals.ui = ui;
loggedSignals.delta_r_prev = delta_r;
loggedSignals.e_psi_prev   = e_psi;

end
