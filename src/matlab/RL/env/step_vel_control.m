function [nextObs, reward, isDone, loggedSignals] = step_vel_control(action, loggedSignals)
% step_vel_control
% controller: my_PIDsurge
% Method: TD3 / DDPG / SAC etc.
%
% 说明：
% 1) 用强化学习在线整定 surge 速度PID参数
% 2) 控制量为主推进器转速 n
% 3) 假设舵角固定为 0，即 ui = [delta_r; delta_s; n] = [0; 0; n_cmd]
%
% 依赖:
%   my_PIDsurge.m
%   rk4.m
%   remus100.m
%   sat.m
%   param2ismc.m   % 用于把动作[-1,1]映射到参数范围
%
% 注意:
% 本代码默认 params 中含有以下字段:
%   params.Kp
%   params.Ki
%   params.Kd
%   params.Ka
%   params.int_sep_th
%
%   params.kp_range = [Kp_min, Kp_max]
%   params.ki_range = [Ki_min, Ki_max]
%   params.kd_range = [Kd_min, Kd_max]
%   params.ka_range = [Ka_min, Ka_max]
%   params.int_sep_th_range = [th_min, th_max]
%
%   params.n_min
%   params.n_max
%
% 如果你的工程里参数名不是这些，请对应替换。

if iscell(action)
    action = action{1};
end

% 动作限幅
action = max(min(action(:), 1), -1);

isDone = false;
loggedSignals.doneReason = "";

%% ===== 1. 当前状态 =====
x      = loggedSignals.x;
params = loggedSignals.params;
h      = loggedSignals.h;

u      = x(1);   % 当前surge速度
n      = loggedSignals.ui(3);

% 当前加速度估计
if isfield(loggedSignals, 'udot_prev')
    udot = loggedSignals.udot_prev;
else
    udot = 0;
end

%% ===== 2. 参考速度 =====
u_d    = loggedSignals.u_d;
u_d_dot = loggedSignals.u_d_dot;

%% ===== 3. 参数更新 =====
% 记录更新前参数
theta_old = [
    params.Kp
    params.Ki
    params.Kd
    ];

% 目标参数（动作映射）
theta_target.Kp = param2ismc(action(1), params.kp_range(1), params.kp_range(2));
theta_target.Ki = param2ismc(action(2), params.ki_range(1), params.ki_range(2));
theta_target.Kd = param2ismc(action(3), params.kd_range(1), params.kd_range(2));


% 一阶平滑，避免参数突变
beta = 0.1;

params.Kp = (1 - beta) * params.Kp + beta * theta_target.Kp;
params.Ki = (1 - beta) * params.Ki + beta * theta_target.Ki;
params.Kd = (1 - beta) * params.Kd + beta * theta_target.Kd;

%% ===== 4. PID 速度控制 =====
[n_d, term] = my_PIDsurge(u_d, u_d_dot, u, udot, params);
if n_d > 0
    n = n + min(n_d,params.n_rate);
else
    n = n - min(abs(n_d),params.n_rate);
end
% if n < n_d
%     n = n + params.n_rate;
% elseif n > n_d
%     n = n - params.n_rate;
% end
n = sat(n, params.n_max);

%% ===== 5. 控制输入 =====
% 这里只控制主推，舵角固定
ui = [0; 0; n];

%% ===== 6. 动力学更新 =====
x_old = x;
x = rk4(@remus100, h, x, ui, params.Vc, params.betaVc, params.wc);

% 取更新后的速度
u_new = x(1);

% 数值微分估计当前加速度
udot_new = (u_new - x_old(1)) / h;

%% ===== 7. 更新误差 =====
e_u    = u_d - u_new;
e_udot = u_d_dot - udot_new;

abs_eu    = abs(e_u);
abs_udot  = abs(udot_new);

% 参数变化量
theta_new = [
    params.Kp
    params.Ki
    params.Kd
    ];
dtheta = theta_new - theta_old;

%% ===== 8. 奖励函数 =====
% 设计目标：
% 1) 远离目标时尽快收敛
% 2) 接近目标时减小加速度和推力变化，避免振荡
% 3) 避免参数乱漂
% 4) 避免推进器转速过大和变化率过快

t = loggedSignals.stepCount * h;
reward = 0;

% ---------- 基础项 ----------
% 每步生存代价
reward = reward - 0.03;

% 参数漂移惩罚
reward = reward - 0.02 * sum(dtheta.^2);

% 控制能量惩罚（归一化）
% reward = reward - 0.02 * (n / max(params.n_max, 1))^2;

% 控制变化率惩罚（归一化）
% reward = reward - 0.04 * (n_rate / max(params.n_max, 1))^2;

% ---------- 分阶段奖励 ----------
if abs_eu > 0.30
    % 大误差阶段：强调快速拉回速度
    reward = reward ...
        - 12.0 * e_u^2 ...
        - 1.0  * udot_new^2 ...
        - 0.15 * t * abs_eu;
elseif abs_eu > 0.10
    % 中误差阶段：兼顾收敛和抑制动态波动
    reward = reward ...
        - 8.0  * e_u^2 ...
        - 2.5  * udot_new^2 ...
        - 0.08 * t * abs_eu;
else
    % 小误差阶段：重点抑制抖动和过冲
    reward = reward ...
        - 4.0  * e_u^2 ...
        - 5.0  * udot_new^2;
    %         - 0.08 * (n_rate / max(params.n_max, 1))^2;
end

% ---------- 超调惩罚 ----------
if isfield(loggedSignals, 'eu0')
    if sign(e_u) ~= sign(loggedSignals.eu0) && abs_eu > 0.03
        reward = reward - 18.0 * e_u^2 - 2.0 * udot_new^2;
    end
end

% ---------- 接近目标时，若加速度仍大，则额外惩罚 ----------
if abs_eu < 0.10
    reward = reward - 4.0 * udot_new^2;
end

% ---------- 小误差区间奖励 ----------
if abs_eu < 0.08
    reward = reward + 0.8;
end

if abs_eu < 0.05 && abs_udot < 0.05
    reward = reward + 2.0;
end

if abs_eu < 0.02 && abs_udot < 0.02
    reward = reward + 4.0;
end

% ---------- 单步误差改善奖励 ----------
if isfield(loggedSignals, 'e_u_prev')
    reward = reward + 0.8 * (abs(loggedSignals.e_u_prev) - abs_eu);
end

%% ===== 9. 下一观测 =====

nextObs = [
    e_u
    e_udot
    u_new
    udot_new
    n
    params.Kp(1)
    params.Ki(1)
    params.Kd(1)
    ];

%% ===== 10. 终止条件 =====
loggedSignals.stepCount = loggedSignals.stepCount + 1;

if ~isfield(loggedSignals, 'smallErrCount')
    loggedSignals.smallErrCount = 0;
end

% 成功判据：速度误差和加速度都足够小并持续保持
if abs_eu < 0.03 && abs_udot < 0.03
    loggedSignals.smallErrCount = loggedSignals.smallErrCount + 1;
else
    loggedSignals.smallErrCount = 0;
end

if loggedSignals.smallErrCount > 100
    isDone = true;
    reward = reward + 100 + 0.06 * (loggedSignals.maxSteps - loggedSignals.stepCount);
    loggedSignals.doneReason = "Finished";
end

% 失败终止：误差过大
if ~isDone && abs_eu > 1.5
    isDone = true;
    reward = reward - 100;
    loggedSignals.doneReason = "LargeSpeedError";
end

% 失败终止：速度明显异常
if ~isDone && (u_new < -0.5 || u_new > 3.5)
    isDone = true;
    reward = reward - 120;
    loggedSignals.doneReason = "SpeedOutOfRange";
end

% 失败终止：状态非法
if ~isDone && (any(isnan(x)) || any(isinf(x)))
    isDone = true;
    reward = reward - 120;
    loggedSignals.doneReason = "InvalidState";
end

% 超时终止
if ~isDone && loggedSignals.stepCount >= loggedSignals.maxSteps
    isDone = true;
    
    if abs_eu < 0.03 && abs_udot < 0.03
        reward = reward + 15;
        loggedSignals.doneReason = "MaxSteps_NearSuccess";
    elseif abs_eu < 0.08 && abs_udot < 0.05
        reward = reward - 5;
        loggedSignals.doneReason = "MaxSteps_Close";
    elseif abs_eu < 0.20
        reward = reward - 20;
        loggedSignals.doneReason = "MaxSteps_MediumError";
    else
        reward = reward - 50;
        loggedSignals.doneReason = "MaxSteps_Fail";
    end
end

%% ===== 11. 结束打印 =====
if isDone
    fprintf(['Episode End | steps=%d | reason=%s | ' ...
        'Kp=%.4f | Ki=%.4f | Kd=%.4f | ' ...
        'eu=%.4f | u=%.4f | udot=%.4f | reward=%.3f | n=%.2f\n'], ...
        loggedSignals.stepCount, ...
        loggedSignals.doneReason, ...
        params.Kp, ...
        params.Ki, ...
        params.Kd, ...
        e_u, ...
        u_new, ...
        udot_new, ...
        reward, ...
        n);
end

%% ===== 12. 回写 =====
loggedSignals.x = x;
loggedSignals.params = params;
loggedSignals.u_d = u_d;
loggedSignals.u_d_dot = u_d_dot;
loggedSignals.ui = ui;

loggedSignals.n_prev = n;
loggedSignals.u_prev = u_new;
loggedSignals.udot_prev = udot_new;
loggedSignals.e_u_prev = e_u;

% 记录调试量
loggedSignals.term = term;

end
