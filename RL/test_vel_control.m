clc; clear; close all;

%% ===== 加载训练好的智能体 =====
load("TD3agent_vel_260316.mat","agent");   % 按你的文件名改

%% ===== 环境初始化 =====
[obs, loggedSignals] = reset_vel_control();

% 如果你想固定测试条件，可以在这里覆盖 reset 里的默认值
% 例如：
% loggedSignals.u_d = 1.6;      % 目标速度
% loggedSignals.u = 1.2;        % 初始速度
% loggedSignals.udot = 0.0;
% loggedSignals.n_prev = 900;
%
% obs = [
%     loggedSignals.u_d - loggedSignals.u
%     0 - loggedSignals.udot
%     loggedSignals.u
%     loggedSignals.udot
%     loggedSignals.n_prev
%     loggedSignals.params.Kp
%     loggedSignals.params.Ki
%     loggedSignals.params.Kd
% ];

%% ===== 仿真步数 =====
N = loggedSignals.maxSteps;
t = (0:N-1)' * loggedSignals.h;

%% ===== 预分配存储 =====
obsHist    = zeros(N, length(obs));
actHist    = zeros(N, 3);
rewardHist = zeros(N, 1);
doneHist   = false(N, 1);

uHist      = zeros(N,1);
uRefHist   = zeros(N,1);
udotHist   = zeros(N,1);
nHist      = zeros(N,1);

kpHist     = zeros(N,1);
kiHist     = zeros(N,1);
kdHist     = zeros(N,1);

euHist     = zeros(N,1);
eudotHist  = zeros(N,1);

%% ===== 主循环 =====
for k = 1:N
    obsHist(k,:) = obs(:)';
    
    % --- 获取智能体动作 ---
    action = localGetAction(agent, obs);
    actHist(k,:) = action(:)';
    
    % --- 环境步进 ---
    [nextObs, reward, isDone, loggedSignals] = step_vel_control(action, loggedSignals);
    
    rewardHist(k) = reward;
    doneHist(k)   = isDone;
    
    % --- 记录数据 ---
    uHist(k)     = loggedSignals.x(1);
    uRefHist(k)  = loggedSignals.u_d;
    udotHist(k)  = loggedSignals.udot_prev;
    nHist(k)     = loggedSignals.n_prev;
    
    kpHist(k)    = loggedSignals.params.Kp;
    kiHist(k)    = loggedSignals.params.Ki;
    kdHist(k)    = loggedSignals.params.Kd;
    
    euHist(k)    = loggedSignals.u_d - loggedSignals.x(1);
    eudotHist(k) = 0 - loggedSignals.udot_prev;
    
    % 更新观测
    obs = nextObs;
    
    if isDone
        fprintf('Test finished at step %d\n', k);
        break;
    end
end

%% ===== 截断有效数据 =====
idx = 1:k;
t = t(idx);

uHist      = uHist(idx);
uRefHist   = uRefHist(idx);
udotHist   = udotHist(idx);
nHist      = nHist(idx);

kpHist     = kpHist(idx);
kiHist     = kiHist(idx);
kdHist     = kdHist(idx);

euHist     = euHist(idx);
eudotHist  = eudotHist(idx);

rewardHist = rewardHist(idx);
actHist    = actHist(idx,:);

%% ===== 性能指标 =====
absErr = abs(euHist);
MAE    = mean(absErr);
RMSE   = sqrt(mean(euHist.^2));
MaxErr = max(absErr);
FinalErr = euHist(end);

fprintf('\n===== Velocity Tracking Test Result =====\n');
fprintf('Steps      : %d\n', k);
fprintf('MAE        : %.6f m/s\n', MAE);
fprintf('RMSE       : %.6f m/s\n', RMSE);
fprintf('Max |e|    : %.6f m/s\n', MaxErr);
fprintf('Final e_u  : %.6f m/s\n', FinalErr);
fprintf('Final u    : %.6f m/s\n', uHist(end));
fprintf('Final u_ref: %.6f m/s\n', uRefHist(end));
fprintf('Final n    : %.6f rpm\n', nHist(end));
fprintf('Final Kp   : %.6f\n', kpHist(end));
fprintf('Final Ki   : %.6f\n', kiHist(end));
fprintf('Final Kd   : %.6f\n', kdHist(end));
fprintf('TotalReward: %.6f\n', sum(rewardHist));

%% ===== 画图 =====
figure('Name','Velocity Tracking Test','Color','w');

subplot(4,1,1);
plot(t, uRefHist, 'r--', 'LineWidth', 1.8); hold on;
plot(t, uHist, 'b', 'LineWidth', 1.8);
grid on;
ylabel('u (m/s)');
legend('u_{ref}','u','Location','best');
title('Surge Velocity Tracking');

subplot(4,1,2);
plot(t, euHist, 'k', 'LineWidth', 1.5); hold on;
yline(0,'r--');
grid on;
ylabel('e_u (m/s)');
title(sprintf('Tracking Error | MAE=%.4f, RMSE=%.4f', MAE, RMSE));

subplot(4,1,3);
plot(t, nHist, 'm', 'LineWidth', 1.5);
grid on;
ylabel('n (rpm)');
title('Thruster Command');

subplot(4,1,4);
plot(t, kpHist, 'LineWidth', 1.4); hold on;
plot(t, kiHist, 'LineWidth', 1.4);
plot(t, kdHist, 'LineWidth', 1.4);
grid on;
xlabel('Time (s)');
ylabel('Gain');
legend('Kp','Ki','Kd','Location','best');
title('Online Tuned PID Gains');

figure('Name','Reward and Acceleration','Color','w');

subplot(2,1,1);
plot(t, rewardHist, 'LineWidth', 1.2);
grid on;
ylabel('Reward');
title('Reward per Step');

subplot(2,1,2);
plot(t, udotHist, 'LineWidth', 1.2); hold on;
plot(t, eudotHist, '--', 'LineWidth', 1.2);
grid on;
xlabel('Time (s)');
ylabel('Acceleration');
legend('udot','e_{udot}','Location','best');
title('Acceleration Response');

%% ===== 动作输出范围检查 =====
figure('Name','Agent Action','Color','w');
plot(t, actHist(:,1), 'LineWidth', 1.2); hold on;
plot(t, actHist(:,2), 'LineWidth', 1.2);
plot(t, actHist(:,3), 'LineWidth', 1.2);
grid on;
xlabel('Time (s)');
ylabel('Normalized Action');
legend('a_1','a_2','a_3','Location','best');
title('TD3 Actor Output');

%% ===== 本地函数：兼容不同 getAction 返回格式 =====
function action = localGetAction(agent, obs)

try
    % 常见格式1
    action = getAction(agent, {obs});
    if iscell(action)
        action = action{1};
    end
    return;
catch
end

try
    % 常见格式2
    action = getAction(agent, obs);
    if iscell(action)
        action = action{1};
    end
    return;
catch
end

error('Unable to get action from agent. Please check observation format.');
end
