%% 测试 TD3 航向控制训练结果并评估
clc; clear; close all;

%% -------------------- 加载训练好的 TD3 agent --------------------
load('TD3agent260316.mat', 'agent');   % 按你的实际文件名修改

%% -------------------- 初始化环境 --------------------
[obs, logs] = reset_heading_control();

%% -------------------- 测试设置 --------------------
T = 10;
h = logs.h;
N = round(T / h);

% 误差容限
tol_deg = 1;             % 收敛误差带（度）
tol_rad = deg2rad(tol_deg);

% 如果环境中有舵角上限，可读出来；否则默认 20 度
if isfield(logs, 'delta_max')
    delta_max = logs.delta_max;
else
    delta_max = deg2rad(20);
end

% 参数范围（用于统计参数是否撞边界）
paramRange.K_d     = logs.params.hkd_range;
paramRange.K_sigma = logs.params.hksigma_range;
paramRange.lambda  = logs.params.hlambda_range;
paramRange.phi_b   = logs.params.hphi_b_range;


%% -------------------- 数据存储 --------------------
hist.obs      = zeros(N, numel(obs));
hist.reward   = zeros(N,1);

hist.epsi     = zeros(N,1);   % heading error
hist.er       = zeros(N,1);   % yaw-rate error
hist.sigma    = zeros(N,1);   % sliding surface
hist.psi      = zeros(N,1);   % actual heading
hist.psi_ref  = zeros(N,1);   % reference heading
hist.r        = zeros(N,1);   % yaw rate
hist.u        = zeros(N,1);   % surge speed

hist.K_d      = zeros(N,1);
hist.K_sigma  = zeros(N,1);
hist.lambda   = zeros(N,1);
hist.phi_b    = zeros(N,1);

hist.ui       = zeros(N,3);   % [delta_r, delta_s, n]
hist.x        = zeros(N,12);  % full state
hist.action   = zeros(N,2);   % RL action, assuming action dim = 4 % 改为2维

doneStep = N;
doneFlag = false;

%% ==================== 5) 开始测试 ====================
for k = 1:N
    
    % -------- agent 动作 --------
    action = getAction(agent, obs);
    if iscell(action)
        action = action{1};
    end
    action = double(action(:));
    
    if numel(action) ~= 2
        error('Expected action dimension = 4, but got %d', numel(action));
    end
    
    % -------- 环境推进 --------
    [nextObs, reward, done, logs] = step_heading_control(action, logs);
    
    % -------- 数据记录 --------
    hist.obs(k,:)     = nextObs(:)';
    hist.reward(k)    = reward;
    hist.action(k,:)  = action(:)';
    
    % 按你当前观测定义：
    % nextObs = [epsi, er, sigma, psi, r, u, K_d, K_sigma, lambda, phi_b]
    hist.epsi(k)      = nextObs(1);
    hist.er(k)        = nextObs(2);
    hist.sigma(k)     = nextObs(3);
    hist.psi(k)       = nextObs(4);
    hist.r(k)         = nextObs(5);
    hist.u(k)         = nextObs(6);
    
    % hist.K_d(k)       = nextObs(7);
    hist.K_d(k)       = 0.005;
    hist.K_sigma(k)   = nextObs(7);
    % hist.lambda(k)    = 0.1;
    hist.phi_b(k)     = nextObs(8);
    
    % 参考航向
    if isfield(logs, 'psi_ref')
        hist.psi_ref(k) = logs.psi_ref;
    else
        hist.psi_ref(k) = NaN;
    end
    
    % 控制输入
    if isfield(logs, 'ui')
        hist.ui(k,:) = logs.ui(:)';
    else
        hist.ui(k,:) = [NaN NaN NaN];
    end
    
    % 状态
    if isfield(logs, 'x')
        x_now = logs.x(:)';
        nx = min(numel(x_now), size(hist.x,2));
        hist.x(k,1:nx) = x_now(1:nx);
    end
    
    obs = nextObs;
    
    if done
        doneStep = k;
        doneFlag = true;
        fprintf('Episode finished at step %d, t = %.2f s\n', k, k*h);
        break;
    end
end

%% -------------------- 截取有效数据 --------------------
k = doneStep;
t = (0:k-1)' * h;

fields = fieldnames(hist);
for i = 1:numel(fields)
    data = hist.(fields{i});
    if size(data,1) == N
        hist.(fields{i}) = data(1:k,:);
    end
end

%% ==================== 7) 单位转换 ====================
epsi_deg      = rad2deg(hist.epsi);
er_deg_s      = rad2deg(hist.er);
r_deg_s       = rad2deg(hist.r);
psi_deg       = rad2deg(hist.psi);
psi_ref_deg   = rad2deg(hist.psi_ref);
delta_r_deg   = rad2deg(hist.ui(:,1));

%% ==================== 8) 性能指标评估 ====================
metrics = struct();

% ---- 奖励指标 ----
metrics.TotalReward   = sum(hist.reward, 'omitnan');
metrics.MeanReward    = mean(hist.reward, 'omitnan');
metrics.MinReward     = min(hist.reward, [], 'omitnan');
metrics.MaxReward     = max(hist.reward, [], 'omitnan');
metrics.RewardStd     = std(hist.reward, 'omitnan');

% ---- 航向误差指标 ----
metrics.HeadingError_MAE_deg   = mean(abs(epsi_deg), 'omitnan');
metrics.HeadingError_RMSE_deg  = sqrt(mean(epsi_deg.^2, 'omitnan'));
metrics.HeadingError_MAX_deg   = max(abs(epsi_deg), [], 'omitnan');

% IAE / ITAE
metrics.HeadingError_IAE_deg_s  = sum(abs(epsi_deg), 'omitnan') * h;
metrics.HeadingError_ITAE_deg_s = sum(t .* abs(epsi_deg), 'omitnan') * h;

% ---- 偏航角速度指标 ----
metrics.YawRate_MAE_deg_s      = mean(abs(r_deg_s), 'omitnan');
metrics.YawRate_RMSE_deg_s     = sqrt(mean(r_deg_s.^2, 'omitnan'));
metrics.YawRate_MAX_deg_s      = max(abs(r_deg_s), [], 'omitnan');

% ---- 滑模面指标 ----
metrics.Sigma_MAE              = mean(abs(hist.sigma), 'omitnan');
metrics.Sigma_RMSE             = sqrt(mean(hist.sigma.^2, 'omitnan'));
metrics.Sigma_MAX              = max(abs(hist.sigma), [], 'omitnan');

% ---- 速度指标 ----
metrics.Speed_Mean_m_s         = mean(hist.u, 'omitnan');
metrics.Speed_RMSE_m_s         = sqrt(mean(hist.u.^2, 'omitnan'));
metrics.Speed_MIN_m_s          = min(hist.u, [], 'omitnan');
metrics.Speed_MAX_m_s          = max(hist.u, [], 'omitnan');

% ---- 舵角指标 ----
metrics.Rudder_MAE_deg         = mean(abs(delta_r_deg), 'omitnan');
metrics.Rudder_RMSE_deg        = sqrt(mean(delta_r_deg.^2, 'omitnan'));
metrics.Rudder_MAX_deg         = max(abs(delta_r_deg), [], 'omitnan');

% 舵角饱和比例
rudderSatMask = abs(hist.ui(:,1)) >= 0.98 * delta_max;
metrics.RudderSaturation_Ratio = mean(rudderSatMask, 'omitnan');

% 控制变化率（可反映抖动）
if k >= 2
    ddelta = diff(delta_r_deg) / h;
    metrics.RudderRate_MAE_deg_s  = mean(abs(ddelta), 'omitnan');
    metrics.RudderRate_RMSE_deg_s = sqrt(mean(ddelta.^2, 'omitnan'));
    metrics.RudderRate_MAX_deg_s  = max(abs(ddelta), [], 'omitnan');
else
    metrics.RudderRate_MAE_deg_s  = NaN;
    metrics.RudderRate_RMSE_deg_s = NaN;
    metrics.RudderRate_MAX_deg_s  = NaN;
end

% ---- 稳态误差：最后10%时间段 ----
tailIdx = max(1, round(0.9*k)) : k;
metrics.SteadyHeadingError_MAE_deg   = mean(abs(epsi_deg(tailIdx)), 'omitnan');
metrics.SteadyHeadingError_RMSE_deg  = sqrt(mean(epsi_deg(tailIdx).^2, 'omitnan'));
metrics.SteadyHeadingError_MAX_deg   = max(abs(epsi_deg(tailIdx)), [], 'omitnan');

% ---- 到达时间 / 稳定时间 ----
metrics.FirstEntryTime_2deg_s = NaN;
firstEntryIdx = find(abs(hist.epsi) <= tol_rad, 1, 'first');
if ~isempty(firstEntryIdx)
    metrics.FirstEntryTime_2deg_s = (firstEntryIdx - 1) * h;
end

metrics.SettlingTime_2deg_s = NaN;
for i = 1:k
    if all(abs(hist.epsi(i:end)) <= tol_rad)
        metrics.SettlingTime_2deg_s = (i - 1) * h;
        break;
    end
end

% ---- 超调量 ----
% 对于阶跃跟踪，超调可理解为越过参考值后的最大偏差
% 这里依据初始误差方向判断
if all(~isnan(hist.psi_ref))
    e0 = wrapToPi(hist.psi(1) - hist.psi_ref(1));
    if e0 > 0
        % 初始在参考右侧，需要往下收敛，超调表示越到参考另一侧
        overshootRad = max(0, max(hist.psi_ref - hist.psi, [], 'omitnan'));
    else
        % 初始在参考左侧，需要往上收敛
        overshootRad = max(0, max(hist.psi - hist.psi_ref, [], 'omitnan'));
    end
    metrics.Overshoot_deg = rad2deg(overshootRad);
else
    metrics.Overshoot_deg = NaN;
end

% ---- 参数统计 ----
metrics.Kd_Mean       = mean(hist.K_d, 'omitnan');
metrics.Ksigma_Mean   = mean(hist.K_sigma, 'omitnan');
metrics.Lambda_Mean   = mean(hist.lambda, 'omitnan');
metrics.Phib_Mean     = mean(hist.phi_b, 'omitnan');

metrics.Kd_Std        = std(hist.K_d, 'omitnan');
metrics.Ksigma_Std    = std(hist.K_sigma, 'omitnan');
metrics.Lambda_Std    = std(hist.lambda, 'omitnan');
metrics.Phib_Std      = std(hist.phi_b, 'omitnan');

% 参数撞边界比例（2%边界带）
edgeBand = 0.02;
metrics.Kd_BoundaryRatio = boundaryRatio(hist.K_d, paramRange.K_d, edgeBand);
metrics.Ksigma_BoundaryRatio = boundaryRatio(hist.K_sigma, paramRange.K_sigma, edgeBand);
metrics.Lambda_BoundaryRatio = boundaryRatio(hist.lambda, paramRange.lambda, edgeBand);
metrics.Phib_BoundaryRatio = boundaryRatio(hist.phi_b, paramRange.phi_b, edgeBand);

% ---- 成功判据 ----
% 可根据你的任务调整
success1 = metrics.SteadyHeadingError_MAE_deg <= 2;
success2 = ~isnan(metrics.SettlingTime_2deg_s);
success3 = metrics.RudderSaturation_Ratio <= 0.5;
metrics.Success = success1 && success2 && success3;

% ---- 其他信息 ----
metrics.FinalTime_s      = t(end);
metrics.TotalSteps       = k;
metrics.DoneTriggered    = doneFlag;

%% ==================== 9) 打印结果 ====================
disp('================ TD3 航向控制测试评估结果 ================');
disp(metrics);

fprintf('\n================ 关键结论 ================\n');
fprintf('总回报: %.3f\n', metrics.TotalReward);
fprintf('平均回报: %.3f\n', metrics.MeanReward);
fprintf('航向误差 MAE: %.3f deg\n', metrics.HeadingError_MAE_deg);
fprintf('航向误差 RMSE: %.3f deg\n', metrics.HeadingError_RMSE_deg);
fprintf('稳态误差 MAE: %.3f deg\n', metrics.SteadyHeadingError_MAE_deg);
fprintf('超调量: %.3f deg\n', metrics.Overshoot_deg);
fprintf('首次进入2deg误差带时间: %.3f s\n', metrics.FirstEntryTime_2deg_s);
fprintf('2deg稳定时间: %.3f s\n', metrics.SettlingTime_2deg_s);
fprintf('舵角饱和比例: %.3f\n', metrics.RudderSaturation_Ratio);
fprintf('测试是否成功: %d\n', metrics.Success);

%% ==================== 10) 绘图 ====================
figure('Name','Heading Control Performance','Color','w');

subplot(4,1,1);
plot(t, psi_ref_deg, 'r--', 'LineWidth', 1.5); hold on;
plot(t, psi_deg, 'b', 'LineWidth', 1.5);
ylabel('\psi (deg)');
legend('ref','actual');
title('Heading Tracking');
grid on;

subplot(4,1,2);
plot(t, epsi_deg, 'k', 'LineWidth', 1.2); hold on;
yline(tol_deg, 'r--'); yline(-tol_deg, 'r--');
ylabel('e_\psi (deg)');
title('Heading Error');
grid on;

subplot(4,1,3);
plot(t, r_deg_s, 'g', 'LineWidth', 1.2);
ylabel('r (deg/s)');
title('Yaw Rate');
grid on;

subplot(4,1,4);
plot(t, hist.reward, 'm', 'LineWidth', 1.2);
ylabel('Reward');
xlabel('Time (s)');
title('Reward');
grid on;

figure('Name','Sliding Surface and Speed','Color','w');

subplot(2,1,1);
plot(t, hist.sigma, 'b', 'LineWidth', 1.2);
ylabel('\sigma');
title('Sliding Surface');
grid on;

subplot(2,1,2);
plot(t, hist.u, 'r', 'LineWidth', 1.2);
ylabel('u (m/s)');
xlabel('Time (s)');
title('Surge Speed');
grid on;

figure('Name','Control Input and RL-tuned Parameters','Color','w');

subplot(5,1,1);
plot(t, delta_r_deg, 'b', 'LineWidth', 1.2); hold on;
yline(rad2deg(delta_max), 'r--');
yline(-rad2deg(delta_max), 'r--');
ylabel('\delta_r (deg)');
title('Rudder Command');
grid on;

subplot(5,1,2);
plot(t, hist.K_d, 'LineWidth', 1.2); hold on;
yline(paramRange.K_d(1), 'r--'); yline(paramRange.K_d(2), 'r--');
ylabel('K_d');
title('Adaptive Parameter K_d');
grid on;

subplot(5,1,3);
plot(t, hist.K_sigma, 'LineWidth', 1.2); hold on;
yline(paramRange.K_sigma(1), 'r--'); yline(paramRange.K_sigma(2), 'r--');
ylabel('K_\sigma');
title('Adaptive Parameter K_\sigma');
grid on;

subplot(5,1,4);
plot(t, hist.lambda, 'LineWidth', 1.2); hold on;
yline(paramRange.lambda(1), 'r--'); yline(paramRange.lambda(2), 'r--');
ylabel('\lambda');
title('Adaptive Parameter \lambda');
grid on;

subplot(5,1,5);
plot(t, hist.phi_b, 'LineWidth', 1.2); hold on;
yline(paramRange.phi_b(1), 'r--'); yline(paramRange.phi_b(2), 'r--');
ylabel('\phi_b');
xlabel('Time (s)');
title('Adaptive Parameter \phi_b');
grid on;

%% ==================== 11) 保存结果 ====================
result.metrics = metrics;
result.hist = hist;
result.t = t;
save('TD3_heading_test_result.mat', 'result');

disp('测试结果已保存到 TD3_heading_test_result.mat');

%% ==================== 局部函数 ====================
function ratio = boundaryRatio(x, range, edgeBand)
% 统计参数处于边界附近的比例
% edgeBand 例如 0.02 表示 2%

xmin = range(1);
xmax = range(2);
width = xmax - xmin;

lowBand  = xmin + edgeBand * width;
highBand = xmax - edgeBand * width;

mask = (x <= lowBand) | (x >= highBand);
ratio = mean(mask, 'omitnan');
end