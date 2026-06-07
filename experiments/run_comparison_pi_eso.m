%% 物理信息ESO-SMC对比实验
% Idea 1: Physics-Informed ESO-SMC for Spatially Correlated Ocean Currents
%
% 对比三种方法 × 4种海流场景：
%   方法0: 纯SMC（无ESO）
%   方法1: 经典LESO+SMC
%   方法2: 物理信息ESO+SMC（PI-ESO，Gauss-Markov模型）
%
% 场景1: 均匀恒定流
% 场景2: Gauss-Markov缓变流
% 场景3: 深度剪切流
% 场景4: 空间相关流场
%
% 指标: 跟踪RMSE、扰动估计误差、控制能量、抖振指数

clear; close all; clc;
project_root = setup_paths();

fprintf('=== 物理信息ESO-SMC对比实验 ===\n');
fprintf('方法: SMC | LESO+SMC | PI-ESO+SMC\n');
fprintf('场景: 恒定流 | Gauss-Markov流 | 深度剪切流 | 空间相关流\n\n');

%% 参数
params = get_params;
params.pieso = params.eso;
params.pieso.tau_c = 50;  % PI-ESO的Gauss-Markov时间常数

n_methods   = 3;
n_scenarios = 2;  % 跑场景1（恒定流）和场景2（GM时变流）
method_names   = {'纯SMC', 'LESO+SMC', 'PI-ESO+SMC'};
scenario_names = {'恒定流', 'GM缓变流', '深度剪切流', '空间相关流'};

%% 运行仿真
results = cell(n_methods, n_scenarios);

for s = 1:n_scenarios
    fprintf('场景 %d/%d: %s\n', s, n_scenarios, scenario_names{s});
    for m = 0:n_methods-1
        fprintf('  方法 %d: %s ... ', m, method_names{m+1});
        % 每次仿真前清除persistent变量
        clear smc_yaw_xhy smc_pitch_xhy smc_surge_xhy smc_heave_xhy
        clear my_ALOS3D vec_leso_update_adv vec_pieso_update
        tic;
        results{m+1, s} = xhy_pi_eso_simulator(m, s, params);
        fprintf('完成 (%.1fs)\n', toc);
    end
end

fprintf('\n所有仿真完成，开始计算指标...\n');

%% 计算性能指标
metrics = struct();
metrics.pos_rmse    = zeros(n_methods, n_scenarios);  % 位置跟踪RMSE (m)
metrics.heading_rmse = zeros(n_methods, n_scenarios); % 航向跟踪RMSE (rad)
metrics.ctrl_energy  = zeros(n_methods, n_scenarios); % 控制能量 (N²·s)
metrics.chattering   = zeros(n_methods, n_scenarios); % 抖振指数（控制方差）
metrics.est_error    = zeros(n_methods, n_scenarios); % 扰动估计误差（仅ESO方法）

for s = 1:n_scenarios
    for m = 1:n_methods
        hist = results{m, s};
        N    = length(hist.t);
        h    = hist.t(2) - hist.t(1);

        % 位置跟踪RMSE（与圆形轨迹的横向误差）
        pos_rmse = compute_path_rmse(hist);
        metrics.pos_rmse(m, s) = pos_rmse;

        % 航向跟踪RMSE
        psi_err = ssa_vec(hist.x(:,12) - hist.xd(:,1));
        metrics.heading_rmse(m, s) = sqrt(mean(psi_err.^2));

        % 控制能量（推力平方和积分）
        ctrl_energy = sum(sum(hist.ui.^2)) * h;
        metrics.ctrl_energy(m, s) = ctrl_energy;

        % 抖振指数（控制信号方差）
        chattering = mean(var(hist.tau));
        metrics.chattering(m, s) = chattering;

        % ESO Z3估计方差（仅ESO方法，反映估计稳定性）
        if m > 1
            z3_surge = hist.Z(:, 13);  % Z3 surge通道（加速度单位）
            metrics.est_error(m, s) = std(z3_surge);  % 标准差越小越稳定
        end
    end
end

%% 打印结果表格
fprintf('\n=== 性能指标汇总 ===\n\n');

fprintf('位置跟踪RMSE (m):\n');
fprintf('%-15s', '方法\场景');
for s = 1:n_scenarios, fprintf('%-15s', scenario_names{s}); end
fprintf('\n');
for m = 1:n_methods
    fprintf('%-15s', method_names{m});
    for s = 1:n_scenarios
        fprintf('%-15.4f', metrics.pos_rmse(m,s));
    end
    fprintf('\n');
end

fprintf('\n控制能量 (×10³ N²·s):\n');
fprintf('%-15s', '方法\场景');
for s = 1:n_scenarios, fprintf('%-15s', scenario_names{s}); end
fprintf('\n');
for m = 1:n_methods
    fprintf('%-15s', method_names{m});
    for s = 1:n_scenarios
        fprintf('%-15.2f', metrics.ctrl_energy(m,s)/1e3);
    end
    fprintf('\n');
end

fprintf('\nESO Z3估计标准差（越小越稳定，m/s²）:\n');
fprintf('%-15s', '方法\场景');
for s = 1:n_scenarios, fprintf('%-15s', scenario_names{s}); end
fprintf('\n');
for m = 2:n_methods
    fprintf('%-15s', method_names{m});
    for s = 1:n_scenarios
        fprintf('%-15.4f', metrics.est_error(m,s));
    end
    fprintf('\n');
end

%% 绘图
plot_pi_eso_comparison(results, metrics, method_names, scenario_names(1:n_scenarios));

%% 保存结果
if ~exist('results', 'dir'), mkdir('results'); end
save('results/pi_eso_comparison_results.mat', 'results', 'metrics', ...
     'method_names', 'scenario_names', 'params');
fprintf('\n结果已保存至 results/pi_eso_comparison_results.mat\n');

%% ===== 辅助函数 =====

function rmse = compute_path_rmse(hist)
% 计算与圆形参考轨迹的横向位置RMSE
pts  = hist.traj;
x_pos = hist.x(:,7);
y_pos = hist.x(:,8);

% 对每个时刻找最近轨迹点，计算横向误差
n = length(x_pos);
cross_err = zeros(n, 1);
n_pts = length(pts.pos.x);
for i = 1:n
    dx = pts.pos.x - x_pos(i);
    dy = pts.pos.y - y_pos(i);
    [~, idx] = min(dx.^2 + dy.^2);
    cross_err(i) = sqrt(dx(idx)^2 + dy(idx)^2);
end
rmse = sqrt(mean(cross_err.^2));
end

function psi_err = ssa_vec(psi_err)
% 向量化角度归一化到[-pi, pi]
psi_err = mod(psi_err + pi, 2*pi) - pi;
end
