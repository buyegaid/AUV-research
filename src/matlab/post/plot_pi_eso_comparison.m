function plot_pi_eso_comparison(results, metrics, method_names, scenario_names)
% PLOT_PI_ESO_COMPARISON 绘制物理信息ESO-SMC对比结果
%
% 图1: 4场景下的XY轨迹对比（3×4子图）
% 图2: 性能指标柱状图（4个指标）
% 图3: 场景2（Gauss-Markov流）的时域响应对比
% 图4: ESO扰动估计对比（场景2）

n_methods   = length(method_names);
n_scenarios = length(scenario_names);
colors = {[0.2 0.4 0.8], [0.8 0.4 0.1], [0.1 0.7 0.3]};  % 蓝/橙/绿
linestyles = {'-', '--', '-.'};

%% 图1: XY轨迹对比
n_cols = min(n_scenarios, 2);
n_rows = ceil(n_scenarios / n_cols);
fig1 = figure('Name', 'XY轨迹对比', 'Position', [50 50 700*n_cols 450*n_rows]);
for s = 1:n_scenarios
    subplot(n_rows, n_cols, s);
    hold on; grid on;
    pts = results{1,s}.traj;
    plot(pts.pos.y, pts.pos.x, 'k--', 'LineWidth', 1.5, 'DisplayName', '参考轨迹');
    for m = 1:n_methods
        hist = results{m, s};
        plot(hist.x(:,8), hist.x(:,7), ...
             'Color', colors{m}, 'LineStyle', linestyles{m}, ...
             'LineWidth', 1.2, 'DisplayName', method_names{m});
    end
    xlabel('Y (m)'); ylabel('X (m)');
    title(sprintf('场景%d: %s', s, scenario_names{s}));
    legend('Location', 'best', 'FontSize', 8);
    axis equal;
end
sgtitle('XY轨迹对比', 'FontSize', 12);

%% 图2: 性能指标柱状图
fig2 = figure('Name', '性能指标对比', 'Position', [50 50 1200 800]);

subplot(2,2,1);
bar(metrics.pos_rmse');
set(gca, 'XTickLabel', scenario_names);
ylabel('位置RMSE (m)');
title('轨迹跟踪精度');
legend(method_names, 'Location', 'northwest', 'FontSize', 8);
grid on;

subplot(2,2,2);
bar(metrics.heading_rmse' * 180/pi);
set(gca, 'XTickLabel', scenario_names);
ylabel('航向RMSE (°)');
title('航向跟踪精度');
legend(method_names, 'Location', 'northwest', 'FontSize', 8);
grid on;

subplot(2,2,3);
bar(metrics.ctrl_energy' / 1e3);
set(gca, 'XTickLabel', scenario_names);
ylabel('控制能量 (×10³ N²·s)');
title('控制能量消耗');
legend(method_names, 'Location', 'northwest', 'FontSize', 8);
grid on;

subplot(2,2,4);
bar(metrics.chattering');
set(gca, 'XTickLabel', scenario_names);
ylabel('抖振指数 (N²)');
title('控制抖振');
legend(method_names, 'Location', 'northwest', 'FontSize', 8);
grid on;

sgtitle('性能指标对比（三种方法 × 四种海流场景）', 'FontSize', 12);

%% 图3: 时域响应对比（取最后一个场景，最能体现PI-ESO优势）
s = n_scenarios;
fig3 = figure('Name', '场景2时域响应', 'Position', [50 50 1200 900]);

subplot(3,1,1);
hold on; grid on;
for m = 1:n_methods
    hist = results{m, s};
    t_plot = hist.t;
    % 计算实时横向误差
    cross_err = compute_cross_err_ts(hist);
    plot(t_plot, cross_err, 'Color', colors{m}, 'LineStyle', linestyles{m}, ...
         'LineWidth', 1.2, 'DisplayName', method_names{m});
end
ylabel('横向误差 (m)');
title('场景2（Gauss-Markov缓变流）：横向跟踪误差');
legend('Location', 'northeast', 'FontSize', 9);

subplot(3,1,2);
hold on; grid on;
for m = 1:n_methods
    hist = results{m, s};
    psi_err = ssa_vec(hist.x(:,12) - hist.xd(:,1)) * 180/pi;
    plot(hist.t, psi_err, 'Color', colors{m}, 'LineStyle', linestyles{m}, ...
         'LineWidth', 1.2, 'DisplayName', method_names{m});
end
ylabel('航向误差 (°)');
title('航向跟踪误差');
legend('Location', 'northeast', 'FontSize', 9);

subplot(3,1,3);
hold on; grid on;
% 绘制实际海流速度
hist_ref = results{1, s};
plot(hist_ref.t, hist_ref.Vc_t, 'k-', 'LineWidth', 1.5, 'DisplayName', '实际流速');
% 绘制ESO估计的流速（从hat_d反推）
for m = 2:n_methods
    hist = results{m, s};
    % hat_d(1) ≈ 海流在surge方向的力 = M(1,1) * a_current_surge
    % 近似流速估计（仅surge通道）
    M11 = 44.2773;  % m_eff surge
    Vc_est = abs(hist.hat_d(:,1)) / (M11 * 0.5);  % 粗略估计
    plot(hist.t, Vc_est, 'Color', colors{m}, 'LineStyle', linestyles{m}, ...
         'LineWidth', 1.2, 'DisplayName', [method_names{m} ' 估计']);
end
ylabel('流速 (m/s)');
xlabel('时间 (s)');
title('海流速度：实际 vs ESO估计');
legend('Location', 'northeast', 'FontSize', 9);

sgtitle(sprintf('场景%d（%s）时域响应对比', s, scenario_names{s}), 'FontSize', 12);

%% 图4: ESO扰动估计对比（同一场景）
fig4 = figure('Name', 'ESO扰动估计对比', 'Position', [50 50 1200 600]);

subplot(2,1,1);
hold on; grid on;
for m = 2:n_methods
    hist = results{m, s};
    plot(hist.t, hist.hat_d(:,1), 'Color', colors{m}, 'LineStyle', linestyles{m}, ...
         'LineWidth', 1.2, 'DisplayName', [method_names{m} ' hat_d(surge)']);
end
ylabel('扰动估计 (N)');
title('Surge通道扰动估计对比（场景2）');
legend('Location', 'northeast', 'FontSize', 9);
grid on;

subplot(2,1,2);
hold on; grid on;
for m = 2:n_methods
    hist = results{m, s};
    plot(hist.t, hist.hat_d(:,6), 'Color', colors{m}, 'LineStyle', linestyles{m}, ...
         'LineWidth', 1.2, 'DisplayName', [method_names{m} ' hat_d(yaw)']);
end
ylabel('扰动估计 (N·m)');
xlabel('时间 (s)');
title('Yaw通道扰动估计对比（场景2）');
legend('Location', 'northeast', 'FontSize', 9);
grid on;

sgtitle(sprintf('ESO扰动估计对比（场景%d: %s）', s, scenario_names{s}), 'FontSize', 12);

%% 保存图片
if ~exist('results', 'dir'), mkdir('results'); end
saveas(fig1, 'results/pi_eso_trajectory.png');
saveas(fig2, 'results/pi_eso_metrics.png');
saveas(fig3, 'results/pi_eso_timeseries.png');
saveas(fig4, 'results/pi_eso_estimation.png');
fprintf('图片已保存至 results/ 目录\n');
end

%% 辅助函数
function cross_err = compute_cross_err_ts(hist)
pts   = hist.traj;
x_pos = hist.x(:,7);
y_pos = hist.x(:,8);
n     = length(x_pos);
cross_err = zeros(n, 1);
for i = 1:n
    dx = pts.pos.x - x_pos(i);
    dy = pts.pos.y - y_pos(i);
    [~, idx] = min(dx.^2 + dy.^2);
    cross_err(i) = sqrt(dx(idx)^2 + dy(idx)^2);
end
end

function psi_err = ssa_vec(psi_err)
psi_err = mod(psi_err + pi, 2*pi) - pi;
end
