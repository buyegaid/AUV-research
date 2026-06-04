%% 海流观测器对比分析
% 对比7种方法的: 海流估计精度、控制性能、计算时间
% 2026-06-04

function compare_current_estimators(results, method_names)
% results: cell array of result structures from xhy_sim_compare
% method_names: cell array of method name strings

if nargin < 1
    error('请先运行 xhy_sim_compare(8, ...) 生成对比数据');
end

n = length(results);

%% ====== 1. 海流估计精度 ======
fprintf('\n========== 海流估计性能 ==========\n');
fprintf('%-12s %10s %10s %10s %10s\n', '方法', 'RMSE_Vc', 'MAE_Vc', 'RMSE_cN', 'RMSE_cE');
fprintf('%s\n', repmat('-', 1, 55));

rmse_Vc = zeros(n,1); mae_Vc = zeros(n,1);
rmse_cN = zeros(n,1); rmse_cE = zeros(n,1);

for i = 1:n
    c_est = results{i}.hist.c_est;
    c_true = results{i}.hist.c_true;
    c_err = c_est - c_true;
    Vc_err = sqrt(c_err(:,1).^2 + c_err(:,2).^2);

    rmse_Vc(i) = sqrt(mean(Vc_err.^2));
    mae_Vc(i) = mean(abs(Vc_err));
    rmse_cN(i) = sqrt(mean(c_err(:,1).^2));
    rmse_cE(i) = sqrt(mean(c_err(:,2).^2));

    fprintf('%-12s %10.4f %10.4f %10.4f %10.4f\n', ...
        method_names{i}, rmse_Vc(i), mae_Vc(i), rmse_cN(i), rmse_cE(i));
end

%% ====== 2. 控制性能 ======
fprintf('\n========== 控制性能 ==========\n');
fprintf('%-12s %10s %10s %10s %10s\n', '方法', 'RMSE_psi', 'RMSE_xy', 'RMS_tau', 'RMS_n');
fprintf('%s\n', repmat('-', 1, 55));

rmse_psi = zeros(n,1); rmse_xy = zeros(n,1);
rms_tau = zeros(n,1); rms_rpm = zeros(n,1);

for i = 1:n
    x = results{i}.hist.x;
    tau = results{i}.hist.tau;
    ui = results{i}.hist.ui;

    rmse_psi(i) = sqrt(mean((x(:,12) - results{i}.hist.xd(:,1)).^2));
    x_err = x(:,7) - results{i}.hist.traj.pos.x;
    y_err = x(:,8) - results{i}.hist.traj.pos.y;
    rmse_xy(i) = sqrt(mean(x_err.^2 + y_err.^2));
    rms_tau(i) = sqrt(mean(tau(:,1).^2));  % Surge force RMS
    rms_rpm(i) = sqrt(mean(ui(:,1).^2));   % Main thruster RPM

    fprintf('%-12s %10.4f %10.4f %10.4f %10.2f\n', ...
        method_names{i}, rmse_psi(i), rmse_xy(i), rms_tau(i), rms_rpm(i));
end

%% ====== 3. 绘图 ======
% 3.1 海流估计误差时程
figure('Name', '海流估计误差对比', 'Position', [100 100 1400 400]);
for i = 1:n
    subplot(1, n, i);
    t = results{i}.hist.t;
    c_est = results{i}.hist.c_est;
    c_true = results{i}.hist.c_true;

    % 丢弃前10%（初始瞬态）
    start_idx = round(0.1 * length(t));

    Vc_err = sqrt((c_est(:,1)-c_true(:,1)).^2 + (c_est(:,2)-c_true(:,2)).^2);
    plot(t(start_idx:end), Vc_err(start_idx:end), 'b-', 'LineWidth', 1);
    xlabel('时间 (s)'); ylabel('|c_{err}| (m/s)');
    title(sprintf('%s', method_names{i}));
    ylim([0, 0.3]); grid on;
    text(0.05, 0.9, sprintf('RMSE=%.4f', rmse_Vc(i)), 'Units', 'normalized');
end
sgtitle('海流估计误差 (Vc误差幅值)');

% 3.2 海流估计 vs 真值 (cN, cE)
figure('Name', '海流分量估计对比', 'Position', [100 550 1400 500]);
for i = 1:n
    subplot(2, 3, i);
    t = results{i}.hist.t;
    c_est = results{i}.hist.c_est;
    c_true = results{i}.hist.c_true;

    plot(t, c_true(:,1), 'k--', 'LineWidth', 1.5); hold on;
    plot(t, c_est(:,1), 'b-', 'LineWidth', 1);
    xlabel('时间 (s)'); ylabel('c_N (m/s)');
    title(method_names{i}); grid on;
    if i == 1, legend('真值', '估计', 'Location', 'best'); end
end
sgtitle('北向海流分量: 估计 vs 真值');

% 3.3 柱状图对比
figure('Name', '性能对比柱状图', 'Position', [100 100 1000 800]);

subplot(2,2,1);
bar(rmse_Vc); set(gca, 'XTickLabel', method_names);
ylabel('RMSE V_c (m/s)'); title('海流估计精度');
grid on;

subplot(2,2,2);
bar(rmse_psi * 180/pi); set(gca, 'XTickLabel', method_names);
ylabel('RMSE \psi (deg)'); title('航向跟踪精度');
grid on;

subplot(2,2,3);
bar(rmse_xy); set(gca, 'XTickLabel', method_names);
ylabel('RMSE_{xy} (m)'); title('横向轨迹误差');
grid on;

subplot(2,2,4);
bar(rms_rpm); set(gca, 'XTickLabel', method_names);
ylabel('RMS RPM'); title('推进器能耗');
grid on;

sgtitle('观测器方法性能对比');

%% ====== 4. 归一化得分 ======
fprintf('\n========== 综合评分（越低越好）==========\n');
% 归一化到 [0,1]，取等权平均
score_Vc = rmse_Vc / max(rmse_Vc);
score_psi = rmse_psi / max(rmse_psi);
score_xy = rmse_xy / max(rmse_xy);
score_energy = rms_rpm / max(rms_rpm);

total_score = score_Vc + score_psi + score_xy + score_energy;

fprintf('%-12s %8s %8s %8s %8s %8s\n', '方法', 'Vc', 'psi', 'xy', '能耗', '总分');
fprintf('%s\n', repmat('-', 1, 55));
for i = 1:n
    fprintf('%-12s %8.3f %8.3f %8.3f %8.3f %8.3f\n', ...
        method_names{i}, score_Vc(i), score_psi(i), score_xy(i), score_energy(i), total_score(i));
end

[~, best_idx] = min(total_score);
fprintf('\n🏆 最佳方法: %s (总分=%.3f)\n', method_names{best_idx}, total_score(best_idx));
end
