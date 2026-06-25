function plot_trajectory_comparison()
% PLOT_TRAJECTORY_COMPARISON  绘制轨迹对比实验图
%   圆形 vs 直线 × 4基线 × 4失配水平的 RMSE 对比图
%   2026-06-23

project_root = setup_paths();
load(fullfile(project_root, 'results', 'trajectory_comparison.mat'), 'merged');

obs_int = {'KIN','BORHAUG','EKF','UCCO'};
obs_lbl = {'KIN','CFD-Luenberger','EKF','PC-RCO'};
colors  = {[0.5 0.5 0.5], [0.8 0.4 0], [0.2 0.6 0.8], [0.9 0.2 0.2]};  % 灰,橙,青,红
markers = {'s','d','^','o'};
mismatches = [0, 10, 20, 30];

figure('Position', [100 100 1200 480]);

%% ===== 左图: 圆形轨迹 =====
subplot(1,2,1); hold on;
for oi = 1:4
    vals = zeros(1,4);
    for mi = 1:4
        vals(mi) = merged.circle.(obs_int{oi}).(['m' num2str(mismatches(mi))]).rmse_mean;
    end
    plot(mismatches, vals, ['-' markers{oi}], 'Color', colors{oi}, ...
        'LineWidth', 2, 'MarkerSize', 8, 'MarkerFaceColor', colors{oi});
end
xlabel('CFD Drag Mismatch (%)', 'FontSize', 12);
ylabel('RMSE_{V_c} (m/s)', 'FontSize', 12);
title('Circle Trajectory (Sustained Excitation)', 'FontSize', 13);
legend(obs_lbl, 'Location', 'northwest', 'FontSize', 10);
grid on; box on;
ylim([0 0.5]);
set(gca, 'FontSize', 11);

%% ===== 右图: 直线轨迹 =====
subplot(1,2,2); hold on;
for oi = 1:4
    vals = zeros(1,4);
    for mi = 1:4
        vals(mi) = merged.straight.(obs_int{oi}).(['m' num2str(mismatches(mi))]).rmse_mean;
    end
    plot(mismatches, vals, ['-' markers{oi}], 'Color', colors{oi}, ...
        'LineWidth', 2, 'MarkerSize', 8, 'MarkerFaceColor', colors{oi});
end
xlabel('CFD Drag Mismatch (%)', 'FontSize', 12);
ylabel('RMSE_{V_c} (m/s)', 'FontSize', 12);
title('Straight Trajectory (Low Excitation)', 'FontSize', 13);
legend(obs_lbl, 'Location', 'northwest', 'FontSize', 10);
grid on; box on;
ylim([0 0.5]);
set(gca, 'FontSize', 11);

sgtitle('Current Estimation Accuracy: Trajectory Comparison (5 seeds, low noise \sigma_v=0.02 m/s)', ...
    'FontSize', 14, 'FontWeight', 'bold');

%% ===== 保存 =====
out_dir = fullfile(project_root, 'figures');
if ~exist(out_dir, 'dir'), mkdir(out_dir); end

exportgraphics(gcf, fullfile(out_dir, 'trajectory_comparison.pdf'), ...
    'ContentType', 'vector', 'BackgroundColor', 'white');
exportgraphics(gcf, fullfile(out_dir, 'trajectory_comparison.png'), ...
    'Resolution', 300, 'BackgroundColor', 'white');

fprintf('图片已保存到: %s\n', fullfile(out_dir, 'trajectory_comparison.pdf'));

%% ===== 第二张图: 退化率对比 =====
figure('Position', [100 100 800 450]);
degradation = zeros(4, 2);  % [圆形, 直线] × 4基线
for ti = 1:2
    traj = {'circle','straight'};
    for oi = 1:4
        base = merged.(traj{ti}).(obs_int{oi}).m0.rmse_mean;
        deg  = merged.(traj{ti}).(obs_int{oi}).m30.rmse_mean;
        degradation(oi, ti) = (deg - base) / base * 100;
    end
end

b = bar(degradation);
b(1).FaceColor = [0.3 0.5 0.8];  % 圆形
b(2).FaceColor = [0.8 0.4 0.2];  % 直线
set(gca, 'XTickLabel', obs_lbl, 'FontSize', 12);
ylabel('RMSE Degradation 0%→30% Mismatch (%)', 'FontSize', 12);
legend('Circle', 'Straight', 'Location', 'northwest', 'FontSize', 11);
title('Mismatch Robustness: Degradation Comparison', 'FontSize', 13);
grid on; box on;

% 在柱上标值
for ti = 1:2
    x = (1:4) + (ti-1.5)*0.3;
    for oi = 1:4
        text(x(oi), degradation(oi,ti) + sign(degradation(oi,ti))*5, ...
            sprintf('%.0f%%', degradation(oi,ti)), ...
            'HorizontalAlignment', 'center', 'FontSize', 10, 'FontWeight', 'bold');
    end
end

exportgraphics(gcf, fullfile(out_dir, 'degradation_comparison.pdf'), ...
    'ContentType', 'vector', 'BackgroundColor', 'white');
exportgraphics(gcf, fullfile(out_dir, 'degradation_comparison.png'), ...
    'Resolution', 300, 'BackgroundColor', 'white');

fprintf('退化率图已保存到: %s\n', fullfile(out_dir, 'degradation_comparison.pdf'));
fprintf('完成\n');
end
