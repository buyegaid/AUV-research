function plot_trajectory_comparison_cn()
% 绘制轨迹对比实验图（中文版）
% 2026-06-23

project_root = setup_paths();
load(fullfile(project_root, 'results', 'trajectory_comparison.mat'), 'merged');

obs_int  = {'KIN','BORHAUG','EKF','UCCO'};
obs_lbl  = {'KIN','CFD-Luenberger','EKF','PC-RCO'};
colors   = {[0.5 0.5 0.5], [0.8 0.4 0], [0.2 0.6 0.8], [0.9 0.2 0.2]};
markers  = {'s','d','^','o'};
mismatches = [0, 10, 20, 30];

%% ===== 图1: 轨迹对比（圆形+直线并排） =====
figure('Position', [100 100 1200 480]);

% 左: 圆形
subplot(1,2,1); hold on;
for oi = 1:4
    vals = arrayfun(@(m) merged.circle.(obs_int{oi}).(['m' num2str(m)]).rmse_mean, mismatches);
    plot(mismatches, vals, ['-' markers{oi}], 'Color', colors{oi}, ...
        'LineWidth', 2, 'MarkerSize', 8, 'MarkerFaceColor', colors{oi});
end
xlabel('CFD阻力系数失配 (%)', 'FontSize', 12);
ylabel('RMSE_{V_c} (m/s)', 'FontSize', 12);
title('圆形轨迹（持续激励）', 'FontSize', 13);
legend(obs_lbl, 'Location', 'northwest', 'FontSize', 10);
grid on; box on; ylim([0 0.5]); set(gca, 'FontSize', 11);

% 右: 直线
subplot(1,2,2); hold on;
for oi = 1:4
    vals = arrayfun(@(m) merged.straight.(obs_int{oi}).(['m' num2str(m)]).rmse_mean, mismatches);
    plot(mismatches, vals, ['-' markers{oi}], 'Color', colors{oi}, ...
        'LineWidth', 2, 'MarkerSize', 8, 'MarkerFaceColor', colors{oi});
end
xlabel('CFD阻力系数失配 (%)', 'FontSize', 12);
ylabel('RMSE_{V_c} (m/s)', 'FontSize', 12);
title('直线轨迹（低激励）', 'FontSize', 13);
legend(obs_lbl, 'Location', 'northwest', 'FontSize', 10);
grid on; box on; ylim([0 0.5]); set(gca, 'FontSize', 11);

sgtitle('海流估计精度：轨迹类型对比（5 seeds, 低噪声 \sigma_v=0.02 m/s）', ...
    'FontSize', 14, 'FontWeight', 'bold');

out_dir = fullfile(project_root, 'figures');
if ~exist(out_dir, 'dir'), mkdir(out_dir); end

exportgraphics(gcf, fullfile(out_dir, 'trajectory_comparison_cn.pdf'), ...
    'ContentType', 'vector', 'BackgroundColor', 'white');
exportgraphics(gcf, fullfile(out_dir, 'trajectory_comparison_cn.png'), ...
    'Resolution', 300, 'BackgroundColor', 'white');

fprintf('图1 已保存\n');

%% ===== 图2: 退化率对比（中文版） =====
figure('Position', [100 100 800 450]);
degradation = zeros(4, 2);
for ti = 1:2
    traj = {'circle','straight'};
    for oi = 1:4
        base = merged.(traj{ti}).(obs_int{oi}).m0.rmse_mean;
        deg  = merged.(traj{ti}).(obs_int{oi}).m30.rmse_mean;
        degradation(oi, ti) = (deg - base) / base * 100;
    end
end

b = bar(degradation);
b(1).FaceColor = [0.3 0.5 0.8];
b(2).FaceColor = [0.8 0.4 0.2];
set(gca, 'XTickLabel', obs_lbl, 'FontSize', 12);
ylabel('RMSE退化率 0%→30% 失配 (%)', 'FontSize', 12);
legend('圆形轨迹', '直线轨迹', 'Location', 'northwest', 'FontSize', 11);
title('模型失配鲁棒性：退化率对比', 'FontSize', 13);
grid on; box on;

% 柱上标注
for ti = 1:2
    x = (1:4) + (ti-1.5)*0.3;
    for oi = 1:4
        text(x(oi), degradation(oi,ti) + sign(degradation(oi,ti))*10, ...
            sprintf('%.0f%%', degradation(oi,ti)), ...
            'HorizontalAlignment', 'center', 'FontSize', 10, 'FontWeight', 'bold');
    end
end

exportgraphics(gcf, fullfile(out_dir, 'degradation_comparison_cn.pdf'), ...
    'ContentType', 'vector', 'BackgroundColor', 'white');
exportgraphics(gcf, fullfile(out_dir, 'degradation_comparison_cn.png'), ...
    'Resolution', 300, 'BackgroundColor', 'white');

fprintf('图2 已保存\n');
fprintf('完成\n');
end
