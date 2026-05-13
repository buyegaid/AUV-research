% 测试ESO扰动观测器性能
% 在不同海流速度下测试ESO对扰动的估计精度

addpath('.', './Lib', './guidance', './controller/xhy', './controller/remus', './model', './eso', './post', './traj');

%% 参数设置
Vc_test = linspace(0, 0.5, 6);  % 海流速度测试点 (m/s)
betaVc = pi/4;  % 海流方向 45度
w_c = 0;        % 垂直海流为0

T_sim = 100;    % 仿真时长 (s)
h = 0.01;       % 时间步长 (s)
N = round(T_sim / h);

% 获取参数
params = get_params;

% 固定推进器输入（主推1000 RPM，保持前进）
ui = [1000; 0; 0; 0; 0];

%% 存储结果
results = cell(length(Vc_test), 1);

%% 主仿真循环
for idx = 1:length(Vc_test)
    Vc = Vc_test(idx);
    fprintf('测试海流速度: %.2f m/s\n', Vc);

    % 初始化状态
    x = zeros(12, 1);
    x(1) = 0.5;  % 初始前进速度

    % 初始化ESO状态 (6通道 x 3状态)
    Z = zeros(6, 3);
    Z(:, 1) = x(1:6);  % Z1 = 速度测量值

    % 历史记录
    hist_x = zeros(12, N);
    hist_Z = zeros(6, 3, N);
    hist_dist_true = zeros(6, N);  % 真实扰动
    hist_dist_est = zeros(6, N);   % ESO估计扰动

    for k = 1:N
        % 动力学仿真
        [xdot, ~, M, C, D, g, tau] = xhy(x, ui, Vc, betaVc, w_c);

        % 计算真实扰动（海流、阻力等引起的加速度变化）
        nu = x(1:6);
        % 已知加速度：推力 + 重力 + 阻尼 + 科氏力
        a_known = M \ (tau - C * nu - D * nu - g);
        a_total = xdot(1:6);  % 总加速度
        dist_true = a_total - a_known;  % 真实扰动（主要是海流和未建模动态）

        % ESO更新
        y_meas = nu;  % 测量值
        Z = vec_leso_update_adv(Z, y_meas, a_known, params.eso, h);

        % 记录数据
        hist_x(:, k) = x;
        hist_Z(:, :, k) = Z;
        hist_dist_true(:, k) = dist_true;
        hist_dist_est(:, k) = Z(:, 3);  % Z3是扰动估计

        % 状态更新
        x = x + h * xdot;
    end

    % 保存结果
    results{idx}.Vc = Vc;
    results{idx}.hist_x = hist_x;
    results{idx}.hist_Z = hist_Z;
    results{idx}.hist_dist_true = hist_dist_true;
    results{idx}.hist_dist_est = hist_dist_est;
end

%% 绘图：选择中间海流速度进行详细分析
idx_mid = ceil(length(Vc_test) / 2);
res = results{idx_mid};
t = (0:N-1) * h;

fig1 = figure('Position', [100, 100, 1400, 900]);

% 子图1-3：纵荡、横荡、沉浮通道
channels = {'纵荡(u)', '横荡(v)', '沉浮(w)'};
for i = 1:3
    subplot(3, 2, 2*i-1);
    plot(t, res.hist_dist_true(i, :), 'b-', 'LineWidth', 1.5); hold on;
    plot(t, res.hist_dist_est(i, :), 'r--', 'LineWidth', 1.5);
    xlabel('时间 (s)');
    ylabel('扰动加速度 (m/s²)');
    title(sprintf('%s通道扰动估计 (Vc=%.2f m/s)', channels{i}, res.Vc));
    legend('真实扰动', 'ESO估计', 'Location', 'best');
    grid on;

    % 估计误差
    subplot(3, 2, 2*i);
    err = res.hist_dist_est(i, :) - res.hist_dist_true(i, :);
    plot(t, err, 'k-', 'LineWidth', 1);
    xlabel('时间 (s)');
    ylabel('估计误差 (m/s²)');
    title(sprintf('%s通道估计误差', channels{i}));
    grid on;
end

sgtitle(sprintf('ESO扰动估计性能 (海流速度 %.2f m/s)', res.Vc));

%% 绘图2：旋转通道
fig2 = figure('Position', [150, 150, 1400, 900]);

channels_rot = {'横滚(p)', '俯仰(q)', '偏航(r)'};
for i = 4:6
    subplot(3, 2, 2*(i-3)-1);
    plot(t, res.hist_dist_true(i, :), 'b-', 'LineWidth', 1.5); hold on;
    plot(t, res.hist_dist_est(i, :), 'r--', 'LineWidth', 1.5);
    xlabel('时间 (s)');
    ylabel('扰动角加速度 (rad/s²)');
    title(sprintf('%s通道扰动估计 (Vc=%.2f m/s)', channels_rot{i-3}, res.Vc));
    legend('真实扰动', 'ESO估计', 'Location', 'best');
    grid on;

    % 估计误差
    subplot(3, 2, 2*(i-3));
    err = res.hist_dist_est(i, :) - res.hist_dist_true(i, :);
    plot(t, err, 'k-', 'LineWidth', 1);
    xlabel('时间 (s)');
    ylabel('估计误差 (rad/s²)');
    title(sprintf('%s通道估计误差', channels_rot{i-3}));
    grid on;
end

sgtitle(sprintf('ESO旋转通道扰动估计 (海流速度 %.2f m/s)', res.Vc));

%% 绘图3：不同海流速度下的估计性能对比
fig3 = figure('Position', [200, 200, 1200, 600]);

% 计算每个海流速度下的RMSE
rmse_all = zeros(length(Vc_test), 6);
for idx = 1:length(Vc_test)
    res = results{idx};
    for ch = 1:6
        err = res.hist_dist_est(ch, :) - res.hist_dist_true(ch, :);
        rmse_all(idx, ch) = sqrt(mean(err.^2));
    end
end

% 子图1：平移通道RMSE
subplot(1, 2, 1);
plot(Vc_test, rmse_all(:, 1), 'b-o', 'LineWidth', 1.5, 'MarkerSize', 6); hold on;
plot(Vc_test, rmse_all(:, 2), 'r-s', 'LineWidth', 1.5, 'MarkerSize', 6);
plot(Vc_test, rmse_all(:, 3), 'g-^', 'LineWidth', 1.5, 'MarkerSize', 6);
xlabel('海流速度 (m/s)');
ylabel('RMSE (m/s²)');
title('平移通道估计误差');
legend('纵荡(u)', '横荡(v)', '沉浮(w)', 'Location', 'best');
grid on;

% 子图2：旋转通道RMSE
subplot(1, 2, 2);
plot(Vc_test, rmse_all(:, 4), 'b-o', 'LineWidth', 1.5, 'MarkerSize', 6); hold on;
plot(Vc_test, rmse_all(:, 5), 'r-s', 'LineWidth', 1.5, 'MarkerSize', 6);
plot(Vc_test, rmse_all(:, 6), 'g-^', 'LineWidth', 1.5, 'MarkerSize', 6);
xlabel('海流速度 (m/s)');
ylabel('RMSE (rad/s²)');
title('旋转通道估计误差');
legend('横滚(p)', '俯仰(q)', '偏航(r)', 'Location', 'best');
grid on;

sgtitle('ESO估计性能随海流速度变化');

%% 保存图像
pic_dir = 'pic';
if ~exist(pic_dir, 'dir')
    mkdir(pic_dir);
end

figure(fig1);
saveas(fig1, fullfile(pic_dir, 'eso_translation_channels.png'));
fprintf('图像已保存至 pic/eso_translation_channels.png\n');

figure(fig2);
saveas(fig2, fullfile(pic_dir, 'eso_rotation_channels.png'));
fprintf('图像已保存至 pic/eso_rotation_channels.png\n');

figure(fig3);
saveas(fig3, fullfile(pic_dir, 'eso_performance_vs_current.png'));
fprintf('图像已保存至 pic/eso_performance_vs_current.png\n');

%% 打印统计结果
fprintf('\n========== ESO估计性能统计 ==========\n');
fprintf('%-12s %-12s %-12s %-12s %-12s %-12s %-12s\n', ...
    'Vc(m/s)', 'u-RMSE', 'v-RMSE', 'w-RMSE', 'p-RMSE', 'q-RMSE', 'r-RMSE');
fprintf('%s\n', repmat('-', 1, 90));
for idx = 1:length(Vc_test)
    fprintf('%-12.2f %-12.4f %-12.4f %-12.4f %-12.6f %-12.6f %-12.6f\n', ...
        Vc_test(idx), rmse_all(idx, 1), rmse_all(idx, 2), rmse_all(idx, 3), ...
        rmse_all(idx, 4), rmse_all(idx, 5), rmse_all(idx, 6));
end

fprintf('\n测试完成！\n');





