%% plot_segment_responses.m
% 按通道绘制各推力等级的阶跃响应时序图
% 输入: data/csv/auv_data_merged.csv + auv_segments_final.csv
%
% 每个通道生成一张大图，包含:
%   - 各推力等级的速度/角速度响应叠加
%   - 对应指令输入
%   - 动力电源功率 (如有)

clear; close all;

% -------------------------------------------------------------------------
% 0. 路径 & 参数
% -------------------------------------------------------------------------
project_root = fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
if isempty(project_root)
    project_root = pwd;
end
data_dir = fullfile(project_root, 'data', 'csv');
out_dir  = fullfile(project_root, 'data', 'figures');
if ~exist(out_dir, 'dir'), mkdir(out_dir); end

% 读取主数据
fprintf('[1/4] 加载主数据...\n');
data_table = readtable(fullfile(data_dir, 'auv_data_merged.csv'));

% 读取段索引
fprintf('[2/4] 加载段索引...\n');
seg_table = readtable(fullfile(data_dir, 'auv_segments_final.csv'));

% 提取数组
t = data_table.stamp_time;
t0 = t(1);
motor_TX = data_table.motor_TX;
motor_TY = data_table.motor_TY;
motor_TZ = data_table.motor_TZ;
motor_MZ = data_table.motor_MZ;
linear_vx = data_table.linear_vx;
linear_vy = data_table.linear_vy;
angular_wz = data_table.angular_wz;
prop_pwr = data_table.prop_pwr_power;
ctrl_pwr = data_table.ctrl_pwr_power;
depth    = data_table.depth;
yaw      = data_table.yaw_rad;  % 注意列名可能不同

% -------------------------------------------------------------------------
% 3. 按通道分组绘制
% -------------------------------------------------------------------------
fprintf('[3/4] 绘图...\n');

channels = {'TX', 'TY', 'MZ'};
ch_labels = {'前进/后退 (Surge)', '右移/左移 (Sway)', '右转/左转 (Yaw)'};
cmd_cols  = {motor_TX, motor_TY, motor_MZ};
vel_cols  = {linear_vx, linear_vy, angular_wz};
vel_units = {'v_x (m/s)', 'v_y (m/s)', '\omega_z (°/s)'};

% 颜色映射: 20%->40%->60%->80%->100%
thrust_levels = [20, 40, 60, 80, 100];
colors = lines(length(thrust_levels));
level_color = containers.Map(thrust_levels, num2cell(colors, 2));

for ch_idx = 1:3
    ch_name = channels{ch_idx};
    cmd = cmd_cols{ch_idx};
    vel = vel_cols{ch_idx};
    v_unit = vel_units{ch_idx};

    % 筛选该通道的段
    ch_segs = seg_table(strcmp(seg_table.channel, ch_name), :);

    if isempty(ch_segs)
        fprintf('  %s: 无数据，跳过\n', ch_name);
        continue;
    end

    % 按推力等级和方向分组
    % 获取该通道所有出现的推力等级
    levels_in_data = unique(ch_segs.amplitude);
    n_levels = length(levels_in_data);

    % 分方向
    dir_pos = ch_segs(ch_segs.direction == 1, :);   % 前进/右移/右转
    dir_neg = ch_segs(ch_segs.direction == -1, :);  % 后退/左移/左转

    figure('Position', [50, 50, 1600, 900], 'Name', ch_name);

    % ---- 子图布局 ----
    % 左列: 正向 (前进/右移/右转)
    % 右列: 负向 (后退/左移/左转)
    % 行: 速度 + 指令 + 功率

    n_rows = 3;
    n_cols = 2;

    % ============ 正向 ============
    subplot_idx = @(r, c) (r-1)*n_cols + c;

    % -- 速度响应 --
    ax_vel_pos = subplot(n_rows, n_cols, subplot_idx(1, 1));
    hold on; grid on;
    title(sprintf('%s 正向', ch_labels{ch_idx}));

    % -- 指令 --
    ax_cmd_pos = subplot(n_rows, n_cols, subplot_idx(2, 1));
    hold on; grid on;
    ylabel('指令');

    % -- 功率 --
    ax_pwr_pos = subplot(n_rows, n_cols, subplot_idx(3, 1));
    hold on; grid on;
    xlabel('时间 (s)'); ylabel('动力功率 (W)');

    for i = 1:height(dir_pos)
        seg = dir_pos(i, :);
        idx_s = seg.idx_pad_start;
        idx_e = seg.idx_pad_end;
        seg_t = (t(idx_s:idx_e) - t(seg.idx_start))';  % 以段开始为零点
        seg_vel = vel(idx_s:idx_e);
        seg_cmd = cmd(idx_s:idx_e);
        seg_pwr = prop_pwr(idx_s:idx_e);
        amp_pct = seg.amp_pct{1};
        level_val = sscanf(amp_pct, '%d');
        clr = level_color(level_val);

        plot(ax_vel_pos, seg_t, seg_vel, 'Color', clr, 'LineWidth', 1.2);
        plot(ax_cmd_pos, seg_t, seg_cmd, 'Color', clr, 'LineWidth', 1.2);
        plot(ax_pwr_pos, seg_t, seg_pwr, 'Color', clr, 'LineWidth', 1);

        % 标注段区间
        t_start_seg = t(seg.idx_start) - t(seg.idx_start);
        t_end_seg = t(seg.idx_end) - t(seg.idx_start);
        xline(ax_vel_pos, t_start_seg, '--k', 'Alpha', 0.3);
        xline(ax_vel_pos, t_end_seg, '--k', 'Alpha', 0.3);
    end
    ylabel(ax_vel_pos, v_unit);

    % ============ 负向 ============
    ax_vel_neg = subplot(n_rows, n_cols, subplot_idx(1, 2));
    hold on; grid on;
    title(sprintf('%s 负向', ch_labels{ch_idx}));

    ax_cmd_neg = subplot(n_rows, n_cols, subplot_idx(2, 2));
    hold on; grid on;

    ax_pwr_neg = subplot(n_rows, n_cols, subplot_idx(3, 2));
    hold on; grid on;
    xlabel('时间 (s)');

    for i = 1:height(dir_neg)
        seg = dir_neg(i, :);
        idx_s = seg.idx_pad_start;
        idx_e = seg.idx_pad_end;
        seg_t = (t(idx_s:idx_e) - t(seg.idx_start))';
        seg_vel = vel(idx_s:idx_e);
        seg_cmd = cmd(idx_s:idx_e);
        seg_pwr = prop_pwr(idx_s:idx_e);
        amp_pct = seg.amp_pct{1};
        level_val = sscanf(amp_pct, '%d');
        clr = level_color(level_val);

        plot(ax_vel_neg, seg_t, seg_vel, 'Color', clr, 'LineWidth', 1.2);
        plot(ax_cmd_neg, seg_t, seg_cmd, 'Color', clr, 'LineWidth', 1.2);
        plot(ax_pwr_neg, seg_t, seg_pwr, 'Color', clr, 'LineWidth', 1);

        t_start_seg = t(seg.idx_start) - t(seg.idx_start);
        t_end_seg = t(seg.idx_end) - t(seg.idx_start);
        xline(ax_vel_neg, t_start_seg, '--k', 'Alpha', 0.3);
        xline(ax_vel_neg, t_end_seg, '--k', 'Alpha', 0.3);
    end

    % 图例
    L = {};
    for lv = thrust_levels
        L{end+1} = sprintf('%d%%', lv);
    end
    legend(ax_vel_pos, L, 'Location', 'best', 'FontSize', 8);
    legend(ax_vel_neg, L, 'Location', 'best', 'FontSize', 8);

    % Y 轴对齐
    linkaxes([ax_vel_pos, ax_vel_neg], 'y');
    linkaxes([ax_cmd_pos, ax_cmd_neg], 'y');

    % 保存
    sgtitle(sprintf('%s — 阶跃响应时序', ch_labels{ch_idx}), 'FontSize', 14);
    saveas(gcf, fullfile(out_dir, sprintf('response_%s.png', ch_name)));
    fprintf('  已保存: response_%s.png\n', ch_name);
end

% -------------------------------------------------------------------------
% 4. 汇总图: 稳态速度 vs 推力等级
% -------------------------------------------------------------------------
fprintf('[4/4] 稳态汇总图...\n');

figure('Position', [100, 100, 1400, 500]);

for ch_idx = 1:3
    ch_name = channels{ch_idx};
    ch_segs = seg_table(strcmp(seg_table.channel, ch_name), :);
    if isempty(ch_segs), continue; end

    subplot(1, 3, ch_idx);
    hold on; grid on;

    % 按推力等级汇总稳态速度均值
    levels = unique(ch_segs.amplitude);
    for d = [-1, 1]  % 负向/正向
        dir_segs = ch_segs(ch_segs.direction == d, :);
        for lv = levels'
            lv_segs = dir_segs(dir_segs.amplitude == lv, :);
            if isempty(lv_segs), continue; end

            if ch_idx == 3
                sv = lv_segs.steady_wz;
            elseif ch_idx == 2
                sv = lv_segs.steady_vy;
            else
                sv = lv_segs.steady_vx;
            end

            if height(lv_segs) == 1
                scatter(lv/100*100, sv, 60, 'filled');
            else
                scatter(repmat(lv/100*100, height(lv_segs), 1), sv, 60, 'filled');
            end
        end
    end

    xlabel('推力等级 (%)'); ylabel(v_unit);
    title(ch_labels{ch_idx});
end

saveas(gcf, fullfile(out_dir, 'steady_state_summary.png'));
fprintf('  已保存: steady_state_summary.png\n');

fprintf('\n完成! 图片保存在: %s\n', out_dir);
