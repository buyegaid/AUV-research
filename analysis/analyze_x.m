% analyze_x.m
% 读取 data/segments/ 中的 TX（主推/X轴）段 CSV，绘制阶跃响应图并计算上升时间。

clear; clc;

repo_root = setup_paths();
seg_dir = fullfile(repo_root, 'data', 'segments_0616');
pic_dir = fullfile(repo_root, 'assets', 'figures');
if ~exist(pic_dir, 'dir'), mkdir(pic_dir); end

% 读取所有 TX 段
tx_files = dir(fullfile(seg_dir, '*_TX.csv'));
if isempty(tx_files)
    error('未找到 TX 段 CSV 文件，请先运行 analyze_pool_test_20260530.m 导出数据。');
end

n_files = length(tx_files);
fprintf('找到 %d 个 TX 段:\n', n_files);
for i = 1:n_files
    fprintf('  [%d] %s\n', i, tx_files(i).name);
end

% 批量读取
tx_data = cell(n_files, 1);
tx_names = cell(n_files, 1);
for i = 1:n_files
    fpath = fullfile(seg_dir, tx_files(i).name);
    tx_data{i} = readtable(fpath);
    [~, tx_names{i}] = fileparts(tx_files(i).name);
end

% 上升时间 & 时间常数参数
RT_PCT_LO = 0.10;  % 上升时间 10%
RT_PCT_HI = 0.90;  % 上升时间 90%
TC_PCT = 0.632;    % 时间常数 63.2% (1 - 1/e)
TX_STEP_THRESHOLD = 500;  % TX 跳变阈值 (CAN-g)
MERGE_GAP = 2;    % 同一次跳变跨采样点的合并间距
MIN_PLATEAU_S = 3.0;  % 恒定段最短持续时长，短于此的不算有效阶跃

% 每段的有效时间区间（自动从文件名生成，空=整段有效）
valid_ranges = struct();
for vi = 1:n_files
    valid_ranges.(tx_names{vi}) = [];
end

all_rise_times = [];  % 收集所有上升时间
all_tau = [];         % 收集所有时间常数
all_step_mag = [];    % 收集阶跃高度（CAN-g，正负分开）
all_u_ss = [];        % 收集稳态 u 速度
all_step_info = {};   % 阶跃信息文本

% =========================================================================
% 图1: 阶跃响应 + 上升时间标注
% =========================================================================
figure("Name", "TX 阶跃响应");
set(gcf, "Position", [100, 100, 1200, 500 * n_files]);

for i = 1:n_files
    T = tx_data{i};
    name = tx_names{i};

    % 应用有效时间区间过滤
    if isfield(valid_ranges, name) && ~isempty(valid_ranges.(name))
        t_range = valid_ranges.(name);
        valid_mask = T.t_rel >= t_range(1) & T.t_rel <= t_range(2);
        fprintf('\n%s: 有效区间 %.1f-%.1fs (%.1f%% 数据保留)\n', ...
            name, t_range(1), t_range(2), mean(valid_mask)*100);
    else
        valid_mask = true(height(T), 1);
        fprintf('\n%s: 全段有效\n', name);
    end

    % 只取有效区间内的数据做阶跃检测
    T_valid = T(valid_mask, :);
    n_valid = height(T_valid);

    subplot(n_files, 1, i);

    % 背景: 全数据（灰色）
    yyaxis left;
    plot(T.t_rel, T.u, 'Color', [0.7 0.7 0.7], 'LineWidth', 0.8);
    hold on;
    % 有效数据（蓝色加粗）
    h1 = plot(T_valid.t_rel, T_valid.u, 'b-', 'LineWidth', 1.5);
    ylabel('u (m/s)');

    % 有效区间标记（浅蓝竖线）
    if isfield(valid_ranges, name) && ~isempty(valid_ranges.(name))
        xline(t_range(1), 'b:', 'LineWidth', 1.0);
        xline(t_range(2), 'b:', 'LineWidth', 1.0);
    end

    % 右轴: force_cmd TX（仅有效区间用实线）
    yyaxis right;
    plot(T.t_rel, T.TX, 'Color', [0.85 0.75 0.75], 'LineWidth', 0.5);
    hold on;
    h2 = stairs(T_valid.t_rel, T_valid.TX, 'r-', 'LineWidth', 1.0);
    ylabel('TX (CAN-g)');

    % --- 检测 TX 阶跃跳变点 ---
    dTX = [0; abs(diff(T_valid.TX))];
    jump_candidates = find(dTX > TX_STEP_THRESHOLD);
    if ~isempty(jump_candidates)
        gaps = [MERGE_GAP + 1; diff(jump_candidates)];
        group_start = jump_candidates(gaps > MERGE_GAP);
    else
        group_start = [];
    end

    boundaries = [1; group_start; n_valid];
    fprintf('检测到 %d 个跳变点:\n', length(group_start));

    step_count = 0;
    for s = 1:length(group_start)
        k0 = group_start(s);
        k_prev = boundaries(s);
        k_next = boundaries(s + 2);

        % 恒定段持续时长检查（前后平台都要 ≥ MIN_PLATEAU_S）
        plateau_pre_s = T_valid.t_rel(k0) - T_valid.t_rel(k_prev);
        plateau_post_s = T_valid.t_rel(k_next) - T_valid.t_rel(k0);

        fprintf('  跳变 @t=%.1fs: 前平台 %.1fs, 后平台 %.1fs', ...
            T_valid.t_rel(k0), plateau_pre_s, plateau_post_s);

        if plateau_post_s < MIN_PLATEAU_S
            fprintf(' -> 后平台不足 %.0fs，跳过\n', MIN_PLATEAU_S);
            continue;
        end

        % 阶跃前基准 u：取阶跃前 1s 内的均值（紧跟阶跃时刻的速度，不一定是零）
        pre_win = max(1, round(1.0 * 8.3));  % 1s 窗口
        idx_pre_start = max(k_prev, k0 - pre_win);
        u_pre = mean(T_valid.u(idx_pre_start:k0));

        % 阶跃后稳态：从段末向前收缩，找真正收敛的稳态区间
        post_len = k_next - k0 + 1;
        u_post = NaN;
        for frac = [0.15, 0.20, 0.25, 0.30, 0.40]
            ss_start = k0 + max(1, round(post_len * (1 - frac)));
            u_tail = T_valid.u(ss_start:k_next);
            u_tail_mean = mean(u_tail);
            if abs(u_tail_mean) > 0.005
                dev = max(abs(u_tail - u_tail_mean)) / abs(u_tail_mean);
                if dev < 0.10  % 波动 < 10%，认为收敛
                    u_post = u_tail_mean;
                    break;
                end
            end
        end
        if isnan(u_post)
            % 回退：取末段 15%
            ss_start = k0 + max(1, round(post_len * 0.85));
            u_post = mean(T_valid.u(ss_start:k_next));
        end

        du = u_post - u_pre;
        fprintf(' | TX %+d->%+d, u %.3f->%.3f\n', ...
            T_valid.TX(k0-1), T_valid.TX(k0), u_pre, u_post);

        if abs(du) < 0.01
            fprintf('    -> 速度变化太小，跳过\n');
            continue;
        end

        % 搜索 10%、63.2%、90% 穿越点
        u_target_lo = u_pre + RT_PCT_LO * du;
        u_target_tc = u_pre + TC_PCT * du;
        u_target_hi = u_pre + RT_PCT_HI * du;

        t_lo = NaN; t_tc = NaN; t_hi = NaN;
        for k = k0:k_next-1
            uk0 = T_valid.u(k); uk1 = T_valid.u(k+1);
            if isnan(t_lo) && (uk0 - u_target_lo) * (uk1 - u_target_lo) <= 0
                t_lo = interp1([uk0 uk1], [T_valid.t_rel(k) T_valid.t_rel(k+1)], u_target_lo);
            end
            if isnan(t_tc) && ~isnan(t_lo) && (uk0 - u_target_tc) * (uk1 - u_target_tc) <= 0
                t_tc = interp1([uk0 uk1], [T_valid.t_rel(k) T_valid.t_rel(k+1)], u_target_tc);
            end
            if ~isnan(t_tc) && isnan(t_hi) && (uk0 - u_target_hi) * (uk1 - u_target_hi) <= 0
                t_hi = interp1([uk0 uk1], [T_valid.t_rel(k) T_valid.t_rel(k+1)], u_target_hi);
                break;
            end
        end

        % 计算从阶跃时刻起的时间常数
        t_step = T_valid.t_rel(k0);
        if ~isnan(t_tc)
            tau = t_tc - t_step;
        else
            tau = NaN;
        end

        if ~isnan(t_lo) && ~isnan(t_hi) && ~isnan(tau)
            tr = t_hi - t_lo;
            step_count = step_count + 1;
            step_mag = round(median(T_valid.TX(k0:k_next)) / 500) * 500;
            % 剔除回零段（阶跃后指令为 0）
            if abs(step_mag) < 500
                fprintf('    -> 回零段，跳过\n');
                continue;
            end
            all_rise_times(end+1) = tr;
            all_tau(end+1) = tau;
            all_step_mag(end+1) = step_mag;
            all_u_ss(end+1) = u_post;
            all_step_info{end+1} = sprintf('#%d %s\\newline%+d', ...
                length(all_step_info)+1, name, step_mag);

            yyaxis left;
            xline(t_lo, 'g--', 'LineWidth', 0.8);
            xline(t_hi, 'g--', 'LineWidth', 0.8);
            xline(t_tc, 'c-', 'LineWidth', 1.0);
            yl = ylim;
            patch([t_lo t_hi t_hi t_lo], [yl(1) yl(1) yl(2) yl(2)], ...
                [0 1 0], 'FaceAlpha', 0.08, 'EdgeColor', 'none');
            text(t_step + tau, u_pre + 0.63*du, ...
                sprintf('\\tau=%.1fs', tau), ...
                'FontSize', 8, 'Color', [0 0.7 0.7], ...
                'HorizontalAlignment', 'left', 'FontWeight', 'bold');
            fprintf('    -> 时间常数 tau = %.2f s, 上升时间 tr = %.2f s, tr/tau = %.2f\n', ...
                tau, tr, tr/tau);
        else
            fprintf('    -> 未找到穿越点 (lo=%d tc=%d hi=%d)\n', ...
                ~isnan(t_lo), ~isnan(t_tc), ~isnan(t_hi));
        end
    end

    fprintf('  -> 有效阶跃: %d / %d\n', step_count, length(group_start));

    % 标注阶跃边界
    yyaxis left;
    for s = 1:length(group_start)
        xline(T_valid.t_rel(group_start(s)), 'm:', 'LineWidth', 0.6);
    end

    xlabel('时间 (s)');
    title(sprintf('%s: 阶跃响应 (恒定段>=%.0fs)', name, MIN_PLATEAU_S));
    legend([h1, h2], {'u (有效区)', 'TX (有效区)'}, 'Location', 'best');
    grid on;
end
sgtitle('TX 主推阶跃响应 — 绿色区=上升时间, 紫虚线=阶跃, 蓝虚线=有效边界');

% =========================================================================
% 图2: 上升时间 & 时间常数汇总
% =========================================================================
figure("Name", "上升时间与时间常数汇总");
set(gcf, "Position", [200, 200, max(500, length(all_rise_times)*50), 400]);

% 分组柱状图
X = categorical(all_step_info);
X = reordercats(X, all_step_info);
b = bar(X, [all_tau(:), all_rise_times(:)], 'grouped');
b(1).FaceColor = [0 0.7 0.7];
b(2).FaceColor = [0.2 0.6 0.2];
hold on;

if ~isempty(all_tau)
    hl_tau = yline(mean(all_tau), 'c--', 'LineWidth', 1.0);
    hl_tr = yline(mean(all_rise_times), 'g--', 'LineWidth', 1.0);
    for k = 1:length(all_tau)
        text(k - 0.15, all_tau(k) + 0.05, sprintf('%.1f', all_tau(k)), ...
            'FontSize', 7, 'Color', [0 0.5 0.5], 'HorizontalAlignment', 'center');
        text(k + 0.15, all_rise_times(k) + 0.05, sprintf('%.1f', all_rise_times(k)), ...
            'FontSize', 7, 'Color', [0 0.4 0], 'HorizontalAlignment', 'center');
    end
    tau_mean_str = sprintf('tau均值=%.2fs', mean(all_tau));
    tr_mean_str = sprintf('tr均值=%.2fs', mean(all_rise_times));
else
    hl_tau = gobjects(0); hl_tr = gobjects(0);
    tau_mean_str = ''; tr_mean_str = '';
end
ylabel('时间 (s)');
title(sprintf('时间常数 tau (63%%) & 上升时间 tr (10%%-90%%)'));
legend([b(1), b(2), hl_tau, hl_tr], ...
    {'tau 时间常数', 'tr 上升时间', tau_mean_str, tr_mean_str}, ...
    'Location', 'best');
grid on;

% 按阶跃高度分组汇总
mags = unique(all_step_mag);
fprintf('\n按阶跃高度分组:\n');
for m = 1:length(mags)
    mask = all_step_mag == mags(m);
    n_m = sum(mask);
    if n_m > 0
        fprintf('  %%+5d CAN-g (n=%%d): tau=%%.2f+/-%%.2f s, tr=%%.2f+/-%%.2f s\n', ...
            mags(m), n_m, mean(all_tau(mask)), std(all_tau(mask)), ...
            mean(all_rise_times(mask)), std(all_rise_times(mask)));
    end
end

% =========================================================================
% 图3: 稳态速度 vs PWM（通过推力分配矩阵+限幅+死区计算）
% =========================================================================
if ~isempty(all_u_ss)
    % 计算每个稳态点的 PWM（TX→T5 主推）
    all_pwm_t5 = zeros(size(all_u_ss));
    for k = 1:length(all_u_ss)
        fw_cmd = [all_step_mag(k); 0; 0; 0; 0; 0];  % TX only
        pwm_us = xhy_force_moment_to_pwm(fw_cmd);
        all_pwm_t5(k) = pwm_us(5) - 1500;  % T5 PWM offset
    end

    figure("Name", "TX 稳态速度 vs PWM");
    set(gcf, "Position", [200, 200, 900, 400]);

    subplot(1, 2, 1);
    scatter(all_pwm_t5, all_u_ss, 60, 'b', 'filled', 'MarkerFaceAlpha', 0.6);
    hold on;
    for k = 1:length(all_u_ss)
        text(all_pwm_t5(k) + 2, all_u_ss(k), sprintf('%.3f', all_u_ss(k)), 'FontSize', 7);
    end
    xlabel('T5 PWM offset (us)');
    ylabel('稳态 u (m/s)');
    title('T5 PWM → 稳态前向速度');
    grid on;

    subplot(1, 2, 2);
    pwm_u = unique(all_pwm_t5);
    u_by_pwm = zeros(size(pwm_u));
    for m = 1:length(pwm_u)
        mask = abs(all_pwm_t5 - pwm_u(m)) < 1;
        u_by_pwm(m) = mean(all_u_ss(mask));
    end
    bar(pwm_u, u_by_pwm, 'FaceColor', [0.2 0.6 0.2]);
    hold on;
    plot(pwm_u, u_by_pwm, 'ko-', 'LineWidth', 1.5);
    for m = 1:length(pwm_u)
        text(pwm_u(m), u_by_pwm(m) + 0.005, sprintf('%.3f', u_by_pwm(m)), ...
            'HorizontalAlignment', 'center', 'FontSize', 8);
    end
    xlabel('T5 PWM offset (us)');
    ylabel('稳态 u (m/s)');
    title('各 PWM 档位平均稳态 u');
    grid on;
    sgtitle('TX 稳态速度 vs T5 PWM (推力分配矩阵+限幅+死区)');

    fprintf('\n稳态速度 vs PWM:\n');
    for m = 1:length(pwm_u)
        mask = abs(all_pwm_t5 - pwm_u(m)) < 1;
        fprintf('  PWM %+4.0f us (n=%d): u_ss=%.3f+/-%.3f m/s\n', ...
            pwm_u(m), sum(mask), mean(all_u_ss(mask)), std(all_u_ss(mask)));
    end
end

fprintf('\n===== 汇总 =====\n');
fprintf('有效阶跃数: %d\n', length(all_rise_times));
if ~isempty(all_rise_times)
    fprintf('上升时间 tr (10%%-90%%): 均值=%.2f s, 最小=%.2f s, 最大=%.2f s\n', ...
        mean(all_rise_times), min(all_rise_times), max(all_rise_times));
    fprintf('时间常数 tau (63%%):   均值=%.2f s, 最小=%.2f s, 最大=%.2f s\n', ...
        mean(all_tau), min(all_tau), max(all_tau));
    fprintf('tr/tau 比值:           均值=%.2f (理论一阶=%.2f)\n', ...
        mean(all_rise_times ./ all_tau), log(0.9/0.1));
end
fprintf('===== analyze_x 完成 =====\n');
