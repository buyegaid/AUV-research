% analyze_pool_test_20260530_reanalysis.m
% 使用当前 force_cmd -> PWM -> M080/M060 推进器模型链路复分析 2026-05-30 水池 debug 数据。
%
% 说明：
%   1. debug_data260530-1.csv 中 force_cmd 是下发到 CAN 分配逻辑的原始输入。
%   2. 平动力 TX/TY/TZ 按 CAN-g 输入处理。
%   3. 转动力矩 MX/MY/MZ 按原始力矩输入处理，进入分配矩阵前不再额外乘 100。
%   4. 推进器顺序统一采用文档顺序 [T1 T2 T3 T4 T5]。

clear; clc;

repo_root = setup_paths(); % 获取根目录

data_file = fullfile(repo_root, 'data', 'raw', 'debug_data260530-1.csv'); % 目标数据文件
out_dir = fullfile(repo_root, 'results'); % 文档输出目录
pic_dir = fullfile(repo_root, 'assets', 'figures'); % 图片输出目录
if ~exist(out_dir, 'dir'), mkdir(out_dir); end
if ~exist(pic_dir, 'dir'), mkdir(pic_dir); end

T = readtable(data_file); % 获取原始数据文件

t = T.pc_timestamp - T.pc_timestamp(1); % 获取时间序列
force_cmd = [T.force_cmd1, T.force_cmd2, T.force_cmd3, T.force_cmd4, T.force_cmd5, T.force_cmd6]; % 获取力矩列
voltage = T.control_voltage; % 获取电压序列
current = T.power_current; % 获取电流序列
power = voltage .* current; % 获取电压序列 
mode = T.mode; % 运行模式 

vel_linear = [T.linear_vel_x, T.linear_vel_y, T.linear_vel_z];
vel_angular_deg = [T.angular_vel_x, T.angular_vel_y, T.angular_vel_z];
vel = [vel_linear, deg2rad(vel_angular_deg)];

dof_names = {'Surge X', 'Sway Y', 'Heave Z', 'Roll K', 'Pitch M', 'Yaw N'};
vel_names = {'u', 'v', 'w', 'p', 'q', 'r'};
thruster_names = {'T1 前垂推', 'T2 后垂推', 'T3 前侧推', 'T4 后侧推', 'T5 主推'};

% 采样率
FS = 8.3;           % 主数据采样率 (Hz) — force_cmd/vel/mode
FS_POWER = 2.0;     % 电压电流更新率 (Hz)


%% =========================================================================
% 第一节：运行模式与功率
% =========================================================================
figure("Name", "运行模式与功率");
set(gcf, "Position", [100, 100, 900, 450]);

% 左轴：运行模式
yyaxis left;
h_mode = plot(t, mode, 'b-', 'LineWidth', 1.2);
ylabel('运行模式');
ylim([-0.5, 4.5]);
yticks(0:4);
yticklabels({'0', '1', '2', '3', '4-自控'});

% 右轴：功率
yyaxis right;
h_pwr = plot(t, power, 'r-', 'LineWidth', 1);
ylabel('功率 (W)');
xlabel('时间 (s)');
title('运行模式与功率');
grid on;

% 标记 mode=4 区间（灰色半透明阴影）
hold on;
mode4_mask = (mode == 4);
if any(mode4_mask)
    yyaxis left;
    d_mode4 = diff([0; mode4_mask; 0]);
    start_idx = find(d_mode4 == 1);
    end_idx = find(d_mode4 == -1) - 1;
    for k = 1:length(start_idx)
        x_start = t(start_idx(k));
        x_end = t(end_idx(k));
        xl = xlim;
        yl = ylim;
        patch([x_start x_end x_end x_start], [yl(1) yl(1) yl(2) yl(2)], ...
            [0.7 0.7 0.7], 'FaceAlpha', 0.25, 'EdgeColor', 'none', ...
            'DisplayName', '自控区间');
    end
end

legend([h_mode, h_pwr], {'运行模式', '功率'}, 'Location', 'best');

%% =========================================================================
% 第二节：推力与功率
% 功率采样为 2Hz，存在滞后。控制功率取推力≈0且持续≥2s的稳态区间功率中位数，
% 以避开推力突变时功率来不及响应的瞬态滞后区。
% =========================================================================
figure("Name", "推力与功率");
set(gcf, "Position", [150, 150, 700, 500]);

% 计算总推力幅值并归一化（以10000为最大值）
thrust_total = vecnorm(force_cmd, 2, 2);       % 6-DOF 合力/力矩幅值
thrust_norm = thrust_total / 10000;             % 归一化到 [0, 1]

% --- 控制功率：稳态零推力区间功率中位数 ---
% 零推力判据：归一化推力幅值 < 1%（即原始幅值 < 100）
zero_thrust_mask = thrust_norm < 0.01;

% 找出连续零推力区间
d_mask = diff([0; zero_thrust_mask; 0]);
start_idx = find(d_mask == 1);
end_idx = find(d_mask == -1) - 1;

% 筛选持续 ≥ 2秒 的稳态区间（2Hz → 至少 4 个采样点）
min_duration_samples = 4;  % 2秒 @ 2Hz
steady_mask = false(size(power));
for k = 1:length(start_idx)
    if (end_idx(k) - start_idx(k) + 1) >= min_duration_samples
        steady_mask(start_idx(k):end_idx(k)) = true;
    end
end

if any(steady_mask)
    control_power = median(power(steady_mask));
else
    % 回退：取推力最小的 5% 样本的功率中位数
    [~, sort_idx] = sort(thrust_norm);
    low_idx = sort_idx(1:max(10, round(0.05 * length(thrust_norm))));
    control_power = median(power(low_idx));
end

% 动力功率 = 总功率 - 控制功率
propulsion_power = power - control_power;

fprintf('控制功率 (稳态零推力区间中位数): %.1f W\n', control_power);
fprintf('稳态零推力样本数: %d / %d (%.1f%%)\n', ...
    sum(steady_mask), length(power), sum(steady_mask)/length(power)*100);
fprintf('总功率范围: %.1f ~ %.1f W, 动力功率范围: %.1f ~ %.1f W\n', ...
    min(power), max(power), min(propulsion_power), max(propulsion_power));

% 散点图：归一化推力 vs 功率
scatter(thrust_norm(~steady_mask), power(~steady_mask), 8, [0.6 0.6 0.6], 'filled', ...
    'MarkerFaceAlpha', 0.4, 'DisplayName', '瞬态/有推力点');
hold on;
% 稳态零推力点用不同颜色标出
scatter(thrust_norm(steady_mask), power(steady_mask), 12, 'g', 'filled', ...
    'MarkerFaceAlpha', 0.7, 'DisplayName', sprintf('稳态零推力点 (n=%d)', sum(steady_mask)));

% 控制功率参考线
yline(control_power, 'k--', 'LineWidth', 1.5, ...
    'DisplayName', sprintf('控制功率 = %.1f W', control_power));

xlabel('归一化推力幅值 |force\_cmd| / 10000');
ylabel('功率 (W)');
title(sprintf(['推力与功率关系\n', ...
    '控制功率 = %.1f W（零推力稳态中位数）, 动力功率 = 总功率 - 控制功率'], control_power));
legend('Location', 'best');
grid on;

% 标注
text(0.02, control_power + 5, '← 控制功率（稳态零推力区间中位数）', ...
    'FontSize', 9, 'Color', [0.3 0.3 0.3]);
text(0.85, mean(power(thrust_norm > 0.3)), '动力功率 →', ...
    'FontSize', 9, 'Color', [0.3 0.3 0.3], 'HorizontalAlignment', 'right');

%% =========================================================================
% 第三节：单轴推力到推进器 PWM 映射（静态标定曲线）
% 参考: Obsidian [[小黄鱼 推进器模型]] §2.3 PWM标定, §2.4 死区补偿
%   PWM 范围: 1000-2000μs (标准RC), 偏移 ±500μs
%   硬限幅: ±450μs (1050-1950μs), 固件限幅 ±500μs
%   死区: 全部推进器统一 +35/-15 μs
% =========================================================================
figure("Name", "单轴推力到推进器PWM映射");
set(gcf, "Position", [200, 200, 1000, 420]);

% 标定表 — 与固件一致 ([[小黄鱼 推进器模型]] §2.3)
%   CAN-g:   -7300  -6100  -4800  -3200     0   3900   5900   7900   9500
%   offset:   -430   -370   -330   -250     0    230    310    370    450
pos_thrust_g = [0;    3900; 5900; 7900; 9500];
pos_pwm_off  = [0;    230;  310;  370;  450];
neg_thrust_g = [-7300; -6100; -4800; -3200; 0];
neg_pwm_off  = [-430;  -370;  -330;  -250;  0];

% 死区 — 全部推进器统一 (文档 §2.4)
dz_pos = 35;   % 正向死区 (μs)
dz_neg = -15;  % 反向死区 (μs)

% PWM 偏移硬限幅 (文档 §2.3: "PWM 硬限幅为 ±450μs")
OFFSET_MAX = 450;   % 正向最大偏移
OFFSET_MIN = -450;  % 反向最大偏移

% 固件限幅 (文档 §2.4: "PWM 偏移限幅到 [-500, 500] μs")
OFFSET_FW_MAX = 500;
OFFSET_FW_MIN = -500;

% 推力限幅
THRUST_LIMIT_POS = 9500;   % CAN-g 正限幅
THRUST_LIMIT_NEG = -7300;  % CAN-g 反限幅

% 生成密集采样点
thrust_pos_dense = linspace(0, THRUST_LIMIT_POS, 200)';
thrust_neg_dense = linspace(THRUST_LIMIT_NEG, 0, 200)';

% 含死区的 PWM offset
pwm_pos_off = interp1(pos_thrust_g, pos_pwm_off, thrust_pos_dense, 'linear', 'extrap') + dz_pos;
pwm_neg_off = interp1(neg_thrust_g, neg_pwm_off, thrust_neg_dense, 'linear', 'extrap') + dz_neg;

% --- 子图1: 推力(g) → PWM offset(us) ---
subplot(1, 2, 1);
plot(thrust_pos_dense, pwm_pos_off, 'b-', 'LineWidth', 1.8);
hold on;
plot(thrust_neg_dense, pwm_neg_off, 'r-', 'LineWidth', 1.8);
% 标定节点 (不含死区的原始插值点)
plot(pos_thrust_g(2:end), pos_pwm_off(2:end) + dz_pos, 'bo', 'MarkerSize', 5, 'MarkerFaceColor', 'b');
plot(neg_thrust_g(1:end-1), neg_pwm_off(1:end-1) + dz_neg, 'ro', 'MarkerSize', 5, 'MarkerFaceColor', 'r');
% 零点参考
xline(0, 'k:', 'LineWidth', 0.8);
yline(0, 'k:', 'LineWidth', 0.8);
% 偏移硬限幅
yline(OFFSET_MAX, 'm--', 'LineWidth', 0.8);
yline(OFFSET_MIN, 'm--', 'LineWidth', 0.8);
xlabel('推力 (CAN-g)');
ylabel('PWM offset (us)');
title('推力 → PWM offset（含死区 +35/-15）');
legend({'正向', '反向', '正向节点', '反向节点', '硬限幅 ±450'}, 'Location', 'southeast');
grid on;

% --- 子图2: 完整 PWM 占空比 (us) ---
subplot(1, 2, 2);
thrust_full = linspace(THRUST_LIMIT_NEG, THRUST_LIMIT_POS, 500)';
pwm_full = zeros(size(thrust_full));
for i = 1:length(thrust_full)
    T = thrust_full(i);
    if abs(T) < 1e-6
        pwm_full(i) = 1500;
    elseif T > 0
        raw_off = interp1(pos_thrust_g, pos_pwm_off, T, 'linear', 'extrap') + dz_pos;
        pwm_full(i) = 1500 + min(OFFSET_FW_MAX, max(OFFSET_FW_MIN, raw_off));
    else
        raw_off = interp1(neg_thrust_g, neg_pwm_off, T, 'linear', 'extrap') + dz_neg;
        pwm_full(i) = 1500 + min(OFFSET_FW_MAX, max(OFFSET_FW_MIN, raw_off));
    end
end

plot(thrust_full, pwm_full, 'k-', 'LineWidth', 1.8);
hold on;
% 死区边界 (PWM)
yline(1500 + dz_pos, 'b--', 'LineWidth', 0.8);      % 正向死区 1535
yline(1500 + dz_neg, 'r--', 'LineWidth', 0.8);      % 反向死区 1485
yline(1500, 'k:', 'LineWidth', 0.8, 'HandleVisibility', 'off');
% 硬件限幅 (1050-1950, offset ±450)
yline(1500 - OFFSET_MAX, 'm:', 'LineWidth', 0.8);   % 1050
yline(1500 + OFFSET_MAX, 'm:', 'LineWidth', 0.8);   % 1950
% 固件限幅 (1000-2000, offset ±500) — 虚线
yline(1000, 'g:', 'LineWidth', 0.5);
yline(2000, 'g:', 'LineWidth', 0.5);
xlabel('推力 (CAN-g)');
ylabel('PWM 占空比 (us)');
title('完整 PWM 输出（死区 + 硬限幅 ±450 + 固件限幅 ±500）');
legend({'推力→PWM', '死区 1535', '死区 1485', ...
    '硬限幅 1050/1950', '固件限幅 1000/2000'}, 'Location', 'southeast');
grid on;

sgtitle('推进器静态标定：单轴推力 → 推进器 PWM 映射（文档口径）');

%% =========================================================================
% 第四节：数据划分
%   mode=4 自控 → 仅记录平均动力功率
%   mode=1 手控 → 按 TX/TY/MZ 通道划分单通道/多通道工况段
%   目的：为后续时域响应分析准备分段数据
% =========================================================================

% --- 动力功率（需引用第二节中已对齐的功率） ---
% 注意: power_aligned / control_power 在第二节已计算
% 若第二节未执行到此处（如分节运行），就地重新计算
if ~exist('power_aligned', 'var') || ~exist('control_power', 'var')
    thrust_total = vecnorm(force_cmd, 2, 2);
    thrust_norm = thrust_total / 10000;
    max_lag_samples = round(3 * 2);
    [c, lags] = xcorr(thrust_norm, power, max_lag_samples, 'coeff');
    [~, idx] = max(c);
    lag_samples = lags(idx);
    if lag_samples > 0
        power_aligned = [power(1+lag_samples:end); NaN(lag_samples, 1)];
    else
        power_aligned = power;
    end
    control_power = min(power_aligned);
end
propulsion_power_aligned = power_aligned - control_power;

% --- mode=4 自控：仅记录平均动力功率 ---
mode4_mask = (mode == 4);
mode4_power = propulsion_power_aligned(mode4_mask & isfinite(propulsion_power_aligned));
fprintf('\n===== 第四节：数据划分 =====\n');
fprintf('mode=4 (自控) 平均动力功率: %.1f W (样本数=%d)\n', ...
    mean(mode4_power), sum(mode4_mask));

% --- mode=1 手控：按通道划分 ---
mode1_mask = (mode == 1);
fprintf('mode=1 (手控) 样本数: %d (%.1f%%)\n', ...
    sum(mode1_mask), sum(mode1_mask)/length(mode)*100);

% 活跃通道判定阈值（CAN-g 绝对值）
ACTIVE_THRESHOLD = 500;   % 5% of 10000

% 各通道活跃标志
tx_active = abs(force_cmd(:, 1)) > ACTIVE_THRESHOLD;
ty_active = abs(force_cmd(:, 2)) > ACTIVE_THRESHOLD;
mz_active = abs(force_cmd(:, 6)) > ACTIVE_THRESHOLD;

% 工况分类（优先级: 组合 > 单通道）
%   0: 无显著输入  1: TX only  2: TY only  3: MZ only
%   4: TX+TY       5: TX+MZ    6: TY+MZ    7: TX+TY+MZ
condition_label = zeros(size(t));
condition_label(tx_active & ~ty_active & ~mz_active) = 1;
condition_label(~tx_active & ty_active & ~mz_active) = 2;
condition_label(~tx_active & ~ty_active & mz_active) = 3;
condition_label(tx_active & ty_active & ~mz_active) = 4;
condition_label(tx_active & ~ty_active & mz_active) = 5;
condition_label(~tx_active & ty_active & mz_active) = 6;
condition_label(tx_active & ty_active & mz_active) = 7;

% 仅保留 mode=1 的划分，其余置 0
condition_label(~mode1_mask) = 0;

cond_names = {'无/非手控', 'TX 前进后退', 'TY 左移右移', 'MZ 旋转', ...
              'TX+TY', 'TX+MZ', 'TY+MZ', 'TX+TY+MZ'};
cond_colors = [0.8 0.8 0.8;   % 灰 — 无
               0.0 0.4 1.0;   % 蓝 — TX
               1.0 0.3 0.0;   % 橙 — TY
               0.0 0.7 0.3;   % 绿 — MZ
               0.6 0.0 0.8;   % 紫 — TX+TY
               0.0 0.7 0.7;   % 青 — TX+MZ
               1.0 0.5 0.0;   % 棕 — TY+MZ
               1.0 0.0 0.3];  % 红 — TX+TY+MZ

% 统计各工况
fprintf('\n工况分布 (mode=1):\n');
for c = 1:7
    n_c = sum(condition_label == c);
    if n_c > 0
        fprintf('  %s: %d 样本 (%.1f%%)\n', cond_names{c+1}, n_c, n_c/sum(mode1_mask)*100);
    end
end

% 各工况的主导速度通道映射（用于稳态判断）
%   cond 1(TX)->u,  2(TY)->v,  3(MZ)->r
%   cond 4(TX+TY)->[u,v], 5(TX+MZ)->[u,r], 6(TY+MZ)->[v,r], 7(TX+TY+MZ)->[u,v,r]
cond_vel_cols = {1, 2, 6, [1 2], [1 6], [2 6], [1 2 6]};
cond_vel_names = {'u', 'v', 'r', 'u,v', 'u,r', 'v,r', 'u,v,r'};

% 稳态判据参数
STEADY_WINDOW_S = 2.0;        % 取段末 N 秒作为稳态检查窗口
STEADY_BAND_PCT = 0.20;       % +/-10% 稳态带
MIN_SEG_DURATION_S = 3.0;     % 段最短持续时长（至少 > 稳态窗口）

% 找出各工况的连续段（最短 3s，确保有足够数据做稳态判断）
min_seg_samples = round(MIN_SEG_DURATION_S * FS);      % @ 8.3Hz
steady_window_samples = round(STEADY_WINDOW_S * FS);

segments_all = struct('cond', {}, 't_start', {}, 't_end', {}, ...
    'idx_start', {}, 'idx_end', {}, 'n_samples', {}, ...
    'duration_s', {}, 'has_steady', {}, 'vel_final', {}, 'vel_band', {});

for c = 1:7
    bin = (condition_label == c);
    d_bin = diff([0; bin; 0]);
    s_idx = find(d_bin == 1);
    e_idx = find(d_bin == -1) - 1;
    for k = 1:length(s_idx)
        n_samp = e_idx(k) - s_idx(k) + 1;
        dur = n_samp / FS;  % 秒 @ 8.3Hz
        if n_samp >= min_seg_samples
            segments_all(end+1) = struct('cond', c, ...
                't_start', t(s_idx(k)), 't_end', t(e_idx(k)), ...
                'idx_start', s_idx(k), 'idx_end', e_idx(k), ...
                'n_samples', n_samp, 'duration_s', dur, ...
                'has_steady', false, 'vel_final', [], 'vel_band', []);
        end
    end
end

% --- 稳态判据 ---
fprintf('\n稳态判据: 段末 %.0fs 内速度波动 < +/-%.0f%% 稳态带\n', ...
    STEADY_WINDOW_S, STEADY_BAND_PCT * 100);
fprintf('最小段长: >= %.0fs (%d 样本 @ %.1fHz)\n', MIN_SEG_DURATION_S, min_seg_samples, FS);
fprintf('%-4s %-12s %7s %7s %8s %12s %s\n', 'ID', '工况', '起始s', '结束s', '时长s', '稳态?', '末段速度(+/-带)');

for i = 1:length(segments_all)
    seg = segments_all(i);
    idx = seg.idx_start:seg.idx_end;
    c = seg.cond;

    % 取段末稳态检查窗口
    n_win = min(steady_window_samples, seg.n_samples);
    idx_tail = seg.idx_end - n_win + 1 : seg.idx_end;

    % 检查该工况对应的所有速度通道
    vel_cols = cond_vel_cols{c};
    all_stable = true;
    vel_final_vals = zeros(1, length(vel_cols));
    vel_band_vals = zeros(1, length(vel_cols));

    for j = 1:length(vel_cols)
        vc = vel_cols(j);
        v_tail = vel(idx_tail, vc);
        v_mean = mean(v_tail, 'omitnan');
        vel_final_vals(j) = v_mean;

        if abs(v_mean) < 0.01
            % 速度接近零，不认为有稳态（无有效激励）
            all_stable = false;
            vel_band_vals(j) = 0.02;
        else
            % 检查波动是否超出 +/-10% 带
            deviation = max(abs(v_tail - v_mean));
            band = STEADY_BAND_PCT * abs(v_mean);
            vel_band_vals(j) = band;
            if deviation > band
                all_stable = false;
            end
        end
    end

    seg.has_steady = all_stable;
    seg.vel_final = vel_final_vals;
    seg.vel_band = vel_band_vals;
    segments_all(i) = seg;

    % 打印稳态判断结果
    vel_str = '';
    for j = 1:length(vel_cols)
        vc = vel_cols(j);
        if vc == 6  % r 用 deg/s 显示
            vel_str = [vel_str sprintf('%s=%.1f deg/s(+/-%.1f) ', ...
                cond_vel_names{c}(j), rad2deg(vel_final_vals(j)), rad2deg(vel_band_vals(j)))];
        else
            vel_str = [vel_str sprintf('%s=%.3f(+/-%.3f) ', ...
                cond_vel_names{c}(j), vel_final_vals(j), vel_band_vals(j))];
        end
    end
    fprintf('%-4d %-12s %7.1f %7.1f %8.1f %12s %s\n', ...
        i, cond_names{c+1}, seg.t_start, seg.t_end, seg.duration_s, ...
        cond_steady_str(seg.has_steady), vel_str);
end

% 筛选出有稳态的段
segments = segments_all([segments_all.has_steady]);
fprintf('\n有效稳态段: %d 段 / 总计 %d 段\n', length(segments), length(segments_all));

% --- 可视化：工况划分总览 ---
figure("Name", "数据划分总览");
set(gcf, "Position", [50, 50, 1400, 800]);

% 子图1: force_cmd 三个主通道 + 稳态段标记
subplot(3, 1, 1);
plot(t, force_cmd(:, 1), 'b-', 'LineWidth', 0.8); hold on;
plot(t, force_cmd(:, 2), 'r-', 'LineWidth', 0.8);
plot(t, force_cmd(:, 6), 'g-', 'LineWidth', 0.8);
yline(ACTIVE_THRESHOLD, 'k:', 'LineWidth', 0.5);
yline(-ACTIVE_THRESHOLD, 'k:', 'LineWidth', 0.5);

% 稳态段编号和色块
for i = 1:length(segments)
    seg = segments(i);
    y_top = max(abs(force_cmd(:))) * 1.05;
    patch([seg.t_start seg.t_end seg.t_end seg.t_start], ...
          [y_top*0.92 y_top*0.92 y_top y_top], ...
          cond_colors(seg.cond+1, :), 'FaceAlpha', 0.3, 'EdgeColor', 'none');
    text((seg.t_start + seg.t_end)/2, y_top*0.96, sprintf('S%d', i), ...
        'FontSize', 7, 'HorizontalAlignment', 'center', 'FontWeight', 'bold');
end

ylabel('CAN-g');
title('force\_cmd 主通道: TX(蓝) / TY(红) / MZ(绿) -- 稳态段 S1,S2,...');
legend({'TX', 'TY', 'MZ', '+/-阈值'}, 'Location', 'best');
grid on;

% mode=4 自控区间标记
yyaxis right;
mode4_shade = double(mode4_mask);
mode4_shade(mode4_shade == 0) = NaN;
plot(t, mode4_shade * 0.5, 'k.', 'MarkerSize', 2);
ylabel('mode=4');
set(gca, 'YTick', []);
ylim([0, 1]);

% 子图2: 工况分类色带 + 稳态段竖线
subplot(3, 1, 2);
imagesc(t, 1, condition_label');
colormap(gca, cond_colors);
caxis([0, 7]);
hold on;
for i = 1:length(segments)
    xline(segments(i).t_start, 'k-', 'LineWidth', 1.2);
    xline(segments(i).t_end, 'k-', 'LineWidth', 1.2);
end
cb = colorbar;
cb.Ticks = 0:7;
cb.TickLabels = cond_names;
xlabel('时间 (s)');
title(sprintf('工况分类（稳态段 %d 段，黑色竖线）', length(segments)));
set(gca, 'YTick', []);

% 子图3: 速度响应 + 稳态段色块
subplot(3, 1, 3);
plot(t, vel(:, 1), 'b-', 'LineWidth', 0.8); hold on;
plot(t, vel(:, 2), 'r-', 'LineWidth', 0.8);
plot(t, rad2deg(vel(:, 6)), 'g-', 'LineWidth', 0.8);
for i = 1:length(segments)
    seg = segments(i);
    yl = ylim;
    patch([seg.t_start seg.t_end seg.t_end seg.t_start], ...
          [yl(1) yl(1) yl(2) yl(2)], ...
          cond_colors(seg.cond+1, :), 'FaceAlpha', 0.12, 'EdgeColor', 'none');
end
xlabel('时间 (s)');
ylabel('速度');
title('速度响应: u[m/s] / v[m/s] / r[deg/s] -- 色块=稳态段');
legend({'u', 'v', 'r (deg/s)'}, 'Location', 'best');
grid on;

sgtitle(sprintf(['数据划分总览 -- 阈值=%d CAN-g | 稳态窗口=%.0fs +/-%.0f%% | %d/%d 段有稳态'], ...
    ACTIVE_THRESHOLD, STEADY_WINDOW_S, STEADY_BAND_PCT*100, ...
    length(segments), length(segments_all)));

% --- 各稳态段阶跃响应子图（按工况类型分组，每段一个子图） ---
PAD_S = 10.0;                   % 前后额外延长秒数
PAD_SAMPLES = round(PAD_S * FS);
STEP_QUIET_THRESHOLD = 200;     % "安静"判据: |force_cmd| < 200 CAN-g
MIN_QUIET_SAMPLES = round(1.0 * FS);  % 需要至少1s连续安静

% 各工况对应的主导 force_cmd 列
fc_dominant_map = {1, 2, 6, [1 2], [1 6], [2 6], [1 2 6]};

if ~isempty(segments)
    % 统计各工况段的全局编号
    global_ids = cell(7, 1);
    for c = 1:7
        global_ids{c} = find([segments.cond] == c);
    end

    % 对每类工况分别建图
    for c = 1:7
        seg_ids = global_ids{c};
        n_c = length(seg_ids);
        if n_c == 0, continue; end

        n_cols = min(2, n_c);
        n_rows = ceil(n_c / n_cols);
        fig_h = max(300, n_rows * 280);

        figure("Name", sprintf("阶跃响应_%s", strrep(cond_names{c+1}, ' ', '_')));
        set(gcf, "Position", [100, 50, 1200, fig_h]);

        for j = 1:n_c
            subplot(n_rows, n_cols, j);
            seg = segments(seg_ids(j));
            vel_cols = cond_vel_cols{c};
            fc_cols = fc_dominant_map{c};

            % --- 向前搜索正阶跃起点 ---
            pos_step_idx = seg.idx_start;
            quiet_count = 0;
            for k = seg.idx_start:-1:2
                if all(abs(force_cmd(k, fc_cols)) < STEP_QUIET_THRESHOLD)
                    quiet_count = quiet_count + 1;
                    if quiet_count >= MIN_QUIET_SAMPLES
                        pos_step_idx = k + quiet_count;
                        break;
                    end
                else
                    quiet_count = 0;
                end
            end

            % --- 向后搜索负阶跃终点 ---
            neg_step_idx = seg.idx_end;
            quiet_count = 0;
            for k = seg.idx_end:length(t)-1
                if all(abs(force_cmd(k, fc_cols)) < STEP_QUIET_THRESHOLD)
                    quiet_count = quiet_count + 1;
                    if quiet_count >= MIN_QUIET_SAMPLES
                        neg_step_idx = k - quiet_count;
                        break;
                    end
                else
                    quiet_count = 0;
                end
            end

            % 扩展范围
            idx_start_ext = max(1, pos_step_idx - PAD_SAMPLES);
            idx_end_ext = min(length(t), neg_step_idx + PAD_SAMPLES);
            idx_ext = idx_start_ext:idx_end_ext;
            t_ext = t(idx_ext) - t(idx_ext(1));

            % --- 左轴: 速度响应 ---
            yyaxis left;
            h_lines = gobjects(0);
            leg_entries = {};
            for vj = 1:length(vel_cols)
                vc = vel_cols(vj);
                if vc == 6
                    h_lines(end+1) = plot(t_ext, rad2deg(vel(idx_ext, vc)), ...
                        'LineWidth', 1.2, 'Color', [0 0.5 0]);
                    leg_entries{end+1} = 'r (deg/s)';
                elseif vc == 1
                    h_lines(end+1) = plot(t_ext, vel(idx_ext, vc), 'b-', 'LineWidth', 1.2);
                    leg_entries{end+1} = 'u (m/s)';
                else
                    h_lines(end+1) = plot(t_ext, vel(idx_ext, vc), 'r-', 'LineWidth', 1.2);
                    leg_entries{end+1} = 'v (m/s)';
                end
            end
            hold on;

            % 正阶跃（红线）+ 负阶跃（蓝线）
            t_pos = t(pos_step_idx) - t(idx_ext(1));
            t_neg = t(neg_step_idx) - t(idx_ext(1));
            xline(t_pos, 'r-', 'LineWidth', 1.2);
            xline(t_neg, 'b-', 'LineWidth', 1.2);

            % 稳态段（浅绿）+ 稳态窗口（虚线）
            t_s1 = t(seg.idx_start) - t(idx_ext(1));
            t_s2 = t(seg.idx_end) - t(idx_ext(1));
            yl = ylim;
            patch([t_s1 t_s2 t_s2 t_s1], [yl(1) yl(1) yl(2) yl(2)], ...
                  [0.3 0.8 0.3], 'FaceAlpha', 0.10, 'EdgeColor', 'none');
            xline(t_s2 - STEADY_WINDOW_S, 'k--', 'LineWidth', 1.0);
            xline(t_s2, 'k--', 'LineWidth', 1.0);

            ylabel('速度');
            if j == 1, legend(h_lines, leg_entries, 'Location', 'best'); end

            % --- 右轴: force_cmd ---
            yyaxis right;
            fc_map = [1, 2, 6];
            fc_clr = {[0 0 1], [1 0 0], [0 0.6 0]};
            for vj = 1:length(vel_cols)
                vc = vel_cols(vj);
                fi = find(fc_map == vc);
                stairs(t_ext, force_cmd(idx_ext, vc), '-', 'LineWidth', 1.0, 'Color', fc_clr{fi});
            end
            ylabel('CAN-g');

            xlabel('时间 (s)');
            title(sprintf('S%d: %.0f-%.0fs', seg_ids(j), t(pos_step_idx), t(neg_step_idx)));
            grid on;
        end
        sgtitle(sprintf('%s (n=%d) | 红线=正阶跃 蓝线=负阶跃 | 浅绿=稳态 虚线=稳态窗 | +/-%.0fs', ...
            cond_names{c+1}, n_c, PAD_S));
    end
end

% =========================================================================
% 第五节：导出稳态段数据
% =========================================================================
csv_dir = fullfile(repo_root, 'data', 'segments_0530');
if ~exist(csv_dir, 'dir'), mkdir(csv_dir); end

fprintf('\n===== 第五节：导出稳态段数据 =====\n');
fprintf('输出目录: %s\n', csv_dir);

for i = 1:length(segments)
    seg = segments(i);
    c = seg.cond;
    fc_cols = fc_dominant_map{c};

    % 搜索正/负阶跃（与绘图一致）
    pos_step_idx = seg.idx_start;
    quiet_count = 0;
    for k = seg.idx_start:-1:2
        if all(abs(force_cmd(k, fc_cols)) < STEP_QUIET_THRESHOLD)
            quiet_count = quiet_count + 1;
            if quiet_count >= MIN_QUIET_SAMPLES
                pos_step_idx = k + quiet_count; break;
            end
        else, quiet_count = 0; end
    end
    neg_step_idx = seg.idx_end;
    quiet_count = 0;
    for k = seg.idx_end:length(t)-1
        if all(abs(force_cmd(k, fc_cols)) < STEP_QUIET_THRESHOLD)
            quiet_count = quiet_count + 1;
            if quiet_count >= MIN_QUIET_SAMPLES
                neg_step_idx = k - quiet_count; break;
            end
        else, quiet_count = 0; end
    end

    idx_start = max(1, pos_step_idx - PAD_SAMPLES);
    idx_end = min(length(t), neg_step_idx + PAD_SAMPLES);
    idx_ext = idx_start:idx_end;

    T_out = table();
    T_out.t_abs = t(idx_ext);
    T_out.t_rel = t(idx_ext) - t(idx_ext(1));
    T_out.TX = force_cmd(idx_ext, 1);
    T_out.TY = force_cmd(idx_ext, 2);
    T_out.TZ = force_cmd(idx_ext, 3);
    T_out.MX = force_cmd(idx_ext, 4);
    T_out.MY = force_cmd(idx_ext, 5);
    T_out.MZ = force_cmd(idx_ext, 6);
    T_out.u = vel(idx_ext, 1);
    T_out.v = vel(idx_ext, 2);
    T_out.w = vel(idx_ext, 3);
    T_out.r_deg_s = rad2deg(vel(idx_ext, 6));
    T_out.voltage = voltage(idx_ext);
    T_out.current = current(idx_ext);
    T_out.power = power(idx_ext);
    T_out.mode = mode(idx_ext);

    cond_short = {'TX', 'TY', 'MZ', 'TX+TY', 'TX+MZ', 'TY+MZ', 'TX+TY+MZ'};
    fname = sprintf('S%02d_%s.csv', i, cond_short{c});
    fpath = fullfile(csv_dir, fname);
    writetable(T_out, fpath);
    fprintf('  [%2d/%2d] %s (%d 行, %.1f-%.1fs)\n', ...
        i, length(segments), fname, height(T_out), t(idx_start), t(idx_end));
end
fprintf('导出完成: %d 个 CSV -> %s\n', length(segments), csv_dir);


%% 局部函数

function s = cond_steady_str(flag)
%COND_STEADY_STR 稳态标志 -> 显示字符串
if flag
    s = 'V 稳态';
else
    s = 'x';
end
end

function C = corr_safe(A, B)
%CORR_SAFE 计算相关系数，自动去除非有限样本。
C = nan(size(A, 2), size(B, 2));
for i = 1:size(A, 2)
    for j = 1:size(B, 2)
        mask = isfinite(A(:, i)) & isfinite(B(:, j));
        if sum(mask) >= 5 && std(A(mask, i)) > 0 && std(B(mask, j)) > 0
            cij = corrcoef(A(mask, i), B(mask, j));
            C(i, j) = cij(1, 2);
        end
    end
end
end

function [pwm_us, thrust_g] = force_cmd_to_pwm_batch(force_cmd, alloc_params)
%FORCE_CMD_TO_PWM_BATCH 批量复现 force/moment -> 推进器 CAN-g -> PWM 的静态映射。
[~, info0] = xhy_force_moment_to_pwm(zeros(6, 1), alloc_params);
params = info0.params;

alloc_cmd = force_cmd;

K = [
    0,   0,  -0.5, 0,  0.83,  0;
    0,   0,  -0.5, 0, -0.83,  0;
    0,  -0.5, 0,   0,  0,    -0.83;
    0,  -0.5, 0,   0,  0,     0.83;
    1.0, 0,   0,   0,  0,       0
    ];

thrust_raw_g = alloc_cmd * K';
ratio = zeros(size(thrust_raw_g));
for i = 1:5
    pos = thrust_raw_g(:, i) >= 0;
    ratio(pos, i) = abs(thrust_raw_g(pos, i)) ./ params.thrust_limit_pos_g(i);
    ratio(~pos, i) = abs(thrust_raw_g(~pos, i)) ./ params.thrust_limit_neg_g(i);
end
scale_factor = max(1, max(ratio, [], 2));
thrust_g = thrust_raw_g ./ scale_factor;

pwm_offset = zeros(size(thrust_g));
for i = 1:5
    Ti = thrust_g(:, i);
    pos = Ti > 0;
    neg = Ti < 0;
    pwm_offset(pos, i) = interp1(params.pos_thrust_g, params.pos_pwm, Ti(pos), 'linear', 'extrap') + params.deadzone_pos(i);
    pwm_offset(neg, i) = interp1(params.neg_thrust_g, params.neg_pwm, Ti(neg), 'linear', 'extrap') + params.deadzone_neg(i);
end
pwm_offset = min(params.pwm_max, max(-params.pwm_max, pwm_offset));
pwm_us = params.pwm_center_us + pwm_offset;
end

function r = safe_ratio(a, b)
%SAFE_RATIO 计算比例，避开分母过小的点。
mask = isfinite(a) & isfinite(b) & abs(b) > 1e-6;
r = a(mask) ./ b(mask);
end

function [acc, vel_smooth] = estimate_acceleration(t, vel)
%ESTIMATE_ACCELERATION 对速度做轻度平滑后估计加速度。
vel_smooth = movmedian(vel, 9, 1, 'omitnan');
vel_smooth = movmean(vel_smooth, 9, 1, 'omitnan');
acc = zeros(size(vel_smooth));
for i = 1:size(vel_smooth, 2)
    acc(:, i) = gradient(vel_smooth(:, i), t);
end
bad = ~isfinite(acc);
acc(bad) = 0;
end

function result = fit_drag_channel(x1, x2, y)
%FIT_DRAG_CHANNEL 拟合 y = d1*nu + d2*abs(nu)*nu + bias，使用简单 Huber IRLS。
result = struct('d1', nan, 'd2', nan, 'bias', nan, 'rmse', nan, 'n', numel(y));
if numel(y) < 50
    return;
end

X = [x1(:), x2(:), ones(numel(y), 1)];
y = y(:);
mask = all(isfinite(X), 2) & isfinite(y);
X = X(mask, :);
y = y(mask);
if size(X, 1) < 50 || rank(X) < 2
    result.n = size(X, 1);
    return;
end

w = ones(size(y));
beta = X \ y;
for iter = 1:20
    r = y - X * beta;
    sigma = 1.4826 * median(abs(r - median(r))) + eps;
    c = 1.345 * sigma;
    w = min(1, c ./ max(abs(r), eps));
    Xw = X .* sqrt(w);
    yw = y .* sqrt(w);
    beta = Xw \ yw;
end

resid = y - X * beta;
result.d1 = beta(1);
result.d2 = beta(2);
result.bias = beta(3);
result.rmse = sqrt(mean(resid .^ 2));
result.n = numel(y);
end

function make_diagnostic_figure(t, force_cmd, tau_fault, vel, thrust_n, pwm_us, mode, fig_path)
%MAKE_DIAGNOSTIC_FIGURE 输出复分析诊断图。
f = figure('Visible', 'off', 'Position', [100 100 1500 1000]);

subplot(3, 2, 1);
plot(t, force_cmd(:, [1 2 3 5 6]));
grid on; xlabel('t (s)'); ylabel('force\_cmd');
title('原始 force\_cmd 输入');
legend('TX', 'TY', 'TZ', 'MY', 'MZ', 'Location', 'best');

subplot(3, 2, 2);
plot(t, tau_fault(:, [1 2 3 5 6]));
grid on; xlabel('t (s)'); ylabel('N / N*m');
title('T1 故障后的模型有效 5DOF 输出');
legend('X', 'Y', 'Z', 'M', 'N', 'Location', 'best');

subplot(3, 2, 3);
plot(t, vel(:, [1 2 3]));
grid on; xlabel('t (s)'); ylabel('m/s');
title('线速度响应');
legend('u', 'v', 'w', 'Location', 'best');

subplot(3, 2, 4);
plot(t, rad2deg(vel(:, [4 5 6])));
grid on; xlabel('t (s)'); ylabel('deg/s');
title('角速度响应');
legend('p', 'q', 'r', 'Location', 'best');

subplot(3, 2, 5);
plot(t, thrust_n);
grid on; xlabel('t (s)'); ylabel('N');
title('M080/M060 模型推力');
legend('T1', 'T2', 'T3', 'T4', 'T5', 'Location', 'best');

subplot(3, 2, 6);
plot(t, pwm_us - 1500);
hold on;
stairs(t, 80 * (mode == 4), 'k:');
grid on; xlabel('t (s)'); ylabel('PWM offset (us)');
title('PWM 偏置与 mode=4 标记');
legend('T1', 'T2', 'T3', 'T4', 'T5', 'mode4*80', 'Location', 'best');

sgtitle('2026-05-30 水池 debug 数据复分析：force\_cmd -> PWM -> M080/M060 -> tau');
saveas(f, fig_path);
close(f);
end

function write_markdown_report(report_path, data_file, fig_path, N, t, mode, voltage, current, power, ...
    force_cmd, pwm_us, thrust_doc_g, thrust_model_n, tau_nominal, tau_fault, ...
    heave_loss, pitch_delta, t1_active, vertical_active, fault_ratio_mz, ...
    corr_cmd_vel, corr_tau_vel, corr_tau_vel_mode1, fit_results, dof_names, vel_names, thruster_names, x_vert_r)
%WRITE_MARKDOWN_REPORT 写出 Obsidian 可直接追加的 Markdown 复分析报告。
fid = fopen(report_path, 'w', 'n', 'UTF-8');
if fid < 0
    error('无法写入报告: %s', report_path);
end
cleanup = onCleanup(@() fclose(fid));

fprintf(fid, '# 2026-05-30 debug 数据复分析（2026-06-07）\n\n');
fprintf(fid, '## 复分析口径\n\n');
fprintf(fid, '- 数据文件：`%s`\n', rel_path(data_file));
fprintf(fid, '- 分析程序：`analyze_pool_test_20260530_reanalysis.m`\n');
fprintf(fid, '- 新链路：`force_cmd/CAN-g 原始输入 -> 推力分配 -> PWM -> M080/M060(24V 实测电压) -> 5 推进器推力 -> 6DOF 有效力/力矩`。\n');
fprintf(fid, '- 本次将 0530 已知故障显式建模为 **T1 前垂推实际输出为 0**，并与名义双垂推输出对比。\n');
fprintf(fid, '- `force_cmd4-6` 按原始力矩输入处理，进入分配矩阵前 `moment_scale=1`，不再按物理 N*m 二次乘 100。\n\n');

fprintf(fid, '## 数据概况\n\n');
fprintf(fid, '| 项目 | 数值 |\n|---|---:|\n');
fprintf(fid, '| 样本数 | %d |\n', N);
fprintf(fid, '| 时长 | %.1f s / %.2f min |\n', t(end) - t(1), (t(end) - t(1)) / 60);
fprintf(fid, '| 控制电压 | %.2f - %.2f V，均值 %.2f V |\n', min(voltage), max(voltage), mean(voltage));
fprintf(fid, '| 电流 | %.2f - %.2f A，均值 %.2f A |\n', min(current), max(current), mean(current));
fprintf(fid, '| 功率 | %.1f - %.1f W，均值 %.1f W |\n', min(power), max(power), mean(power));
mode_values = unique(mode);
for i = 1:numel(mode_values)
    mi = mode_values(i);
    fprintf(fid, '| mode=%g 样本 | %d / %.1f%% |\n', mi, sum(mode == mi), sum(mode == mi) / N * 100);
end
fprintf(fid, '\n');

fprintf(fid, '## force_cmd 与 PWM/推力统计\n\n');
fprintf(fid, '| 通道 | mean | std | min | max | 非零比例 |\n|---|---:|---:|---:|---:|---:|\n');
for i = 1:6
    x = force_cmd(:, i);
    fprintf(fid, '| %s | %.2f | %.2f | %.2f | %.2f | %.1f%% |\n', ...
        dof_names{i}, mean(x), std(x), min(x), max(x), mean(abs(x) > 1e-6) * 100);
end
fprintf(fid, '\n');

fprintf(fid, '| 推进器 | PWM offset RMS (us) | 文档分配 RMS (g) | M080/M060 推力 RMS (N) | 推力范围 (N) |\n|---|---:|---:|---:|---:|\n');
for i = 1:5
    pwm_offset = pwm_us(:, i) - 1500;
    thrust_i = thrust_model_n(:, i);
    fprintf(fid, '| %s | %.1f | %.1f | %.3f | %.3f 到 %.3f |\n', ...
        thruster_names{i}, rms_local(pwm_offset), rms_local(thrust_doc_g(:, i)), ...
        rms_local(thrust_i), min(thrust_i), max(thrust_i));
end
fprintf(fid, '\n');

fprintf(fid, '## T1 故障影响\n\n');
fprintf(fid, '| 指标 | 数值 |\n|---|---:|\n');
fprintf(fid, '| T1 有指令样本比例（|T1_doc|>100g） | %.1f%% |\n', mean(t1_active) * 100);
fprintf(fid, '| T1 有指令时名义 T1 推力 RMS | %.3f N |\n', rms_local(thrust_model_n(t1_active, 1)));
fprintf(fid, '| T1 故障造成的 Z 输出损失 RMS | %.3f N |\n', rms_local(heave_loss(t1_active)));
fprintf(fid, '| T1 故障造成的 M 输出变化 RMS | %.3f N*m |\n', rms_local(pitch_delta(t1_active)));
fprintf(fid, '| 垂向活跃段有效 M/Z 中位数 | %.3f m |\n', median(fault_ratio_mz, 'omitnan'));
fprintf(fid, '| 垂向活跃段有效 M/Z 均值 | %.3f m |\n', mean(fault_ratio_mz, 'omitnan'));
fprintf(fid, '\n');
fprintf(fid, '解释：T1 前垂推断开后，T2 后垂推仍在工作，因此任意有效垂向输出都会伴随约 `x_vert_r=%.3f m` 量级的俯仰力矩臂。', x_vert_r);
fprintf(fid, '这会让 Z/M 通道从可解耦的双垂推控制退化为单垂推耦合控制。\n\n');

fprintf(fid, '## 相关性诊断\n\n');
write_corr_table(fid, '原始 force_cmd vs 速度', corr_cmd_vel, dof_names, vel_names);
write_corr_table(fid, 'T1 故障后有效 tau vs 速度（全数据）', corr_tau_vel, dof_names, vel_names);
write_corr_table(fid, 'T1 故障后有效 tau vs 速度（mode=1）', corr_tau_vel_mode1, dof_names, vel_names);

fprintf(fid, '## 灰箱阻尼拟合诊断\n\n');
fprintf(fid, '拟合形式：`tau_i - M_ii * nudot_i = d1_i * nu_i + d2_i * |nu_i| * nu_i + bias_i`。该结果用于定位数量级和符号风险，不直接替代水池阶跃/CFD 标定。\n\n');
fprintf(fid, '| 自由度 | 样本数 | d1 | d2 | bias | RMSE |\n|---|---:|---:|---:|---:|---:|\n');
for i = 1:6
    r = fit_results{i};
    fprintf(fid, '| %s | %d | %.4g | %.4g | %.4g | %.4g |\n', ...
        dof_names{i}, r.n, r.d1, r.d2, r.bias, r.rmse);
end
fprintf(fid, '\n');

fprintf(fid, '## 复分析结论\n\n');
fprintf(fid, '1. 0530 数据应按“原始 force_cmd -> PWM -> 推进器模型 -> 有效 tau”分析，而不是把 force_cmd 直接当作物理 N/N*m。\n');
fprintf(fid, '2. T1 前垂推故障是 0530 Z/M 通道异常的主因：名义分配仍给 T1 指令，但实际有效输出缺失，使垂向力损失并引入固定力臂俯仰耦合。\n');
fprintf(fid, '3. 用当前 M080/M060 模型换算后，有效力/力矩数量级远小于早期按 `8000g≈78N` 直接解释的口径；这解释了旧模型稳态速度数量级偏大的主要来源。\n');
fprintf(fid, '4. 0530 数据不适合直接标定完整 6DOF 阻尼矩阵：故障导致 Z/M 不可解耦，机动段多通道同时激励，灰箱拟合只能作为风险定位。\n');
fprintf(fid, '5. 可用于模型校正的较可信通道是主推 X、侧推 Y/N；Z/M 应以 0601 T1 修复后的水池数据为主。\n\n');

fprintf(fid, '## 输出文件\n\n');
fprintf(fid, '- 诊断图：`%s`\n', rel_path(fig_path));
fprintf(fid, '- Markdown 报告：`%s`\n', rel_path(report_path));
end

function write_corr_table(fid, title_text, C, row_names, col_names)
%WRITE_CORR_TABLE 写出相关系数表。
fprintf(fid, '### %s\n\n', title_text);
fprintf(fid, '| 输入/输出 |');
for j = 1:numel(col_names)
    fprintf(fid, ' %s |', col_names{j});
end
fprintf(fid, '\n|---|');
for j = 1:numel(col_names)
    fprintf(fid, '---:|');
end
fprintf(fid, '\n');
for i = 1:numel(row_names)
    fprintf(fid, '| %s |', row_names{i});
    for j = 1:numel(col_names)
        fprintf(fid, ' %.3f |', C(i, j));
    end
    fprintf(fid, '\n');
end
fprintf(fid, '\n');
end

function y = rms_local(x)
%RMS_LOCAL 不依赖工具箱的 RMS。
x = x(isfinite(x));
if isempty(x)
    y = nan;
else
    y = sqrt(mean(x .^ 2));
end
end

function p = rel_path(p)
%REL_PATH 把绝对路径压缩成仓库相对路径，便于 Obsidian 阅读。
[~, name, ext] = fileparts(p);
parts = split(string(p), filesep);
idx = find(parts == "research", 1, 'last');
if ~isempty(idx) && idx < numel(parts)
    p = strjoin(parts(idx+1:end), '/');
else
    p = [name ext];
end
end
