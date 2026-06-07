% analyze_pool_test_20260530_reanalysis.m
% 使用当前 force_cmd -> PWM -> M080/M060 推进器模型链路复分析 2026-05-30 水池 debug 数据。
%
% 说明：
%   1. debug_data260530-1.csv 中 force_cmd 是下发到 CAN 分配逻辑的原始输入。
%   2. 平动力 TX/TY/TZ 按 CAN-g 输入处理。
%   3. 转动力矩 MX/MY/MZ 按原始力矩输入处理，进入分配矩阵前不再额外乘 100。
%   4. 推进器顺序采用文档顺序 [T1 T2 T3 T4 T5]，动力学矩阵顺序采用 [T5 T1 T2 T3 T4]。

clear; clc;

project_root = setup_paths();
repo_root = project_root;

data_file = fullfile(repo_root, 'data', 'raw', 'debug_data260530-1.csv');
out_dir = fullfile(repo_root, 'results');
pic_dir = fullfile(repo_root, 'assets', 'figures');
if ~exist(out_dir, 'dir'), mkdir(out_dir); end
if ~exist(pic_dir, 'dir'), mkdir(pic_dir); end

T = readtable(data_file);

t = T.pc_timestamp - T.pc_timestamp(1);
force_cmd = [T.force_cmd1, T.force_cmd2, T.force_cmd3, T.force_cmd4, T.force_cmd5, T.force_cmd6];
voltage = T.control_voltage;
current = T.power_current;
power = voltage .* current;
mode = T.mode;

vel_linear = [T.linear_vel_x, T.linear_vel_y, T.linear_vel_z];
vel_angular_deg = [T.angular_vel_x, T.angular_vel_y, T.angular_vel_z];
vel = [vel_linear, deg2rad(vel_angular_deg)];

dof_names = {'Surge X', 'Sway Y', 'Heave Z', 'Roll K', 'Pitch M', 'Yaw N'};
vel_names = {'u', 'v', 'w', 'p', 'q', 'r'};
thruster_names = {'T1 前垂推', 'T2 后垂推', 'T3 前侧推', 'T4 后侧推', 'T5 主推'};

% 当前 XHY 几何，与 model/xhy.m 保持一致。
x_vert_f = +0.344;
x_vert_r = -0.293;
x_side_f = +0.424;
x_side_r = -0.376;
B_thr = [
    1,  0,          0,          0,        0;
    0,  0,          0,          1,        1;
    0,  1,          1,          0,        0;
    0,  0,          0,          0,        0;
    0, -x_vert_f,  -x_vert_r,   0,        0;
    0,  0,          0,          x_side_f, x_side_r
    ];

% 使用当前质量矩阵对角线做局部灰箱检查。
M_diag = [101.642, 210.562, 128.702, 1.43314, 7.21453, 6.51361]';

alloc_params = struct();
alloc_params.moment_scale = 1;       % debug 中的力矩列已经是原始分配输入，不再按物理 N*m 放大。
alloc_params.enable_ramp = false;    % 离线分析使用每个采样点的静态映射。

m060_params = m060_thruster_params();
m080_params = m080_thruster_params();

N = height(T);
pwm_us = zeros(N, 5);
thrust_doc_g = zeros(N, 5);
thrust_model_n = zeros(N, 5);
tau_nominal = zeros(N, 6);
tau_t1_fault = zeros(N, 6);

% 批量复现 xhy_force_moment_to_pwm 的分配和 PWM 插值，避免逐点调用导致离线分析过慢。
[pwm_us, thrust_doc_g] = force_cmd_to_pwm_batch(force_cmd, alloc_params);

s1 = m060_thruster_model(pwm_us(:, 1), voltage, m060_params);
s2 = m060_thruster_model(pwm_us(:, 2), voltage, m060_params);
s3 = m060_thruster_model(pwm_us(:, 3), voltage, m060_params);
s4 = m060_thruster_model(pwm_us(:, 4), voltage, m060_params);
s5 = m080_thruster_model(pwm_us(:, 5), voltage, m080_params);
thrust_model_n = [s1.thrust_n, s2.thrust_n, s3.thrust_n, s4.thrust_n, s5.thrust_n];

T_vec_nominal = [s5.thrust_n, s1.thrust_n, s2.thrust_n, s3.thrust_n, s4.thrust_n];
T_vec_fault = [s5.thrust_n, zeros(N, 1), s2.thrust_n, s3.thrust_n, s4.thrust_n];
tau_nominal = (B_thr * T_vec_nominal')';
tau_t1_fault = (B_thr * T_vec_fault')';

valid = all(isfinite([force_cmd, vel, tau_t1_fault]), 2) & isfinite(t) & isfinite(voltage) & isfinite(current);
mode1 = valid & mode == 1;
active_tau = valid & vecnorm(tau_t1_fault(:, [1 2 3 5 6]), 2, 2) > 0.25;
active_mode1 = mode1 & active_tau;

corr_cmd_vel = corr_safe(force_cmd(valid, :), vel(valid, :));
corr_tau_vel = corr_safe(tau_t1_fault(valid, :), vel(valid, :));
corr_tau_vel_mode1 = corr_safe(tau_t1_fault(mode1, :), vel(mode1, :));

% T1 故障导致的垂向/俯仰差异。
t1_active = valid & abs(thrust_doc_g(:, 1)) > 100;
vertical_active = valid & (abs(force_cmd(:, 3)) > 500 | abs(force_cmd(:, 5)) > 500);
heave_loss = tau_nominal(:, 3) - tau_t1_fault(:, 3);
pitch_delta = tau_nominal(:, 5) - tau_t1_fault(:, 5);
fault_ratio_mz = safe_ratio(tau_t1_fault(vertical_active, 5), tau_t1_fault(vertical_active, 3));

% 用当前动力学质量矩阵做灰箱阻尼检查。该结果只用于定位数量级/符号风险，不直接作为最终标定。
fit_mask = active_mode1;
[acc, vel_smooth] = estimate_acceleration(t, vel);
fit_results = cell(6, 1);
for i = 1:6
    y = tau_t1_fault(:, i) - M_diag(i) .* acc(:, i);
    x1 = vel_smooth(:, i);
    x2 = abs(vel_smooth(:, i)) .* vel_smooth(:, i);
    mask_i = fit_mask & isfinite(y) & isfinite(x1) & isfinite(x2) & abs(x1) > 1e-4;
    fit_results{i} = fit_drag_channel(x1(mask_i), x2(mask_i), y(mask_i));
end

% 生成诊断图。
fig_path = fullfile(pic_dir, 'pool_test_20260530_reanalysis_matlab.png');
make_diagnostic_figure(t, force_cmd, tau_t1_fault, vel, thrust_model_n, pwm_us, mode, fig_path);

% 生成 Markdown 报告。
report_path = fullfile(out_dir, 'pool_test_20260530_reanalysis.md');
write_markdown_report(report_path, data_file, fig_path, N, t, mode, voltage, current, power, ...
    force_cmd, pwm_us, thrust_doc_g, thrust_model_n, tau_nominal, tau_t1_fault, ...
    heave_loss, pitch_delta, t1_active, vertical_active, fault_ratio_mz, ...
    corr_cmd_vel, corr_tau_vel, corr_tau_vel_mode1, fit_results, dof_names, vel_names, thruster_names, x_vert_r);

fprintf('复分析完成。\n');
fprintf('报告: %s\n', report_path);
fprintf('图像: %s\n', fig_path);

%% 局部函数

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
alloc_cmd(:, 4:6) = alloc_cmd(:, 4:6) .* params.moment_scale;

K = [
    0,   0,  -0.5, 0,  0.0083,  0;
    0,   0,  -0.5, 0, -0.0083,  0;
    0,  -0.5, 0,   0,  0,      -0.0083;
    0,  -0.5, 0,   0,  0,       0.0083;
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
