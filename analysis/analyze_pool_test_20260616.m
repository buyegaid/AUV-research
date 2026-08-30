% analyze_pool_test_20260616.m
% 读取 0616 水池实验数据（已预分段），统一输出为标准分段 CSV 格式。
% 输出到 data/segments_0616/，兼容 analyze_x/y/r 直接读取。

clear; clc;

repo_root = setup_paths();

data_file = fullfile(repo_root, 'data', 'csv', 'auv_data_merged.csv');
seg_file  = fullfile(repo_root, 'data', 'csv', 'auv_segments_final.csv');
out_dir   = fullfile(repo_root, 'data', 'segments_0616');
if ~exist(out_dir, 'dir'), mkdir(out_dir); end

% 读取合并数据
fprintf('读取合并数据: %s\n', data_file);
T = readtable(data_file);
fprintf('  总行数: %d\n', height(T));

% 读取预分段信息
fprintf('读取分段信息: %s\n', seg_file);
S = readtable(seg_file);
n_segs = height(S);
fprintf('  分段数: %d\n', n_segs);

% 通道映射
channels = unique(S.channel);
fprintf('  通道: %s\n', strjoin(channels, ', '));

% 构建时间列
t_abs = T.stamp_time;
t_abs = t_abs - t_abs(1);  % 从 0 开始

% 力/力矩列 (CAN-g)
force_cmd_all = [T.motor_TX, T.motor_TY, T.motor_TZ, ...
                 T.motor_MX, T.motor_MY, T.motor_MZ];

% 速度
u_all = T.linear_vx;
v_all = T.linear_vy;
w_all = T.linear_vz;
r_all = T.angular_wz;  % 0616 格式中是 deg/s

% 功率（控制+推进分开记录）
voltage_all = T.ctrl_voltage;
current_all = T.ctrl_current;
power_all = T.prop_pwr_power + T.ctrl_pwr_power;

% 模式
mode_all = T.control_mode;

fprintf('\n===== 导出分段 CSV =====\n');

for i = 1:n_segs
    ch = S.channel{i};
    seg_id = S.seg_id(i);
    idx_s = S.idx_pad_start(i) + 1;  % Python 0-index → MATLAB 1-index
    idx_e = S.idx_pad_end(i) + 1;

    % 钳位
    idx_s = max(1, idx_s);
    idx_e = min(height(T), idx_e);
    idx_range = idx_s:idx_e;

    % 构建输出表
    T_out = table();
    T_out.t_abs = t_abs(idx_range);
    T_out.t_rel = t_abs(idx_range) - t_abs(idx_s);
    T_out.TX = force_cmd_all(idx_range, 1);
    T_out.TY = force_cmd_all(idx_range, 2);
    T_out.TZ = force_cmd_all(idx_range, 3);
    T_out.MX = force_cmd_all(idx_range, 4);
    T_out.MY = force_cmd_all(idx_range, 5);
    T_out.MZ = force_cmd_all(idx_range, 6);
    T_out.u = u_all(idx_range);
    T_out.v = v_all(idx_range);
    T_out.w = w_all(idx_range);
    T_out.r_deg_s = r_all(idx_range);
    T_out.voltage = voltage_all(idx_range);
    T_out.current = current_all(idx_range);
    T_out.power = power_all(idx_range);
    T_out.mode = mode_all(idx_range);

    % 文件名（与 0530/0601 格式统一: Sxx_XX.csv）
    fname = sprintf('S%02d_%s.csv', seg_id, ch);
    fpath = fullfile(out_dir, fname);
    writetable(T_out, fpath);
    fprintf('  [%2d/%2d] %s (%d 行, %.1f-%.1fs, %s)\n', ...
        i, n_segs, fname, height(T_out), ...
        t_abs(idx_s), t_abs(idx_e), S.amp_pct{i});
end

fprintf('导出完成: %d 个 CSV -> %s\n', n_segs, out_dir);
