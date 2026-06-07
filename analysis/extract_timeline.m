% extract_timeline.m — 提取测试时间线，用于完善水池实验记录
clear; clc;
project_root = setup_paths();

load('results/pool_test_20260601_results.mat', 'merged_segments', 'type_names');

T = readtable('data/raw/debug_data260601-1.csv');
t_raw = T.pc_timestamp - T.pc_timestamp(1);
N_raw = length(t_raw);
dt_raw = median(diff(t_raw));

% CAN矩阵
K_thr = [0,0,-0.5,0,0.0083,0; 0,0,-0.5,0,-0.0083,0; 0,-0.5,0,0,0,-0.0083; 0,-0.5,0,0,0,0.0083; 1,0,0,0,0,0];
force_cmd = [T.force_cmd1, T.force_cmd2, T.force_cmd3, T.force_cmd4, T.force_cmd5, T.force_cmd6];

% 北京时间基准
bj_start_h = 15; bj_start_m = 13; bj_start_s = 59.0;
bj_start_sec = bj_start_h*3600 + bj_start_m*60 + bj_start_s;

% 降采样参数 (与主分析一致)
FS_TARGET = 5;
ds_factor = max(1, round(1/(FS_TARGET * dt_raw)));
n_ds = floor(N_raw / ds_factor);
win_seg = max(10, round(2.0 / dt_raw));
n_win = floor(N_raw / win_seg);

% 窗口能量计算
seg_t = zeros(n_win, 1);
total_energy_w = zeros(n_win, 1);
energy_ratio_w = zeros(n_win, 6);
for w = 1:n_win
    idx = (w-1)*win_seg + 1 : min(w*win_seg, N_raw);
    seg_t(w) = mean(t_raw(idx));
    for ch = 1:6
        energy_ratio_w(w, ch) = sum(abs(force_cmd(idx, ch)));
    end
    total_energy_w(w) = sum(energy_ratio_w(w, :));
end
energy_ratio_w = energy_ratio_w ./ (total_energy_w + 1e-6);

chan_names = {'TX(Surge)', 'TY(Sway)', 'TZ(Heave)', 'MX(Roll)', 'MY(Pitch)', 'MZ(Yaw)'};

% 输出分段详情
fprintf('===== 水池实验时间线 =====\n');
fprintf('数据起始: %02d:%02d:%05.2f (北京时间)\n', bj_start_h, bj_start_m, bj_start_s);
fprintf('总时长: %.1f 分钟\n\n', t_raw(end)/60);

% 手动分段: 基于数据分析 + 实验记录, 识别大阶段
% 1. 准备阶段 (t=0~494s): 摄像头测试等
% 2. 定点测试 (t=494~960s): 15:22-15:30
% 3. 机动测试 (t=960~2940s): 前进后退/侧移/转艏/浮潜

fprintf('████ 1. 准备阶段 (15:14 - 15:22)\n');
t1 = 0; t2 = 494;
idx_range = t_raw >= t1 & t_raw < t2;
fprintf('   时长: %.0f 秒\n', t2-t1);
fprintf('   mode分布: mode=1:%.0f%%, mode=3:%.0f%%, mode=4:%.0f%%\n', ...
    sum(T.mode(idx_range)==1)/sum(idx_range)*100, ...
    sum(T.mode(idx_range)==3)/sum(idx_range)*100, ...
    sum(T.mode(idx_range)==4)/sum(idx_range)*100);
fprintf('   说明: AUV上电, 水面待机, 摄像头检测\n\n');

fprintf('████ 2. 定点测试 (15:22 - 15:30)\n');
t1 = 494; t2 = 960;
idx_range = t_raw >= t1 & t_raw < t2;
force_abs = sum(abs(force_cmd(idx_range,:)), 2);
fprintf('   时长: %.0f 秒 (~%.0f 分钟)\n', t2-t1, (t2-t1)/60);
fprintf('   平均总推力: %.0f CAN-g (定点期间应为低推力)\n', mean(force_abs));
% 检测DVL失效段 (速度数据突变)
u_seg = T.linear_vel_x(idx_range);
fprintf('   前进速度: mean=%.3f, std=%.3f m/s\n', mean(u_seg), std(u_seg));
% DVL失效: 15:23:37 (t≈531s), 15:24:24 (t≈578s) 重新开始
fprintf('   ⚠ DVL失效: ~15:23:37 (t≈%ds)\n', round(531));
fprintf('   ✅ 重新开始: ~15:24:24 (t≈%ds)\n', round(578));
fprintf('   结束: 15:30:56 (t≈%ds)\n\n', round(960));

% 大阶段划分
fprintf('████ 3. 机动测试阶段 (15:30 - 16:03)\n');
% 基于合并分段, 合并相邻同类段得到大阶段
fprintf('   以下基于力/力矩能量占比自动识别:\n\n');

for s = 1:size(merged_segments, 1)
    st_win = merged_segments(s, 1);
    en_win = merged_segments(s, 2);
    ty = merged_segments(s, 3);
    dur = merged_segments(s, 4);

    st_t = seg_t(st_win);
    en_t = seg_t(en_win);

    % 北京时间
    st_bj = bj_start_sec + st_t;
    en_bj = bj_start_sec + en_t;
    st_h = floor(st_bj/3600); st_m = floor(mod(st_bj,3600)/60); st_s = mod(st_bj,60);
    en_h = floor(en_bj/3600); en_m = floor(mod(en_bj,3600)/60); en_s = mod(en_bj,60);

    % 获取该段内的原始数据统计
    idx_seg = t_raw >= st_t & t_raw <= en_t;
    if sum(idx_seg) > 0
        cmd_seg = force_cmd(idx_seg, :);
        max_cmd = max(abs(cmd_seg), [], 1);
        [~, dom_ch] = max(max_cmd);

        % 判断正反向
        cmd_mean = mean(cmd_seg(:, dom_ch));
        if abs(cmd_mean) > 50
            dir_str = '正/负交替';
        else
            dir_str = '双向';
        end

        % 推进器推力
        thr_seg = zeros(sum(idx_seg), 5);
        for i = 1:sum(idx_seg)
            thr_seg(i,:) = (K_thr * cmd_seg(i,:)')';
        end
        thr_max = max(abs(thr_seg), [], 1);

        % 速度
        u_max = max(abs(T.linear_vel_x(idx_seg)));
        v_max = max(abs(T.linear_vel_y(idx_seg)));
        w_max = max(abs(T.linear_vel_z(idx_seg)));
        r_max = max(abs(T.angular_vel_z(idx_seg)));

        % 功率
        p_mean = mean(T.control_voltage(idx_seg) .* T.power_current(idx_seg));
    else
        cmd_mean = 0; dir_str = '';
        u_max = 0; v_max = 0; w_max = 0; r_max = 0; p_mean = 0;
        thr_max = zeros(1,5);
    end

    % 输出
    if dur >= 30  % 只输出长段
        fprintf('  ▸ %s  [%02d:%02d:%02.0f - %02d:%02d:%02.0f]  (%.0fs)\n', ...
            type_names{ty+1}, st_h, st_m, st_s, en_h, en_m, en_s, dur);
        fprintf('    指令: |TX|=%.0f |TY|=%.0f |TZ|=%.0f |MZ|=%.0f |MY|=%.0f\n', ...
            max(abs(cmd_seg(:,1))), max(abs(cmd_seg(:,2))), max(abs(cmd_seg(:,3))), ...
            max(abs(cmd_seg(:,6))), max(abs(cmd_seg(:,5))));
        fprintf('    推力: T5=%.0f T1=%.0f T2=%.0f T3=%.0f T4=%.0f (CAN-g)\n', ...
            thr_max(5), thr_max(1), thr_max(2), thr_max(3), thr_max(4));
        fprintf('    速度: |u|=%.2f |v|=%.2f |w|=%.2f |r|=%.1f°/s\n', u_max, v_max, w_max, r_max);
        fprintf('    功率: %.0f W\n\n', p_mean);
    end
end
