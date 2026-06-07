% analyze_pool_test_20260601.m — 水池实验数据分析
% 数据文件: data/raw/debug_data260601-1.csv (2026-06-01 水池测试, ~49分钟)
% 测试内容: 定点5分钟 + 不同力矩前进后退 + 左移右移 + 左转右转 + 上浮下潜
% 关键变化: T1前垂推已修复 (上次 debug_data260530-1.csv 时 T1 故障)
%
% 分析方法:
%   1. CAN协议推力分配矩阵 → 6-DOF指令转5推进器个体推力
%   2. 基于force_cmd能量占比自动分段
%   3. 时域灰箱辨识 (IRLS稳健回归): m*dv/dt = k*cmd - d1*v - d2*v*|v|
%   4. 与上次故障数据对比
%   5. 正向仿真验证
%
% 输出:
%   - assets/figures/pool_test_20260601_segmentation.png  (分段总览)
%   - assets/figures/pool_test_20260601_sysid.png         (系统辨识结果)
%   - assets/figures/pool_test_20260601_comparison.png    (修复前后对比)
%   - assets/figures/pool_test_20260601_thrusters.png     (推进器推力分析)
%   - 控制台输出关键辨识参数

clear; clc;
project_root = setup_paths();

%% ===== 配置 =====
DATA_FILE = 'data/raw/debug_data260601-1.csv';
PREV_DATA_FILE = 'data/raw/debug_data260530-1.csv';  % 上次故障数据(对比用)
OUT_DIR = 'C:/Users/sixuh/Documents/A_temp/test_obs/test';
OUT_PIC_DIR = fullfile(project_root, 'assets', 'figures');

% 物理参数 (与 sysid_drag.m 保持一致)
m_rb = 33;           % 刚体质量 kg
m11_ma = 11.2773;    % Surge 附加质量
m22_ma = 132.1086;   % Sway 附加质量
m33_ma = 47.10;      % Heave 附加质量
Izz_rb = 1.849137;   % Yaw 转动惯量
Izz_ma = 0.138;      % Yaw 附加转动惯量
Iyy_rb = 2.15;       % Pitch 转动惯量 (近似)
Iyy_ma = 0.043;      % Pitch 附加转动惯量

m_eff_x = m_rb + m11_ma;    % Surge 有效质量 44.28 kg
m_eff_y = m_rb + m22_ma;    % Sway 有效质量 165.11 kg
m_eff_z = m_rb + m33_ma;    % Heave 有效质量 80.10 kg
Izz_eff = Izz_rb + Izz_ma;  % Yaw 有效转动惯量 1.987 kg·m²
Iyy_eff = Iyy_rb + Iyy_ma;  % Pitch 有效转动惯量 2.193 kg·m²

% CAN 协议参数
g2N = 0.00981;  % g → N 转换系数

% CAN 推力分配矩阵: 5推进器 × 6自由度力/力矩指令
% 指令顺序: [TX, TY, TZ, MX, MY, MZ]
% 推进器顺序: [T1前垂推, T2后垂推, T3前侧推, T4后侧推, T5主推]
K_thr = [  0,   0, -0.5,  0,  0.0083,  0;     % T1 前垂推
           0,   0, -0.5,  0, -0.0083,  0;     % T2 后垂推
           0, -0.5,   0,  0,      0, -0.0083;  % T3 前侧推
           0, -0.5,   0,  0,      0,  0.0083;  % T4 后侧推
           1,   0,    0,  0,      0,      0];  % T5 主推

thr_names = {'T1前垂推', 'T2后垂推', 'T3前侧推', 'T4后侧推', 'T5主推'};

% 系统辨识参数
FS_TARGET = 5;     % 降采样目标频率 Hz (原始~8.3Hz, 轻度降采样提高信噪比)
SG_WIN_SEC = 2.0;  % Savitzky-Golay 滤波窗口 (秒)
IRLS_ITER = 10;    % IRLS 最大迭代次数

% 分段参数
CMD_CHANGE_THRESH = 500;   % 指令突变阈值 (CAN-g)
MIN_SEG_DURATION = 10;     % 最短段持续时间 (秒)
ENERGY_DOMINANCE = 0.4;    % 某通道能量占比超过此值即判定为主导通道

%% ===== 步骤1: 数据加载与预处理 =====
fprintf('========== 步骤1: 数据加载 ==========\n');

T_data = readtable(DATA_FILE);
t_raw = T_data.pc_timestamp - T_data.pc_timestamp(1);
N_raw = length(t_raw);
dt_raw = median(diff(t_raw));
fprintf('数据点: %d, 采样间隔: %.4fs (%.1f Hz), 总时长: %.1f min\n', ...
    N_raw, dt_raw, 1/dt_raw, t_raw(end)/60);

% 提取6-DOF力/力矩指令
force_cmd = [T_data.force_cmd1, T_data.force_cmd2, T_data.force_cmd3, ...
             T_data.force_cmd4, T_data.force_cmd5, T_data.force_cmd6];
% force_cmd1=X(TX), force_cmd2=Y(TY), force_cmd3=Z(TZ)
% force_cmd4=K(MX), force_cmd5=M(MY), force_cmd6=N(MZ)

% 运动状态
u_raw = T_data.linear_vel_x;  % m/s
v_raw = T_data.linear_vel_y;
w_raw = T_data.linear_vel_z;
p_raw = T_data.angular_vel_x;  % rad/s
q_raw = T_data.angular_vel_y;
r_raw = T_data.angular_vel_z * pi/180;  % deg/s → rad/s
attitude = [T_data.roll, T_data.pitch, T_data.yaw];  % deg
depth = T_data.depth_lpf;  % m

% 电功率
voltage = T_data.control_voltage;
current = T_data.power_current;
power_w = voltage .* current;
mode = T_data.mode;

fprintf('电压: %.1f~%.1f V, 电流: %.1f~%.1f A, 功率: %.1f~%.1f W\n', ...
    min(voltage), max(voltage), min(current), max(current), min(power_w), max(power_w));

% 数据质量检查
if any(isnan(force_cmd(:)))
    warning('force_cmd 含 NaN, 用前一有效值填充');
    force_cmd = fillmissing(force_cmd, 'previous');
end
if any(isnan(u_raw))
    warning('速度数据含 NaN');
    u_raw = fillmissing(u_raw, 'previous');
    v_raw = fillmissing(v_raw, 'previous');
    w_raw = fillmissing(w_raw, 'previous');
    r_raw = fillmissing(r_raw, 'previous');
end

%% ===== 步骤2: CAN推力分配 =====
fprintf('\n========== 步骤2: CAN推力分配 ==========\n');

thr_force = zeros(N_raw, 5);  % 5个推进器的个体推力 (CAN-g)
for i = 1:N_raw
    cmd_6dof = force_cmd(i, :)';
    thr_force(i, :) = (K_thr * cmd_6dof)';
end

% 统计各推进器推力
fprintf('各推进器推力统计 (CAN-g):\n');
for i = 1:5
    tf = thr_force(:, i);
    nonzero_pct = sum(abs(tf) > 10) / N_raw * 100;
    fprintf('  %-12s: mean=%8.1f, std=%8.1f, max=%8.1f, min=%8.1f, 活跃=%.1f%%\n', ...
        thr_names{i}, mean(tf), std(tf), max(tf), min(tf), nonzero_pct);
end

% 推进器推力 → 运动响应 (全部数据)
fprintf('\n全部数据 推力→速度 相关系数:\n');
fprintf('  T5主推     → u (前进): r=%.3f\n', corr(thr_force(:,5), u_raw));
fprintf('  T3+T4侧推  → v (侧移): r=%.3f\n', corr(thr_force(:,3)+thr_force(:,4), v_raw));
fprintf('  T1+T2垂推  → w (垂向): r=%.3f\n', corr(thr_force(:,1)+thr_force(:,2), w_raw));
fprintf('  T3-T4差动  → r (转艏): r=%.3f\n', corr(thr_force(:,3)-thr_force(:,4), r_raw));
fprintf('  T1前垂推   → q (俯仰): r=%.3f\n', corr(thr_force(:,1), q_raw));

%% ===== 步骤3: 数据分段 =====
fprintf('\n========== 步骤3: 自动分段 ==========\n');

% 计算滑动窗口内的各通道能量占比
win_seg = max(10, round(2.0 / dt_raw));  % 2秒窗口
n_win = floor(N_raw / win_seg);

% 滑动能量计算
seg_energy = zeros(n_win, 6);  % 6通道的能量
seg_t = zeros(n_win, 1);
for w = 1:n_win
    idx = (w-1)*win_seg + 1 : min(w*win_seg, N_raw);
    seg_t(w) = mean(t_raw(idx));
    for ch = 1:6
        seg_energy(w, ch) = sum(abs(force_cmd(idx, ch)));
    end
end

% 计算总能量和归一化能量占比
total_energy = sum(seg_energy, 2);
energy_ratio = seg_energy ./ (total_energy + 1e-6);

% 分段逻辑: 根据主导通道和mode分类
% 通道映射: 1=TX(Surge), 2=TY(Sway), 3=TZ(Heave), 4=MX(Roll), 5=MY(Pitch), 6=MZ(Yaw)
seg_labels = cell(n_win, 1);
seg_type = zeros(n_win, 1);  % 0=idle, 1=surge, 2=sway, 3=heave, 4=yaw, 5=pitch, 6=mixed
for w = 1:n_win
    if total_energy(w) < 100
        seg_labels{w} = '待机/定点';
        seg_type(w) = 0;
    else
        [max_ratio, max_ch] = max(energy_ratio(w, :));
        if max_ratio > ENERGY_DOMINANCE
            switch max_ch
                case 1, seg_labels{w} = '前进/后退'; seg_type(w) = 1;
                case 2, seg_labels{w} = '左移/右移'; seg_type(w) = 2;
                case 3, seg_labels{w} = '上浮/下潜'; seg_type(w) = 3;
                case 6, seg_labels{w} = '左转/右转'; seg_type(w) = 4;
                case 5, seg_labels{w} = '俯仰'; seg_type(w) = 5;
                otherwise, seg_labels{w} = '混合机动'; seg_type(w) = 6;
            end
        else
            seg_labels{w} = '混合机动'; seg_type(w) = 6;
        end
    end
end

% 合并连续的同类段
merged_segments = [];
curr_type = seg_type(1);
curr_start = 1;
curr_label = seg_labels{1};
for w = 2:n_win
    if seg_type(w) ~= curr_type
        dur = seg_t(w-1) - seg_t(curr_start);
        if dur >= MIN_SEG_DURATION
            merged_segments = [merged_segments; curr_start, w-1, curr_type, dur];
        end
        curr_type = seg_type(w);
        curr_start = w;
        curr_label = seg_labels{w};
    end
end
% 最后一段
dur = seg_t(end) - seg_t(curr_start);
if dur >= MIN_SEG_DURATION
    merged_segments = [merged_segments; curr_start, n_win, curr_type, dur];
end

type_names = {'待机/定点', '前进/后退', '左移/右移', '上浮/下潜', '左转/右转', '俯仰', '混合机动'};
fprintf('识别到 %d 个测试段:\n', size(merged_segments, 1));
fprintf('  %-4s %-6s %-6s %-10s %-10s\n', '#', '起始idx', '结束idx', '类型', '时长(s)');
for s = 1:size(merged_segments, 1)
    st = merged_segments(s, 1);
    en = merged_segments(s, 2);
    ty = merged_segments(s, 3);
    du = merged_segments(s, 4);
    fprintf('  %-4d %-6d %-6d %-10s %-10.1f\n', s, st, en, type_names{ty+1}, du);
end

%% ===== 步骤4: 系统辨识数据准备 =====
fprintf('\n========== 步骤4: 数据预处理 (降采样+滤波) ==========\n');

% 降采样到目标频率
ds_factor = max(1, round(1/(FS_TARGET * dt_raw)));
n_ds = floor(N_raw / ds_factor);
fprintf('降采样因子: %d (%.1f Hz → %.1f Hz), 降采样后点数: %d\n', ...
    ds_factor, 1/dt_raw, 1/(dt_raw*ds_factor), n_ds);

u_ds = zeros(n_ds, 1); v_ds = zeros(n_ds, 1); w_ds = zeros(n_ds, 1);
r_ds = zeros(n_ds, 1); q_ds = zeros(n_ds, 1);
cmd_X_ds = zeros(n_ds, 1); cmd_Y_ds = zeros(n_ds, 1); cmd_Z_ds = zeros(n_ds, 1);
cmd_N_ds = zeros(n_ds, 1); cmd_M_ds = zeros(n_ds, 1);
thr_ds = zeros(n_ds, 5); t_ds = zeros(n_ds, 1);

for i = 1:n_ds
    idx = (i-1)*ds_factor + 1 : min(i*ds_factor, N_raw);
    u_ds(i) = median(u_raw(idx));
    v_ds(i) = median(v_raw(idx));
    w_ds(i) = median(w_raw(idx));
    r_ds(i) = median(r_raw(idx));
    q_ds(i) = median(q_raw(idx));
    cmd_X_ds(i) = median(force_cmd(idx, 1));
    cmd_Y_ds(i) = median(force_cmd(idx, 2));
    cmd_Z_ds(i) = median(force_cmd(idx, 3));
    cmd_N_ds(i) = median(force_cmd(idx, 6));
    cmd_M_ds(i) = median(force_cmd(idx, 5));
    for j = 1:5
        thr_ds(i, j) = median(thr_force(idx, j));
    end
    t_ds(i) = median(t_raw(idx));
end
dt_ds = median(diff(t_ds));

% Savitzky-Golay 滤波
win_sg = max(3, round(SG_WIN_SEC / dt_ds));
if mod(win_sg, 2) == 0, win_sg = win_sg + 1; end
u_filt = sgolayfilt(u_ds, 2, win_sg);
v_filt = sgolayfilt(v_ds, 2, win_sg);
w_filt = sgolayfilt(w_ds, 2, win_sg);
r_filt = sgolayfilt(r_ds, 2, win_sg);
q_filt = sgolayfilt(q_ds, 2, win_sg);

% 中心差分求加速度
du_dt = centered_diff(u_filt, dt_ds);
dv_dt = centered_diff(v_filt, dt_ds);
dw_dt = centered_diff(w_filt, dt_ds);
dr_dt = centered_diff(r_filt, dt_ds);
dq_dt = centered_diff(q_filt, dt_ds);

fprintf('滤波后加速度范围:\n');
fprintf('  du/dt: [%.3f, %.3f] m/s²\n', min(du_dt), max(du_dt));
fprintf('  dv/dt: [%.3f, %.3f] m/s²\n', min(dv_dt), max(dv_dt));
fprintf('  dw/dt: [%.3f, %.3f] m/s²\n', min(dw_dt), max(dw_dt));
fprintf('  dr/dt: [%.3f, %.3f] rad/s²\n', min(dr_dt), max(dr_dt));
fprintf('  dq/dt: [%.3f, %.3f] rad/s²\n', min(dq_dt), max(dq_dt));

%% ===== 步骤5: 时域系统辨识 =====
fprintf('\n========== 步骤5: 时域灰箱辨识 ==========\n');

results = struct();

%% 5a. Surge (前进): m*du/dt = k_x*T5 - d1*u - d2*u*|u|
fprintf('\n--- Surge (主推 T5 → u) ---\n');
lhs_x = m_eff_x * du_dt;
mask_x = abs(cmd_X_ds) > 200 & abs(du_dt) > 0.005;
fprintf('数据点: %d / %d (%.1f%%)\n', sum(mask_x), n_ds, sum(mask_x)/n_ds*100);

if sum(mask_x) > 50
    X_reg_x = [cmd_X_ds(mask_x), -u_filt(mask_x), -u_filt(mask_x).*abs(u_filt(mask_x))];
    y_reg_x = lhs_x(mask_x);

    % IRLS
    w = ones(sum(mask_x), 1);
    coeff_x = X_reg_x \ y_reg_x;
    for iter = 1:IRLS_ITER
        resid = y_reg_x - X_reg_x * coeff_x;
        mad_val = median(abs(resid));
        if mad_val < 1e-8, break; end
        w = 1 ./ (1 + (resid / (mad_val * 1.4826)).^2);
        coeff_x = (X_reg_x' * diag(w) * X_reg_x) \ (X_reg_x' * diag(w) * y_reg_x);
    end

    k_x = coeff_x(1); d1_x = coeff_x(2); d2_x = coeff_x(3);
    y_pred_x = X_reg_x * coeff_x;
    R2_x = 1 - sum((y_reg_x-y_pred_x).^2) / sum((y_reg_x-mean(y_reg_x)).^2);

    fprintf('  辨识结果: k_x=%.6f N/CAN-g, d1=%.4f, d2=%.4f, R²=%.4f\n', k_x, d1_x, d2_x, R2_x);
    fprintf('  满指令推力 (cmd=8000): %.1f N\n', k_x*8000);

    results.surge = struct('k', k_x, 'd1', d1_x, 'd2', d2_x, 'R2', R2_x, 'mask', mask_x, ...
        'lhs', lhs_x, 'y_pred', y_pred_x, 'y_reg', y_reg_x, 'coeff', coeff_x);

    % 正向仿真验证
    seg_len = min(3000, n_ds);
    u_sim = zeros(seg_len, 1);
    for i = 1:seg_len-1
        F = k_x*cmd_X_ds(i) - d1_x*u_sim(i) - d2_x*u_sim(i)*abs(u_sim(i));
        u_sim(i+1) = u_sim(i) + F/m_eff_x * dt_ds;
    end
    SSE_val = sum((u_filt(1:seg_len) - u_sim).^2);
    SST_val = sum((u_filt(1:seg_len) - mean(u_filt(1:seg_len))).^2);
    R2_val_x = 1 - SSE_val/SST_val;
    fprintf('  正向仿真 R²=%.4f\n', R2_val_x);
    results.surge.R2_val = R2_val_x;
    results.surge.u_sim = u_sim;
    results.surge.seg_len = seg_len;
end

%% 5b. Sway (侧移): m*dv/dt = k_y*(T3+T4) - d1*v - d2*v*|v|
fprintf('\n--- Sway (侧推 T3+T4 → v) ---\n');
cmd_side_sum = thr_ds(:,3) + thr_ds(:,4);  % T3+T4 合力
lhs_y = m_eff_y * dv_dt;
mask_y = abs(cmd_side_sum) > 100 & abs(dv_dt) > 0.002;
fprintf('数据点: %d / %d (%.1f%%)\n', sum(mask_y), n_ds, sum(mask_y)/n_ds*100);

if sum(mask_y) > 50
    X_reg_y = [cmd_side_sum(mask_y), -v_filt(mask_y), -v_filt(mask_y).*abs(v_filt(mask_y))];
    y_reg_y = lhs_y(mask_y);

    w = ones(sum(mask_y), 1);
    coeff_y = X_reg_y \ y_reg_y;
    for iter = 1:IRLS_ITER
        resid = y_reg_y - X_reg_y * coeff_y;
        mad_val = median(abs(resid));
        if mad_val < 1e-8, break; end
        w = 1 ./ (1 + (resid/(mad_val*1.4826)).^2);
        coeff_y = (X_reg_y' * diag(w) * X_reg_y) \ (X_reg_y' * diag(w) * y_reg_y);
    end

    k_y = coeff_y(1); d1_y = coeff_y(2); d2_y = coeff_y(3);
    y_pred_y = X_reg_y * coeff_y;
    R2_y = 1 - sum((y_reg_y-y_pred_y).^2) / sum((y_reg_y-mean(y_reg_y)).^2);

    fprintf('  辨识结果: k_y=%.6f N/CAN-g, d1=%.4f, d2=%.4f, R²=%.4f\n', k_y, d1_y, d2_y, R2_y);
    results.sway = struct('k', k_y, 'd1', d1_y, 'd2', d2_y, 'R2', R2_y, 'mask', mask_y, ...
        'lhs', lhs_y, 'y_pred', y_pred_y, 'y_reg', y_reg_y);

    % 正向仿真
    seg_len = min(3000, n_ds);
    v_sim = zeros(seg_len, 1);
    for i = 1:seg_len-1
        F = k_y*cmd_side_sum(i) - d1_y*v_sim(i) - d2_y*v_sim(i)*abs(v_sim(i));
        v_sim(i+1) = v_sim(i) + F/m_eff_y * dt_ds;
    end
    R2_val_y = 1 - sum((v_filt(1:seg_len)-v_sim).^2)/sum((v_filt(1:seg_len)-mean(v_filt(1:seg_len))).^2);
    fprintf('  正向仿真 R²=%.4f\n', R2_val_y);
    results.sway.R2_val = R2_val_y;
end

%% 5c. Heave (垂向): m*dw/dt = k_z*(T1+T2) - d1*w - d2*w*|w| - g_buoyancy
fprintf('\n--- Heave (垂推 T1+T2 → w) ---\n');
cmd_vert_sum = thr_ds(:,1) + thr_ds(:,2);  % T1+T2 合力
% 浮力修正: 根据实验记录, AUV略微负浮力, 先假设0
g_buoyancy = 0;
lhs_z = m_eff_z * dw_dt + g_buoyancy;
mask_z = abs(cmd_vert_sum) > 50 & abs(dw_dt) > 0.002;
fprintf('数据点: %d / %d (%.1f%%)\n', sum(mask_z), n_ds, sum(mask_z)/n_ds*100);

if sum(mask_z) > 30
    X_reg_z = [cmd_vert_sum(mask_z), -w_filt(mask_z), -w_filt(mask_z).*abs(w_filt(mask_z))];
    y_reg_z = lhs_z(mask_z);

    w_z = ones(sum(mask_z), 1);
    coeff_z = X_reg_z \ y_reg_z;
    for iter = 1:IRLS_ITER
        resid = y_reg_z - X_reg_z * coeff_z;
        mad_val = median(abs(resid));
        if mad_val < 1e-8, break; end
        w_z = 1 ./ (1 + (resid/(mad_val*1.4826)).^2);
        coeff_z = (X_reg_z' * diag(w_z) * X_reg_z) \ (X_reg_z' * diag(w_z) * y_reg_z);
    end

    k_z = coeff_z(1); d1_z = coeff_z(2); d2_z = coeff_z(3);
    y_pred_z = X_reg_z * coeff_z;
    R2_z = 1 - sum((y_reg_z-y_pred_z).^2) / sum((y_reg_z-mean(y_reg_z)).^2);

    fprintf('  辨识结果: k_z=%.6f N/CAN-g, d1=%.4f, d2=%.4f, R²=%.4f\n', k_z, d1_z, d2_z, R2_z);
    fprintf('  T1+T2 有效推力 (cmd=8000): %.1f N\n', k_z*8000);
    results.heave = struct('k', k_z, 'd1', d1_z, 'd2', d2_z, 'R2', R2_z, 'mask', mask_z, ...
        'lhs', lhs_z, 'y_pred', y_pred_z, 'y_reg', y_reg_z);
end

%% 5d. Yaw (转艏): Izz*dr/dt = k_n*cmd_N - dr*r
fprintf('\n--- Yaw (MZ → r) ---\n');
lhs_n = Izz_eff * dr_dt;
mask_n = abs(cmd_N_ds) > 50 & abs(dr_dt) > 0.01;
fprintf('数据点: %d / %d (%.1f%%)\n', sum(mask_n), n_ds, sum(mask_n)/n_ds*100);

if sum(mask_n) > 50
    X_reg_n = [cmd_N_ds(mask_n), -r_filt(mask_n)];
    y_reg_n = lhs_n(mask_n);

    w_n = ones(sum(mask_n), 1);
    coeff_n = X_reg_n \ y_reg_n;
    for iter = 1:IRLS_ITER
        resid = y_reg_n - X_reg_n * coeff_n;
        mad_val = median(abs(resid));
        if mad_val < 1e-8, break; end
        w_n = 1 ./ (1 + (resid/(mad_val*1.4826)).^2);
        coeff_n = (X_reg_n' * diag(w_n) * X_reg_n) \ (X_reg_n' * diag(w_n) * y_reg_n);
    end

    k_n = coeff_n(1); dr_n = coeff_n(2);
    y_pred_n = X_reg_n * coeff_n;
    R2_n = 1 - sum((y_reg_n-y_pred_n).^2) / sum((y_reg_n-mean(y_reg_n)).^2);

    fprintf('  辨识结果: k_n=%.6f N·m/CAN-unit, dr=%.4f N·m·s/rad, R²=%.4f\n', k_n, dr_n, R2_n);
    results.yaw = struct('k', k_n, 'dr', dr_n, 'R2', R2_n, 'mask', mask_n, ...
        'lhs', lhs_n, 'y_pred', y_pred_n, 'y_reg', y_reg_n);

    % dr 对应的有效倍数 (相对于 CFD 原值 1.85)
    dr_cfd = 1.849;
    dr_mult = dr_n / (Izz_eff/1 * 2.4);  % 相对上次校准的倍数
    fprintf('  CFD原值 dr=%.3f, 上次校准倍率=2.4\n', dr_cfd);
end

%% 5e. Pitch (俯仰): Iyy*dq/dt = k_m*cmd_MY - dq*q
fprintf('\n--- Pitch (MY → q) ---\n');
lhs_m = Iyy_eff * dq_dt;
mask_m = abs(cmd_M_ds) > 50 & abs(dq_dt) > 0.005;
fprintf('数据点: %d / %d (%.1f%%)\n', sum(mask_m), n_ds, sum(mask_m)/n_ds*100);

if sum(mask_m) > 30
    X_reg_m = [cmd_M_ds(mask_m), -q_filt(mask_m)];
    y_reg_m = lhs_m(mask_m);

    w_m = ones(sum(mask_m), 1);
    coeff_m = X_reg_m \ y_reg_m;
    for iter = 1:IRLS_ITER
        resid = y_reg_m - X_reg_m * coeff_m;
        mad_val = median(abs(resid));
        if mad_val < 1e-8, break; end
        w_m = 1 ./ (1 + (resid/(mad_val*1.4826)).^2);
        coeff_m = (X_reg_m' * diag(w_m) * X_reg_m) \ (X_reg_m' * diag(w_m) * y_reg_m);
    end

    k_m = coeff_m(1); dq_m = coeff_m(2);
    y_pred_m = X_reg_m * coeff_m;
    R2_m = 1 - sum((y_reg_m-y_pred_m).^2) / sum((y_reg_m-mean(y_reg_m)).^2);

    fprintf('  辨识结果: k_m=%.6f N·m/CAN-unit, dq=%.4f, R²=%.4f\n', k_m, dq_m, R2_m);
    results.pitch = struct('k', k_m, 'dq', dq_m, 'R2', R2_m, 'mask', mask_m, ...
        'lhs', lhs_m, 'y_pred', y_pred_m, 'y_reg', y_reg_m);
end

%% ===== 步骤6: 与上次数据对比 =====
fprintf('\n========== 步骤6: 修复前后对比 (vs 2026-05-30 T1故障) ==========\n');

if exist(PREV_DATA_FILE, 'file')
    T_prev = readtable(PREV_DATA_FILE);
    t_prev = T_prev.pc_timestamp - T_prev.pc_timestamp(1);
    N_prev = length(t_prev);

    % 计算推进器推力
    force_cmd_prev = [T_prev.force_cmd1, T_prev.force_cmd2, T_prev.force_cmd3, ...
                      T_prev.force_cmd4, T_prev.force_cmd5, T_prev.force_cmd6];
    thr_prev = zeros(N_prev, 5);
    for i = 1:N_prev
        thr_prev(i, :) = (K_thr * force_cmd_prev(i, :)')';
    end

    w_prev = T_prev.linear_vel_z;
    q_prev = T_prev.angular_vel_y;

    % 对比指标
    fprintf('\n指标                      上次(T1故障)    本次(修复后)    变化\n');
    fprintf('%s\n', repmat('-', 1, 75));

    % Heave: T1+T2 有效推力
    r_zw_prev = corr(thr_prev(:,1)+thr_prev(:,2), w_prev);
    r_zw_curr = corr(thr_ds(:,1)+thr_ds(:,2), w_filt);
    fprintf('Heave T1+T2→w 相关系数    %.3f           %.3f           %+.3f\n', r_zw_prev, r_zw_curr, r_zw_curr-r_zw_prev);

    % Pitch: MY→q
    r_mq_prev = corr(force_cmd_prev(:,5), q_prev);
    r_mq_curr = corr(cmd_M_ds, q_filt);
    fprintf('Pitch MY→q 相关系数       %.3f           %.3f           %+.3f\n', r_mq_prev, r_mq_curr, r_mq_curr-r_mq_prev);

    % 垂推总有效推力
    T1_prev_active = sum(abs(thr_prev(:,1)) > 10);
    T2_prev_active = sum(abs(thr_prev(:,2)) > 10);
    T1_curr_active = sum(abs(thr_ds(:,1)) > 10);
    T2_curr_active = sum(abs(thr_ds(:,2)) > 10);
    fprintf('T1 活跃比例               %.1f%%            %.1f%%\n', ...
        T1_prev_active/N_prev*100, T1_curr_active/n_ds*100);
    fprintf('T2 活跃比例               %.1f%%            %.1f%%\n', ...
        T2_prev_active/N_prev*100, T2_curr_active/n_ds*100);

    % Z-M 耦合 (T1正常时T1和T2应对称)
    r_zm_prev = corr(force_cmd_prev(:,3), force_cmd_prev(:,5));
    r_zm_curr = corr(cmd_Z_ds, cmd_M_ds);
    fprintf('Z-M指令相关系数           %.3f           %.3f           %+.3f\n', r_zm_prev, r_zm_curr, r_zm_curr-r_zm_prev);

    % 主推 k
    fprintf('\n主推 Surge k (N/CAN-g):   上次=%.6f  本次=%.6f\n', 0.00080, k_x);

    comparison = struct();
    comparison.r_zw_prev = r_zw_prev; comparison.r_zw_curr = r_zw_curr;
    comparison.r_mq_prev = r_mq_prev; comparison.r_mq_curr = r_mq_curr;
    comparison.r_zm_prev = r_zm_prev; comparison.r_zm_curr = r_zm_curr;
    comparison.T1_prev = T1_prev_active/N_prev*100;
    comparison.T1_curr = T1_curr_active/n_ds*100;
    comparison.T2_prev = T2_prev_active/N_prev*100;
    comparison.T2_curr = T2_curr_active/n_ds*100;
    comparison.k_x_prev = 0.00080;
    comparison.k_x_curr = k_x;
    comparison.has_prev = true;
else
    fprintf('未找到上次数据文件: %s, 跳过对比\n', PREV_DATA_FILE);
    comparison = struct('has_prev', false);
end

%% ===== 步骤7: 推进器效率分析 =====
fprintf('\n========== 步骤7: 推进效率分析 ==========\n');

% 总推力幅值
total_thrust_g = sqrt(sum(thr_ds.^2, 2));

% 机械功率估算
mech_T5 = abs(thr_ds(:,5) * g2N .* u_filt);
mech_side = abs((thr_ds(:,3)+thr_ds(:,4)) * g2N .* v_filt);
mech_vert = abs((thr_ds(:,1)+thr_ds(:,2)) * g2N .* w_filt);
total_mech_w = mech_T5 + mech_side + mech_vert;

% 降采样后的功率
power_ds = zeros(n_ds, 1);
for i = 1:n_ds
    idx = (i-1)*ds_factor + 1 : min(i*ds_factor, N_raw);
    power_ds(i) = median(power_w(idx));
end
P_base_ds = min(power_ds);
P_excess = power_ds - P_base_ds;
P_excess(P_excess < 0) = 0;

mask_thrust = total_thrust_g > 200;
if sum(mask_thrust) > 50
    eff_inst = total_mech_w(mask_thrust) ./ power_ds(mask_thrust) * 100;
    eff_excess = total_mech_w(mask_thrust) ./ P_excess(mask_thrust) * 100;
    fprintf('有推力期间 (total>200g, %d点):\n', sum(mask_thrust));
    fprintf('  平均电功率: %.1f W, 平均机械功率: %.2f W\n', mean(power_ds(mask_thrust)), mean(total_mech_w(mask_thrust)));
    fprintf('  总效率: mean=%.1f%%, max=%.1f%%\n', mean(eff_inst), max(eff_inst));
    fprintf('  超额效率: mean=%.1f%%, max=%.1f%%\n', mean(eff_excess), max(eff_excess));

    % 各推进器对功耗贡献
    fprintf('\n各推进器 |推力| vs 电功率:\n');
    for i = 1:5
        r_tp = corr(abs(thr_ds(:,i)), power_ds);
        fprintf('  %s: r=%.3f\n', thr_names{i}, r_tp);
    end
end

%% ===== 步骤8: 汇总表 =====
fprintf('\n');
fprintf('========== 时域辨识汇总 ==========\n');
fprintf('%-10s %-16s %-12s %-12s %-8s %-8s\n', '通道', 'k (N/单位)', 'd1', 'd2/dr', 'R²', '正向R²');
fprintf('%s\n', repmat('-', 1, 75));
if isfield(results, 'surge')
    fprintf('%-10s %-16.6f %-12.4f %-12.4f %-8.4f %-8.4f\n', ...
        'Surge', results.surge.k, results.surge.d1, results.surge.d2, results.surge.R2, results.surge.R2_val);
end
if isfield(results, 'sway')
    fprintf('%-10s %-16.6f %-12.4f %-12.4f %-8.4f\n', ...
        'Sway', results.sway.k, results.sway.d1, results.sway.d2, results.sway.R2);
end
if isfield(results, 'heave')
    fprintf('%-10s %-16.6f %-12.4f %-12.4f %-8.4f\n', ...
        'Heave', results.heave.k, results.heave.d1, results.heave.d2, results.heave.R2);
end
if isfield(results, 'yaw')
    fprintf('%-10s %-16.6f %-12s %-12.4f %-8.4f\n', ...
        'Yaw', results.yaw.k, '—', results.yaw.dr, results.yaw.R2);
end
if isfield(results, 'pitch')
    fprintf('%-10s %-16.6f %-12s %-12.4f %-8.4f\n', ...
        'Pitch', results.pitch.k, '—', results.pitch.dq, results.pitch.R2);
end

fprintf('\n上次 Surge 辨识: k=0.00080, d1=1.35, d2=1.62, R²=0.30, 正向R²=0.64\n');
fprintf('上次 Sway: d2=87.9 (最大速度匹配法, 非时域辨识)\n');
fprintf('上次 Heave: d2=67.3 (最大速度匹配+浮力修正, T1故障时)\n');
fprintf('上次 Yaw: 倍率=2.4\n');

%% ===== 绘图 =====
fprintf('\n========== 生成图表 ==========\n');

%% 图1: 数据分段总览
figure('Position', [50, 50, 1600, 900]);

subplot(4,1,1);
plot(t_raw/60, force_cmd(:,1), 'b', 'DisplayName', 'TX(Surge)'); hold on;
plot(t_raw/60, force_cmd(:,2), 'r', 'DisplayName', 'TY(Sway)');
plot(t_raw/60, force_cmd(:,3), 'g', 'DisplayName', 'TZ(Heave)');
plot(t_raw/60, force_cmd(:,6), 'm', 'DisplayName', 'MZ(Yaw)');
ylabel('力/力矩指令');
title('6-DOF 控制指令时序 (含自动分段)');
legend('Location', 'best'); grid on;

% 标注分段
yl = ylim;
for s = 1:size(merged_segments, 1)
    st_t = seg_t(merged_segments(s, 1)) / 60;
    en_t = seg_t(merged_segments(s, 2)) / 60;
    ty = merged_segments(s, 3);
    colors_seg = {[0.9 0.9 0.9], [1 0.8 0.8], [0.8 0.8 1], [0.8 1 0.8], [1 1 0.8], [1 0.8 1], [0.8 1 1]};
    patch([st_t en_t en_t st_t], [yl(1) yl(1) yl(2) yl(2)], ...
        colors_seg{min(ty+1, 7)}, 'FaceAlpha', 0.2, 'EdgeColor', 'none');
    text((st_t+en_t)/2, yl(2)*0.95, type_names{ty+1}, ...
        'HorizontalAlignment', 'center', 'FontSize', 7, 'BackgroundColor', 'w');
end

subplot(4,1,2);
plot(t_raw/60, u_raw, 'b', 'DisplayName', 'u'); hold on;
plot(t_raw/60, v_raw, 'r', 'DisplayName', 'v');
plot(t_raw/60, w_raw, 'g', 'DisplayName', 'w');
ylabel('线速度 (m/s)'); legend('Location', 'best'); grid on;
title('线速度响应');

subplot(4,1,3);
plot(t_raw/60, r_raw*180/pi, 'm', 'DisplayName', 'r (deg/s)'); hold on;
plot(t_raw/60, q_raw*180/pi, 'c', 'DisplayName', 'q (deg/s)');
ylabel('角速度 (deg/s)'); legend('Location', 'best'); grid on;
title('角速度响应');

subplot(4,1,4);
plot(t_raw/60, mode, 'k'); ylabel('Mode'); xlabel('时间 (min)');
title('运行模式'); grid on; ylim([0 5]);

sgtitle('AUV 水池测试数据分段 (2026-06-01)');
saveas(gcf, fullfile(OUT_PIC_DIR, 'pool_test_20260601_segmentation.png'));
fprintf('图表1: %s/pool_test_20260601_segmentation.png\n', OUT_PIC_DIR);

%% 图2: 系统辨识结果 (类似 sysid_results_v3.png)
figure('Position', [50, 50, 1600, 1000]);

% Surge 阻力拟合
subplot(2,3,1);
if isfield(results, 'surge')
    drag_data = results.surge.k * cmd_X_ds(results.surge.mask) - m_eff_x * du_dt(results.surge.mask);
    scatter(u_filt(results.surge.mask), drag_data, 2, 'filled', 'MarkerFaceAlpha', 0.15); hold on;
    [u_sort, idx] = sort(u_filt(results.surge.mask));
    d1 = results.surge.d1; d2 = results.surge.d2;
    plot(u_sort, d1*u_sort + d2*u_sort.*abs(u_sort), 'r-', 'LineWidth', 2);
    plot(u_sort, 1.35*u_sort + 1.62*u_sort.*abs(u_sort), 'b--', 'LineWidth', 1.5);
    xlabel('u (m/s)'); ylabel('阻力 (N)');
    title(sprintf('Surge 阻力: d1=%.2f d2=%.2f  R²=%.3f', d1, d2, results.surge.R2));
    legend('数据(本次)', '辨识(本次)', '上次辨识', 'Location', 'best'); grid on;
end

% Surge 正向仿真
subplot(2,3,2);
if isfield(results, 'surge')
    plot_len = min(1500, results.surge.seg_len);
    plot(t_ds(1:plot_len)/60, u_filt(1:plot_len), 'b.', 'MarkerSize', 1); hold on;
    plot(t_ds(1:plot_len)/60, results.surge.u_sim(1:plot_len), 'r-', 'LineWidth', 1);
    xlabel('时间 (min)'); ylabel('u (m/s)');
    title(sprintf('Surge 正向验证 R²=%.3f', results.surge.R2_val));
    legend('实测', '仿真', 'Location', 'best'); grid on;
end

% Sway
subplot(2,3,3);
if isfield(results, 'sway')
    scatter(results.sway.y_reg, results.sway.y_pred, 2, 'filled', 'MarkerFaceAlpha', 0.15); hold on;
    plot([min(results.sway.y_reg) max(results.sway.y_reg)], ...
         [min(results.sway.y_reg) max(results.sway.y_reg)], 'r-');
    xlabel('实测 m*dv/dt'); ylabel('预测');
    title(sprintf('Sway 合力拟合 R²=%.3f', results.sway.R2)); grid on;
end

% Heave
subplot(2,3,4);
if isfield(results, 'heave')
    scatter(results.heave.y_reg, results.heave.y_pred, 2, 'filled', 'MarkerFaceAlpha', 0.15); hold on;
    plot([min(results.heave.y_reg) max(results.heave.y_reg)], ...
         [min(results.heave.y_reg) max(results.heave.y_reg)], 'r-');
    xlabel('实测 m*dw/dt'); ylabel('预测');
    title(sprintf('Heave 合力拟合 R²=%.3f', results.heave.R2)); grid on;
end

% Yaw
subplot(2,3,5);
if isfield(results, 'yaw')
    scatter(results.yaw.y_reg, results.yaw.y_pred, 2, 'filled', 'MarkerFaceAlpha', 0.15); hold on;
    plot([min(results.yaw.y_reg) max(results.yaw.y_reg)], ...
         [min(results.yaw.y_reg) max(results.yaw.y_reg)], 'r-');
    xlabel('实测 Izz*dr/dt'); ylabel('预测');
    title(sprintf('Yaw R²=%.3f, dr=%.2f', results.yaw.R2, results.yaw.dr)); grid on;
end

% Pitch
subplot(2,3,6);
if isfield(results, 'pitch')
    scatter(results.pitch.y_reg, results.pitch.y_pred, 2, 'filled', 'MarkerFaceAlpha', 0.15); hold on;
    plot([min(results.pitch.y_reg) max(results.pitch.y_reg)], ...
         [min(results.pitch.y_reg) max(results.pitch.y_reg)], 'r-');
    xlabel('实测 Iyy*dq/dt'); ylabel('预测');
    title(sprintf('Pitch R²=%.3f, dq=%.2f', results.pitch.R2, results.pitch.dq)); grid on;
end

sgtitle('AUV 水池测试 时域系统辨识 (2026-06-01, T1修复后)');
saveas(gcf, fullfile(OUT_PIC_DIR, 'pool_test_20260601_sysid.png'));
fprintf('图表2: %s/pool_test_20260601_sysid.png\n', OUT_PIC_DIR);

%% 图3: 修复前后对比
if comparison.has_prev
    figure('Position', [100, 100, 1400, 800]);

    % Heave 对比: 垂推合力 vs w
    subplot(2,3,1);
    % 降采样上次数据以对齐
    ds_prev = max(1, round(length(t_prev)/n_ds));
    n_prev_ds = floor(length(t_prev)/ds_prev);
    thr_prev_ds = zeros(n_prev_ds, 5);
    w_prev_ds = zeros(n_prev_ds, 1);
    for i = 1:n_prev_ds
        idx = (i-1)*ds_prev+1 : min(i*ds_prev, length(t_prev));
        thr_prev_ds(i,:) = median(thr_prev(idx,:), 1);
        w_prev_ds(i) = median(w_prev(idx));
    end
    scatter(thr_prev_ds(:,1)+thr_prev_ds(:,2), w_prev_ds, 3, 'filled', 'MarkerFaceAlpha', 0.1); hold on;
    scatter(thr_ds(:,1)+thr_ds(:,2), w_filt, 3, 'filled', 'MarkerFaceAlpha', 0.1);
    xlabel('T1+T2 垂推合力 (CAN-g)'); ylabel('w (m/s)');
    title(sprintf('Heave: T1+T2→w (修复前r=%.3f, 修复后r=%.3f)', comparison.r_zw_prev, comparison.r_zw_curr));
    legend('上次(T1故障)', '本次(修复)', 'Location', 'best'); grid on;

    % Pitch 对比: MY→q
    subplot(2,3,2);
    ds_cmd_M = max(1, round(length(t_prev)/n_ds));
    cmd_M_prev_ds = zeros(n_prev_ds, 1);
    q_prev_ds = zeros(n_prev_ds, 1);
    for i = 1:n_prev_ds
        idx = (i-1)*ds_cmd_M+1 : min(i*ds_cmd_M, length(t_prev));
        cmd_M_prev_ds(i) = median(force_cmd_prev(idx, 5));
        q_prev_ds(i) = median(q_prev(idx));
    end
    scatter(cmd_M_prev_ds, q_prev_ds, 3, 'filled', 'MarkerFaceAlpha', 0.1); hold on;
    scatter(cmd_M_ds, q_filt, 3, 'filled', 'MarkerFaceAlpha', 0.1);
    xlabel('MY 俯仰力矩指令'); ylabel('q (rad/s)');
    title(sprintf('Pitch: MY→q (修复前r=%.3f, 修复后r=%.3f)', comparison.r_mq_prev, comparison.r_mq_curr));
    legend('上次(T1故障)', '本次(修复)', 'Location', 'best'); grid on;

    % Z-M 指令耦合对比
    subplot(2,3,3);
    scatter(force_cmd_prev(1:ds_prev:end,3), force_cmd_prev(1:ds_prev:end,5), 3, 'filled', 'MarkerFaceAlpha', 0.1); hold on;
    scatter(cmd_Z_ds, cmd_M_ds, 3, 'filled', 'MarkerFaceAlpha', 0.1);
    xlabel('TZ 垂向力指令'); ylabel('MY 俯仰力矩指令');
    title(sprintf('Z-M指令耦合 (修复前r=%.3f, 修复后r=%.3f)', comparison.r_zm_prev, comparison.r_zm_curr));
    legend('上次(T1故障,强耦合)', '本次(修复,解耦)', 'Location', 'best'); grid on;

    % 速度对比
    subplot(2,3,4);
    histogram(w_prev_ds, 30, 'FaceAlpha', 0.4, 'DisplayName', '上次 w'); hold on;
    histogram(w_filt, 30, 'FaceAlpha', 0.4, 'DisplayName', '本次 w');
    xlabel('w (m/s)'); ylabel('频次');
    title('垂向速度分布对比'); legend; grid on;

    % 推进器活跃度对比
    subplot(2,3,5);
    T1_active = [comparison.T1_prev, comparison.T1_curr];
    T2_active = [comparison.T2_prev, comparison.T2_curr];
    bar_data = [T1_active; T2_active]';
    bar(bar_data);
    set(gca, 'XTickLabel', {'上次(T1故障)', '本次(修复)'});
    ylabel('活跃比例 (%)');
    legend('T1前垂推', 'T2后垂推', 'Location', 'best');
    title('垂推活跃度对比');
    grid on;

    % Surge k 对比
    subplot(2,3,6);
    bar([comparison.k_x_prev, comparison.k_x_curr]);
    set(gca, 'XTickLabel', {'上次', '本次'});
    ylabel('k (N/CAN-g)');
    title(sprintf('主推推力系数: 上次%.5f vs 本次%.5f', comparison.k_x_prev, comparison.k_x_curr));
    grid on;

    sgtitle('AUV 推进器修复前后对比 (2026-05-30 vs 2026-06-01)');
    saveas(gcf, fullfile(OUT_PIC_DIR, 'pool_test_20260601_comparison.png'));
    fprintf('图表3: %s/pool_test_20260601_comparison.png\n', OUT_PIC_DIR);
end

%% 图4: 推进器综合分析 (类似 analyze_final_report.m 的输出)
figure('Position', [50, 50, 1600, 1000]);

% 五推进器推力时序
subplot(3,4,1);
plot(t_ds/60, thr_ds);
xlabel('时间 (min)'); ylabel('推力 (CAN-g)'); title('五推进器推力指令');
legend(thr_names, 'Location', 'best'); grid on;

% 推力分布
subplot(3,4,2);
hold on;
colors = lines(5);
for i = 1:5
    histogram(thr_ds(:,i), 40, 'FaceAlpha', 0.3, 'DisplayName', thr_names{i});
end
xlabel('推力 (CAN-g)'); ylabel('频次'); title('推力分布'); legend('Location', 'best'); grid on;

% 功率时序
subplot(3,4,3);
yyaxis left; plot(t_ds/60, power_ds); ylabel('功率 (W)');
yyaxis right; plot(t_ds/60, P_excess); ylabel('超额功率 (W)');
xlabel('时间 (min)'); title('电功率'); grid on;

% 总推力 vs 电功率
subplot(3,4,4);
scatter(total_thrust_g, power_ds, 3, 'filled', 'MarkerFaceAlpha', 0.15);
xlabel('总推力幅值 (CAN-g)'); ylabel('电功率 (W)');
title(sprintf('推力 vs 功率 r=%.3f', corr(total_thrust_g, power_ds)));
grid on;

% T5 vs u
subplot(3,4,5);
scatter(thr_ds(:,5), u_filt, 3, 'filled', 'MarkerFaceAlpha', 0.1);
xlabel('T5 主推 (CAN-g)'); ylabel('u (m/s)');
title(sprintf('主推→前进 r=%.3f', corr(thr_ds(:,5), u_filt)));
grid on;

% T3+T4 vs v
subplot(3,4,6);
scatter(thr_ds(:,3)+thr_ds(:,4), v_filt, 3, 'filled', 'MarkerFaceAlpha', 0.1);
xlabel('T3+T4 侧推 (CAN-g)'); ylabel('v (m/s)');
title(sprintf('侧推→侧移 r=%.3f', corr(thr_ds(:,3)+thr_ds(:,4), v_filt)));
grid on;

% T1+T2 vs w
subplot(3,4,7);
scatter(thr_ds(:,1)+thr_ds(:,2), w_filt, 3, 'filled', 'MarkerFaceAlpha', 0.1);
xlabel('T1+T2 垂推 (CAN-g)'); ylabel('w (m/s)');
title(sprintf('垂推→垂向 r=%.3f', corr(thr_ds(:,1)+thr_ds(:,2), w_filt)));
grid on;

% T3-T4 vs r
subplot(3,4,8);
scatter(thr_ds(:,3)-thr_ds(:,4), r_filt*180/pi, 3, 'filled', 'MarkerFaceAlpha', 0.1);
xlabel('T3-T4 差动 (CAN-g)'); ylabel('r (deg/s)');
title(sprintf('侧推差动→转艏 r=%.3f', corr(thr_ds(:,3)-thr_ds(:,4), r_filt)));
grid on;

% T1 vs T2 (验证对称性)
subplot(3,4,9);
scatter(thr_ds(:,1), thr_ds(:,2), 3, 'filled', 'MarkerFaceAlpha', 0.1);
xlabel('T1 前垂推 (CAN-g)'); ylabel('T2 后垂推 (CAN-g)');
title(sprintf('前后垂推 T1 vs T2 r=%.3f', corr(thr_ds(:,1), thr_ds(:,2))));
grid on;

% 机械功率 vs 电功率
subplot(3,4,10);
scatter(power_ds, total_mech_w, 3, 'filled', 'MarkerFaceAlpha', 0.15);
xlabel('电功率 (W)'); ylabel('机械功率 (W)');
title('电功率 vs 机械功率'); grid on;

% 姿态角
subplot(3,4,11);
attitude_ds = zeros(n_ds, 3);
for i = 1:n_ds
    idx = (i-1)*ds_factor+1 : min(i*ds_factor, N_raw);
    attitude_ds(i,:) = median(attitude(idx,:), 1);
end
plot(t_ds/60, attitude_ds);
xlabel('时间 (min)'); ylabel('姿态 (deg)');
title('姿态角'); legend('Roll','Pitch','Yaw','Location','best'); grid on;

% 推进器RMS占比
subplot(3,4,12);
thr_rms = sqrt(mean(thr_ds.^2, 1));
pie(thr_rms, thr_names);
title('各推进器 RMS 推力占比');

sgtitle('AUV 水池测试 推进器综合分析 (2026-06-01, T1修复后)');
saveas(gcf, fullfile(OUT_PIC_DIR, 'pool_test_20260601_thrusters.png'));
fprintf('图表4: %s/pool_test_20260601_thrusters.png\n', OUT_PIC_DIR);

%% ===== 保存结果 =====
save('results/pool_test_20260601_results.mat', 'results', 'comparison', 'merged_segments', 'type_names');
fprintf('\n结果已保存至: results/pool_test_20260601_results.mat\n');

fprintf('\n========== 分析完成 ==========\n');
fprintf('总耗时: %.1f 分钟\n', t_raw(end)/60);

%% 辅助函数
function dv = centered_diff(v, dt)
    n = length(v);
    dv = zeros(n, 1);
    dv(1) = (v(2)-v(1))/dt;
    for i = 2:n-1
        dv(i) = (v(i+1)-v(i-1))/(2*dt);
    end
    dv(n) = (v(n)-v(n-1))/dt;
end
