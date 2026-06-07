% analyze_final_report.m — 结合CAN协议进行完整分析
% 输出图表到报告目录
clear; clc;

out_dir = 'C:/Users/sixuh/Documents/A_temp/test_obs/test';
mkdir(out_dir);

T = readtable('data/raw/debug_data260530-1.csv');

t = T.pc_timestamp - T.pc_timestamp(1);
N = length(t);

% === CAN协议参数 ===
% 推力分配矩阵 K: 5×6, T[i] = sum_j K[i,j] * cmd[j]
%          TX   TY   TZ   MX   MY   MZ
K_thr = [  0,   0, -0.5,  0,  0.0083,  0;    % T1 前垂推
           0,   0, -0.5,  0, -0.0083,  0;    % T2 后垂推
           0, -0.5,   0,  0,     0, -0.0083; % T3 前侧推
           0, -0.5,   0,  0,     0,  0.0083; % T4 后侧推
           1,   0,    0,  0,     0,      0]; % T5 主推

T_max = [9500, 9500, 9500, 9500, 9500];  % 正向最大推力 (g)
T_min = [7300, 7300, 7300, 7300, 7300];  % 反向最大推力 (g)

% 推进器名称
thr_names = {'T1 前垂推', 'T2 后垂推', 'T3 前侧推', 'T4 后侧推', 'T5 主推'};
thr_labels = {'T1前垂','T2后垂','T3前侧','T4后侧','T5主推'};

% 读取力/力矩指令 (单位: TX,Y,Z=g, MX,Y,Z=N·m)
% CAN协议: TX,TY,TZ=g; MX,MY,MZ接收后×100转g·cm
force_cmd = [T.force_cmd1, T.force_cmd2, T.force_cmd3, ...
             T.force_cmd4, T.force_cmd5, T.force_cmd6];
% 注意: force_cmd1=X(TX), force_cmd2=Y(TY), force_cmd3=Z(TZ)
%        force_cmd4=K(MX), force_cmd5=M(MY), force_cmd6=N(MZ)

% 运动状态
linear_vel  = [T.linear_vel_x, T.linear_vel_y, T.linear_vel_z];
angular_vel = [T.angular_vel_x, T.angular_vel_y, T.angular_vel_z];
attitude = [T.roll, T.pitch, T.yaw];

% 电功率
voltage = T.control_voltage;   % V
current = T.power_current;     % A
power = voltage .* current;    % W

%% 1. 计算各推进器的实际推力指令 (CAN推力分配)
fprintf('=== 1. 推进器推力分配计算 ===\n');
thr_force = zeros(N, 5);  % N×5, 每列为一个推进器的推力 (g)
for i = 1:N
    cmd_6dof = force_cmd(i,:)';  % 6×1
    T_vec = K_thr * cmd_6dof;     % 5×1, 单位g
    thr_force(i,:) = T_vec';
end

% 统计各推进器推力
for i = 1:5
    tf = thr_force(:,i);
    fprintf('%-12s: mean=%8.1fg, std=%8.1fg, max=%8.1fg, min=%8.1fg, nonzero=%.1f%%\n', ...
        thr_names{i}, mean(tf), std(tf), max(tf), min(tf), sum(abs(tf)>1)/N*100);
end

% 等比例限幅检查
fprintf('\n推力限幅检查 (|T| > %.0fg 为饱和):\n', T_max(1));
for i = 1:5
    tf = thr_force(:,i);
    sat_pct = sum(abs(tf) > T_max(i)) / N * 100;
    if sat_pct > 0.01
        fprintf('  %s: %.1f%% 时间饱和\n', thr_names{i}, sat_pct);
    end
end

%% 2. T1故障模拟: 仅后垂推工作的实际效果
fprintf('\n=== 2. T1前垂推故障影响 ===\n');

% 正常情况: T1和T2共同产生 Z 和 MY
% T1 = -0.5*TZ + 0.0083*MY
% T2 = -0.5*TZ - 0.0083*MY
% 实际Z推力 = T1+T2 (都向下), 实际MY力矩 ∝ T1-T2 (差动)

% T1故障(T1=0)时:
% 如果控制器发出[TZ, MY], 实际只有T2执行 = -0.5*TZ - 0.0083*MY
% 有效Z = T2, 有效MY = -(-0.0083)*T2_force...

% 简化分析: 控制器期望 vs 实际执行
T1_normal = thr_force(:,1);  % 前垂推指令 (正常时应有的推力)
T2_actual = thr_force(:,2);  % 后垂推指令 (故障下唯一执行的推力)

% 期望的Z和MY (来自指令空间)
Z_cmd = force_cmd(:,3);      % g
MY_cmd = force_cmd(:,5);     % N·m

% 正常情况下 Z_effective = -(T1+T2) (注意负号: -0.5系数)
% 故障下 Z_effective_fault = T2_actual (仅后垂推)
% 正常情况下 MY_effective ∝ T1*(-x1) + T2*(-x2) 但CAN用对称矩阵
% 简化: 有效Z比例 = T2/(T1+T2) = T2/(T1+T2)
mask_active = abs(T1_normal) > 100 | abs(T2_actual) > 100;
sum_active = sum(mask_active);

% 故障导致的推力损失
if sum_active > 10
    T1_contrib = abs(T1_normal(mask_active));
    T2_contrib = abs(T2_actual(mask_active));
    fraction_T2 = T2_contrib ./ (T1_contrib + T2_contrib + 1e-6);
    fprintf('活跃期间 T2 贡献占比: mean=%.1f%%, median=%.1f%%\n', ...
        mean(fraction_T2)*100, median(fraction_T2)*100);

    % Z向总推力损失
    Z_total_normal = abs(T1_normal(mask_active)) + abs(T2_actual(mask_active));
    Z_total_fault  = abs(T2_actual(mask_active));
    Z_loss_pct = (1 - Z_total_fault ./ (Z_total_normal + 1e-6)) * 100;
    fprintf('Z向推力损失: mean=%.1f%%\n', mean(Z_loss_pct));
end

%% 3. 推进器推力 vs 电功率 关系
fprintf('\n=== 3. 各推进器推力 vs 电功率 ===\n');

% 总推力幅值 (所有推进器推力的RMS)
total_thrust_g = sqrt(sum(thr_force.^2, 2));

% 功率 vs 总推力
r_pt = corr(total_thrust_g, power);
fprintf('总推力幅值 vs 电功率: r = %.3f\n', r_pt);

% 各通道独立相关
for i = 1:5
    r = corr(abs(thr_force(:,i)), power);
    fprintf('  %s |T| vs 电功率: r = %.3f\n', thr_names{i}, r);
end

% 按总推力分段统计功率
fprintf('\n推力-功率分段:\n');
edges_g = [0, 500, 1000, 2000, 3000, 5000, 15000];
for e = 1:length(edges_g)-1
    mask = total_thrust_g > edges_g(e) & total_thrust_g <= edges_g(e+1);
    if sum(mask) > 10
        fprintf('  推力 %5.0f-%5.0fg: N=%5d, P_mean=%5.1fW, I_mean=%.2fA\n', ...
            edges_g(e), edges_g(e+1), sum(mask), mean(power(mask)), mean(current(mask)));
    end
end

%% 4. 推进器推力 → 运动响应
fprintf('\n=== 4. 推力 → 速度响应 ===\n');

% T5(主推) → u (前进速度)
r_t5u = corr(thr_force(:,5), linear_vel(:,1));
% T3+T4(侧推) → v (侧移速度)
r_side_v = corr(thr_force(:,3)+thr_force(:,4), linear_vel(:,2));
% T2(后垂推,故障下唯一) → w (垂向速度)
r_t2w = corr(thr_force(:,2), linear_vel(:,3));
% T3,T4差动 → r (转艏角速度)
r_side_r = corr(thr_force(:,3)-thr_force(:,4), angular_vel(:,3));

fprintf('T5主推     → u: r = %.3f\n', r_t5u);
fprintf('T3+T4侧推  → v: r = %.3f\n', r_side_v);
fprintf('T2后垂推   → w: r = %.3f (仅后垂推工作)\n', r_t2w);
fprintf('T3-T4差动  → r: r = %.3f\n', r_side_r);

% 对比: 6-DOF指令 vs 推进器级推力
fprintf('\n6-DOF指令 vs 推进器推力 相关对比:\n');
fprintf('  Fx→u: %.3f  |  T5→u: %.3f\n', corr(force_cmd(:,1), linear_vel(:,1)), r_t5u);
fprintf('  Fy→v: %.3f  |  T3+T4→v: %.3f\n', corr(force_cmd(:,2), linear_vel(:,2)), r_side_v);
fprintf('  Nz→r: %.3f  |  T3-T4→r: %.3f\n', corr(force_cmd(:,6), angular_vel(:,3)), r_side_r);

%% 5. 机械功率 vs 电功率 (效率估算)
fprintf('\n=== 5. 推进效率估算 ===\n');

% 机械功率 = 推力(N) × 速度(m/s)
% 推力转换: g → N, 乘以 0.001*9.81 = 0.00981
g2N = 0.00981;

% 各推进器产生的机械功率
% T5主推: thrust * u (前进方向)
mech_T5 = abs(thr_force(:,5) * g2N .* linear_vel(:,1));
% 侧推: thrust * v (侧向)
mech_side = abs((thr_force(:,3)+thr_force(:,4)) * g2N .* linear_vel(:,2));
% 垂推: thrust * w (垂向)
mech_vert = abs(thr_force(:,2) * g2N .* linear_vel(:,3));  % 仅T2

total_mech = mech_T5 + mech_side + mech_vert;
P_base = min(power);  % 基础功耗 (待机)
P_excess = power - P_base;
P_excess(P_excess < 0) = 0;

% 仅考虑有推力输出时的效率
mask_thrust = total_thrust_g > 500;
if sum(mask_thrust) > 100
    eff_inst = total_mech(mask_thrust) ./ power(mask_thrust) * 100;
    eff_excess = total_mech(mask_thrust) ./ P_excess(mask_thrust) * 100;
    fprintf('有推力期间 (%d点):\n', sum(mask_thrust));
    fprintf('  平均电功率: %.1f W\n', mean(power(mask_thrust)));
    fprintf('  平均机械功率: %.2f W\n', mean(total_mech(mask_thrust)));
    fprintf('  总效率 (机械/电): mean=%.1f%%, max=%.1f%%\n', mean(eff_inst), max(eff_inst));
    fprintf('  超额效率 (机械/(P-Pbase)): mean=%.1f%%, max=%.1f%%\n', mean(eff_excess), max(eff_excess));
end

%% 6. 绘制综合报告图
figure('Position', [50 50 1600 1000]);

% 6a. 五推进器推力时序
subplot(3,4,1);
plot(t/60, thr_force);
xlabel('时间 (min)'); ylabel('推力 (g)'); title('五推进器推力指令');
legend(thr_labels, 'Location', 'best'); grid on;

% 6b. 推进器推力分布直方图
subplot(3,4,2);
hold on;
colors = lines(5);
for i = 1:5
    histogram(thr_force(:,i), 50, 'FaceAlpha', 0.3, 'DisplayName', thr_labels{i});
end
xlabel('推力 (g)'); ylabel('频次'); title('推力分布');
legend('Location', 'best'); grid on;

% 6c. 电功率时序
subplot(3,4,3);
yyaxis left; plot(t/60, power); ylabel('功率 (W)');
yyaxis right; plot(t/60, current); ylabel('电流 (A)');
xlabel('时间 (min)'); title('电功率与电流');
grid on;

% 6d. 总推力 vs 电功率散点
subplot(3,4,4);
scatter(total_thrust_g, power, 3, 'filled', 'MarkerFaceAlpha', 0.2);
xlabel('总推力幅值 (g)'); ylabel('电功率 (W)');
title(sprintf('总推力 vs 电功率 (r=%.3f)', r_pt));
grid on;

% 6e. T5主推 vs u
subplot(3,4,5);
scatter(thr_force(:,5), linear_vel(:,1), 3, 'filled', 'MarkerFaceAlpha', 0.15);
xlabel('T5 主推推力 (g)'); ylabel('u (m/s)');
title(sprintf('主推 vs 前进速度 (r=%.3f)', r_t5u));
grid on;

% 6f. T3+T4侧推 vs v
subplot(3,4,6);
scatter(thr_force(:,3)+thr_force(:,4), linear_vel(:,2), 3, 'filled', 'MarkerFaceAlpha', 0.15);
xlabel('T3+T4 侧推力 (g)'); ylabel('v (m/s)');
title(sprintf('侧推 vs 侧移速度 (r=%.3f)', r_side_v));
grid on;

% 6g. T2后垂推 vs w
subplot(3,4,7);
scatter(thr_force(:,2), linear_vel(:,3), 3, 'filled', 'MarkerFaceAlpha', 0.15);
xlabel('T2 后垂推力 (g)'); ylabel('w (m/s)');
title(sprintf('后垂推 vs 垂向速度 (r=%.3f)', r_t2w));
grid on;

% 6h. T3-T4差动 vs r (yaw)
subplot(3,4,8);
scatter(thr_force(:,3)-thr_force(:,4), angular_vel(:,3), 3, 'filled', 'MarkerFaceAlpha', 0.15);
xlabel('T3-T4 差动推力 (g)'); ylabel('r (deg/s)');
title(sprintf('侧推差动 vs 转艏角速度 (r=%.3f)', r_side_r));
grid on;

% 6i. T1(前垂推指令) vs T2(后垂推指令)
subplot(3,4,9);
scatter(T1_normal, T2_actual, 3, 'filled', 'MarkerFaceAlpha', 0.15);
xlabel('T1 前垂推指令 (g)'); ylabel('T2 后垂推指令 (g)');
title(sprintf('前后垂推指令 (r=%.3f)', corr(T1_normal, T2_actual)));
grid on;

% 6j. 机械功率 vs 电功率
subplot(3,4,10);
scatter(power, total_mech, 3, 'filled', 'MarkerFaceAlpha', 0.2);
xlabel('电功率 (W)'); ylabel('机械功率 (W)');
title('电功率 vs 机械功率');
grid on;

% 6k. 姿态角时序
subplot(3,4,11);
plot(t/60, attitude);
xlabel('时间 (min)'); ylabel('姿态角 (deg)');
title('姿态角'); legend('Roll','Pitch','Yaw','Location','best'); grid on;

% 6l. 推进器推力占比 (饼图)
subplot(3,4,12);
thr_rms = sqrt(mean(thr_force.^2, 1));
pie(thr_rms, thr_labels);
title('各推进器 RMS 推力占比');

sgtitle('AUV 实验数据分析: CAN协议推力分配 + 前垂推故障影响');
saveas(gcf, fullfile(out_dir, 'thrust_analysis.png'));
fprintf('\n图表1已保存: %s/thrust_analysis.png\n', out_dir);

%% 7. 功率分解图
figure('Position', [100 100 1400 500]);

subplot(1,3,1);
% 各推进器推力绝对值之和 vs 功率
abs_thrust_sum = sum(abs(thr_force), 2);
scatter(abs_thrust_sum, power, 5, 'filled', 'MarkerFaceAlpha', 0.2);
xlabel('|推力|之和 (g)'); ylabel('电功率 (W)');
title(sprintf('推力绝对值之和 vs 电功率 (r=%.3f)', corr(abs_thrust_sum, power)));
grid on;

subplot(1,3,2);
% 按时间窗口的效率趋势
win = 500;  % 窗口大小
n_win = floor(N/win);
eff_trend = zeros(n_win, 1);
t_trend = zeros(n_win, 1);
p_trend = zeros(n_win, 1);
for w = 1:n_win
    idx = (w-1)*win+1 : w*win;
    mask = total_thrust_g(idx) > 100;
    if sum(mask) > 10
        eff_trend(w) = mean(total_mech(idx(mask))) / mean(power(idx(mask))) * 100;
    else
        eff_trend(w) = NaN;
    end
    t_trend(w) = mean(t(idx)) / 60;
    p_trend(w) = mean(power(idx));
end
yyaxis left; plot(t_trend, eff_trend, 'b-o', 'MarkerSize', 3);
ylabel('推进效率 (%)');
yyaxis right; plot(t_trend, p_trend, 'r-');
ylabel('平均功率 (W)'); xlabel('时间 (min)');
title('推进效率时序 (滑动窗口)');
grid on;

subplot(1,3,3);
% 各推进器推力 vs 电功率 相关系数条状图
r_thr_power = zeros(5,1);
for i = 1:5
    r_thr_power(i) = corr(abs(thr_force(:,i)), power);
end
bar(r_thr_power);
set(gca, 'XTickLabel', thr_labels);
ylabel('|推力| vs 电功率 相关系数');
title('各推进器对功耗的贡献');
grid on;

sgtitle('功率分解与效率分析');
saveas(gcf, fullfile(out_dir, 'power_analysis.png'));
fprintf('图表2已保存: %s/power_analysis.png\n', out_dir);

fprintf('\n分析完成.\n');
