% analyze_debug_data_v2.m — 考虑前垂推故障的补充分析
clear; clc;

T = readtable('data/debug_data260530-1.csv');

t = T.pc_timestamp - T.pc_timestamp(1);
force_cmd = [T.force_cmd1, T.force_cmd2, T.force_cmd3, ...
             T.force_cmd4, T.force_cmd5, T.force_cmd6];
voltage = T.control_voltage;
current = T.power_current;
power = voltage .* current;

linear_vel  = [T.linear_vel_x, T.linear_vel_y, T.linear_vel_z];
angular_vel = [T.angular_vel_x, T.angular_vel_y, T.angular_vel_z];
attitude = [T.roll, T.pitch, T.yaw];

N = length(t);
x_vr = -0.293;  % 后垂推 x 位置 (m)，前垂推失效

fprintf('========== 前垂推故障影响分析 ==========\n');
fprintf('推进器布局:\n');
fprintf('  前垂推 x=+0.344m (故障!!!)  后垂推 x=%.3fm (唯一工作)\n', x_vr);
fprintf('  侧推 x=+0.424, -0.376m      主推 x=0\n');

%% 1. 故障后果：Z 和 M 的强制耦合关系
fprintf('\n--- 1. 前垂推故障导致的 Z-M 强制耦合 ---\n');
fprintf('理论关系: 后垂推单独工作时, M = %.3f * Z\n', abs(x_vr));
fprintf('         (后垂推在CG后方%.3fm, 产生的力同时贡献Z和M)\n', abs(x_vr));

mask_active = abs(force_cmd(:,3)) > 100 | abs(force_cmd(:,5)) > 100;
fprintf('垂向通道活跃点数: %d (%.1f%%)\n', sum(mask_active), sum(mask_active)/N*100);

% Z-M 相关性
r_zm = corr(force_cmd(:,3), force_cmd(:,5));
fprintf('Z-M 指令相关系数: r = %.4f (应接近±1)\n', r_zm);

% 活跃时段的 Z/M 比值
mask_both = abs(force_cmd(:,3)) > 500 & abs(force_cmd(:,5)) > 500;
if sum(mask_both) > 50
    ratio_zm = force_cmd(mask_both,5) ./ force_cmd(mask_both,3);
    fprintf('活跃点 Z/M 比值: 均值=%.3f, 中位数=%.3f, std=%.3f\n', ...
        mean(ratio_zm), median(ratio_zm), std(ratio_zm));
    fprintf('理论值 M/Z = %.3f\n', abs(x_vr));
end

%% 2. 对比正常情况（仿真）vs 实际（前垂推故障）
fprintf('\n--- 2. 理论对比: 双垂推 vs 单垂推 ---\n');
fprintf('正常(双垂推): Z和M可独立控制, 伪逆求解 2个自由度\n');
fprintf('故障(单垂推): Z和M被锁死为 M = %.3f * Z, 控制自由度降为1\n', abs(x_vr));
fprintf('  → 无法同时实现期望的深度和俯仰\n');
fprintf('  → 控制器可能陷入 Z/M 矛盾: 指令Z和指令M不满足 1:%.3f 的比例\n', abs(x_vr));

% 检查指令 Z 和 M 是否满足耦合关系
if sum(mask_both) > 50
    M_predicted_from_Z = abs(x_vr) * force_cmd(mask_both,3);
    M_actual = force_cmd(mask_both,5);
    mismatch = M_actual - M_predicted_from_Z;
    fprintf('\n指令矛盾分析:\n');
    fprintf('  实际 M 与 Z*%.3f 的差异: 均值=%.0f, std=%.0f\n', ...
        abs(x_vr), mean(mismatch), std(mismatch));
    fprintf('  矛盾占比 = %.1f%% (非零表示控制器发出无法执行的指令)\n', ...
        sum(abs(mismatch) > 100) / sum(mask_both) * 100);
end

%% 3. 实际运动响应的耦合分析
fprintf('\n--- 3. 垂荡-俯仰运动耦合 ---\n');

% w (heave速度) vs q (俯仰角速度) 的相关性
r_wq = corr(linear_vel(:,3), angular_vel(:,2));
fprintf('w (垂向速度) vs q (俯仰角速度) 相关系数: r = %.3f\n', r_wq);
fprintf('  正常解耦时期望 r ≈ 0, 故障下应显著增大\n');

% 推力→运动 的完整关系
fprintf('\n实际推力→运动关系 (Z_cmd通过后垂推产生):\n');
mask_z = abs(force_cmd(:,3)) > 200;
if sum(mask_z) > 50
    r_zw = corr(force_cmd(mask_z,3), linear_vel(mask_z,3));
    r_zq = corr(force_cmd(mask_z,3), angular_vel(mask_z,2));
    fprintf('  Z_cmd → w: r = %.3f\n', r_zw);
    fprintf('  Z_cmd → q: r = %.3f (正常应≈0, 故障下因耦合不应≈0)\n', r_zq);
end

mask_m = abs(force_cmd(:,5)) > 200;
if sum(mask_m) > 50
    r_mw = corr(force_cmd(mask_m,5), linear_vel(mask_m,3));
    r_mq = corr(force_cmd(mask_m,5), angular_vel(mask_m,2));
    fprintf('  M_cmd → w: r = %.3f (正常应≈0, 故障下因耦合不应≈0)\n', r_mw);
    fprintf('  M_cmd → q: r = %.3f\n', r_mq);
end

%% 4. 深度和俯仰综合效果
fprintf('\n--- 4. 深度/俯仰同时响应 ---\n');
depth = T.depth_lpf;
pitch_deg = T.pitch;

mask_active_z = abs(force_cmd(:,3)) > 300;
if sum(mask_active_z) > 50
    % Z 指令期间深度变化率
    dt = [diff(t); 0.1];
    depth_rate = [diff(depth); 0] ./ dt;
    r_zd = corr(force_cmd(mask_active_z,3), depth_rate(mask_active_z));
    r_mp = corr(force_cmd(mask_active_z,5), angular_vel(mask_active_z,2));
    fprintf('Z_cmd → 深度变化率: r = %.3f\n', r_zd);
    fprintf('M_cmd → 俯仰角速度: r = %.3f\n', r_mp);
end

%% 5. 对比：侧推通道（正常，双推进器）vs 垂推通道（故障，单推进器）
fprintf('\n--- 5. 侧推(正常) vs 垂推(故障)对比 ---\n');
fprintf('              Sway(Y)  Yaw(N)   Heave(Z)  Pitch(M)\n');

% Sway vs Yaw 相关性 (两个侧推都在工作)
r_yn = corr(force_cmd(:,2), force_cmd(:,6));
fprintf('指令相关性    %7.3f            %7.3f\n', r_yn, r_zm);
r_yv = corr(force_cmd(:,2), linear_vel(:,2));
r_nr = corr(force_cmd(:,6), angular_vel(:,3));
fprintf('指令→主响应  %7.3f  %7.3f  %7.3f  (正常接近0.5)\n', r_yv, r_nr, NaN);

% 效率对比
fprintf('\n通道效率对比 (相关系数):\n');
fprintf('  Surge (主推):      Fx→u  r = %.3f\n', corr(force_cmd(:,1), linear_vel(:,1)));
fprintf('  Sway  (双侧推):     Fy→v  r = %.3f  ← 正常, 双侧推工作\n', corr(force_cmd(:,2), linear_vel(:,2)));
fprintf('  Yaw   (双侧推):     Nz→r  r = %.3f  ← 最强响应\n', corr(force_cmd(:,6), angular_vel(:,3)));
fprintf('  Heave (仅后垂推):   Fz→w  r = %.3f  ← 效率减半\n', corr(force_cmd(:,3), linear_vel(:,3)));
fprintf('  Pitch (仅后垂推):   My→q  r = %.3f  ← 混入垂荡分量\n', corr(force_cmd(:,5), angular_vel(:,2)));

%% 6. 绘制故障影响图
figure('Position', [100 100 1200 800]);

% 6a. Z_cmd vs M_cmd 散点图（验证强制耦合）
subplot(2,3,1);
scatter(force_cmd(:,3), force_cmd(:,5), 3, 'filled', 'MarkerFaceAlpha', 0.2);
hold on;
% 理论耦合线
z_range = [-8000, 8000];
plot(z_range, abs(x_vr)*z_range, 'r--', 'LineWidth', 1.5);
plot(z_range, -abs(x_vr)*z_range, 'r--', 'LineWidth', 1.5);
xlabel('Heave Z 指令'); ylabel('Pitch M 指令');
title('Z vs M 指令 (红线=理论耦合 M=0.293*Z)');
grid on; axis equal;

% 6b. w vs q 运动耦合
subplot(2,3,2);
scatter(linear_vel(:,3), angular_vel(:,2), 3, 'filled', 'MarkerFaceAlpha', 0.2);
xlabel('w (垂向速度 m/s)'); ylabel('q (俯仰角速度 deg/s)');
title(sprintf('垂荡-俯仰运动耦合 (r=%.3f)', r_wq));
grid on;

% 6c. 时序：Z/M 指令
subplot(2,3,3);
yyaxis left; plot(t, force_cmd(:,3)); ylabel('Z 指令');
yyaxis right; plot(t, force_cmd(:,5)); ylabel('M 指令');
xlabel('时间 (s)'); title('Z 和 M 指令时序 (应成比例)');
grid on;

% 6d. Z_cmd→w 散点
subplot(2,3,4);
scatter(force_cmd(:,3), linear_vel(:,3), 3, 'filled', 'MarkerFaceAlpha', 0.15);
xlabel('Z 指令'); ylabel('w (m/s)');
title(sprintf('Z→w (单垂推, r=%.3f)', corr(force_cmd(:,3), linear_vel(:,3))));
grid on;

% 6e. Z_cmd→q 散点 (耦合)
subplot(2,3,5);
scatter(force_cmd(:,3), angular_vel(:,2), 3, 'filled', 'MarkerFaceAlpha', 0.15);
xlabel('Z 指令'); ylabel('q (deg/s)');
title(sprintf('Z→q (故障耦合, r=%.3f)', corr(force_cmd(:,3), angular_vel(:,2))));
grid on;

% 6f. 各通道指令→主响应 相关系数对比
subplot(2,3,6);
channels = {'Surge\nFx→u','Sway\nFy→v','Heave\nFz→w','Pitch\nMy→q','Yaw\nNz→r'};
r_values = [
    corr(force_cmd(:,1), linear_vel(:,1));
    corr(force_cmd(:,2), linear_vel(:,2));
    corr(force_cmd(:,3), linear_vel(:,3));
    corr(force_cmd(:,5), angular_vel(:,2));
    corr(force_cmd(:,6), angular_vel(:,3));
];
bar(r_values);
ylabel('相关系数 r'); title('各通道指令→主响应效率对比');
set(gca, 'XTickLabel', channels);
grid on;

% 标注故障通道
hold on;
h1 = bar(3, r_values(3), 'r');
h2 = bar(4, r_values(4), 'r');
legend([h1], '前垂推故障影响', 'Location', 'southeast');

sgtitle('前垂推故障影响分析: Z-M 强制耦合');
saveas(gcf, 'pic/debug_fault_analysis.png');
fprintf('\n图表已保存: pic/debug_fault_analysis.png\n');

%% 7. 修正后的推进效率估算
fprintf('\n--- 7. 修正后的推进效率 ---\n');
fprintf('前垂推故障导致:\n');
fprintf('  1. Z向最大推力减半 (仅后垂推工作)\n');
fprintf('  2. M向最大力矩减半\n');
fprintf('  3. Z和M无法独立控制 → 同时影响深度和俯仰\n');
fprintf('  4. 耦合关系: 任意垂向推力必伴随 M = %.3f * Z 的俯仰力矩\n', abs(x_vr));
fprintf('  5. 控制器若同时输出Z和M且不满足比例, 实际执行效果将偏离指令\n');

fprintf('\n分析完成.\n');
