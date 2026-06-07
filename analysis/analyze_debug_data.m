% analyze_debug_data.m — 分析 debug_data260530-1.csv
% 聚焦：六自由度力/力矩输入、功率和实际推进器作用效果的关系
clear; clc;

% === 读取数据 ===
T = readtable('data/raw/debug_data260530-1.csv');

% 提取关键列
t = T.pc_timestamp - T.pc_timestamp(1);  % 相对时间 (s)
force_cmd = [T.force_cmd1, T.force_cmd2, T.force_cmd3, ...
             T.force_cmd4, T.force_cmd5, T.force_cmd6];  % 六自由度力/力矩指令
voltage = T.control_voltage;           % 控制电压 (V)
current = T.power_current;             % 功率电流 (A)
power = voltage .* current;            % 总电功率 (W)

% 运动状态
linear_vel = [T.linear_vel_x, T.linear_vel_y, T.linear_vel_z];    % 线速度 (m/s)
angular_vel = [T.angular_vel_x, T.angular_vel_y, T.angular_vel_z]; % 角速度 (deg/s)
attitude = [T.roll, T.pitch, T.yaw];                               % 姿态角 (deg)

N = length(t);
fprintf('数据点数: %d, 时间跨度: %.1f s (%.1f min)\n', N, t(end), t(end)/60);

% === 1. 基本统计 ===
fprintf('\n========== 1. 力/力矩指令基本统计 ==========\n');
dof_names = {'Surge X (N)', 'Sway Y (N)', 'Heave Z (N)', ...
             'Roll K (Nm)', 'Pitch M (Nm)', 'Yaw N (Nm)'};
for i = 1:6
    d = force_cmd(:,i);
    fprintf('%-15s: mean=%8.1f, std=%8.1f, min=%8.1f, max=%8.1f, nonzero=%.1f%%\n', ...
        dof_names{i}, mean(d), std(d), min(d), max(d), sum(d~=0)/N*100);
end
fprintf('\n功率: 均值=%.1fW, 峰值=%.1fW, 最小=%.1fW\n', mean(power), max(power), min(power));
fprintf('电流: 均值=%.2fA, 峰值=%.2fA, 最小=%.2fA\n', mean(current), max(current), min(current));

% === 2. 力指令与运动响应之间的相关性 ===
fprintf('\n========== 2. 力/力矩指令 vs 速度响应 相关系数 ==========\n');

vel_labels = {'u (m/s)', 'v (m/s)', 'w (m/s)', 'p (deg/s)', 'q (deg/s)', 'r (deg/s)'};
fprintf('                ');
for j = 1:6
    fprintf('%15s', vel_labels{j});
end
fprintf('\n');
all_vel = [linear_vel, angular_vel];  % 6列：u,v,w,p,q,r
for i = 1:6
    fprintf('%-15s', dof_names{i});
    for j = 1:6
        r = corr(force_cmd(:,i), all_vel(:,j));
        fprintf('%15.3f', r);
    end
    fprintf('\n');
end

% === 3. 功率与推力的关系 ===
fprintf('\n========== 3. 功率 vs 力指令 相关系数 ==========\n');
for i = 1:6
    r = corr(force_cmd(:,i), power);
    fprintf('P vs %-15s: r = %+.3f\n', dof_names{i}, r);
end

% 功率 vs 合推力（各通道平方和的平方根）
total_force = sqrt(sum(force_cmd.^2, 2));
r_pf = corr(total_force, power);
fprintf('P vs 合推力幅值: r = %+.3f\n', r_pf);

% === 4. 分时段分析：找到力指令活跃的时段 ===
fprintf('\n========== 4. 各通道活跃时段识别 ==========\n');
for i = 1:6
    mask = abs(force_cmd(:,i)) > 100;  % 认为 |cmd|>100 为活跃
    active_samples = sum(mask);
    if active_samples > 100
        fprintf('%s: %d 个活跃点 (%.1f%%), 活跃时均值=%.1f, 活跃时 std=%.1f\n', ...
            dof_names{i}, active_samples, active_samples/N*100, ...
            mean(force_cmd(mask,i)), std(force_cmd(mask,i)));
    else
        fprintf('%s: 基本不活跃 (活跃点=%d)\n', dof_names{i}, active_samples);
    end
end

% === 5. 功率分段分析 ===
fprintf('\n========== 5. 功率 vs 合推力 分段统计 ==========\n');
% 按 total_force 分段
edges = [-inf, 100, 500, 1000, 2000, 5000, 10000, inf];
for i = 1:length(edges)-1
    mask = total_force > edges(i) & total_force <= edges(i+1);
    if sum(mask) > 10
        fprintf('合推力(%6.0f,%6.0f]: N=%5d, P_mean=%5.1fW, P_std=%4.1fW, I_mean=%.2fA\n', ...
            edges(i), edges(i+1), sum(mask), mean(power(mask)), std(power(mask)), mean(current(mask)));
    end
end

% === 6. 绘制关键图表 ===
figure('Position', [100 100 1400 900]);

% --- 6a. 六自由度力指令时序 ---
subplot(3,3,1);
plot(t, force_cmd(:,1:3));
xlabel('时间 (s)'); ylabel('力指令 (N)'); title('平动力指令 (Surge/Sway/Heave)');
legend('X','Y','Z', 'Location','best'); grid on;

subplot(3,3,2);
plot(t, force_cmd(:,4:6));
xlabel('时间 (s)'); ylabel('力矩指令 (Nm)'); title('转动力矩指令 (Roll/Pitch/Yaw)');
legend('K','M','N', 'Location','best'); grid on;

% --- 6b. 功率与电流 ---
subplot(3,3,3);
yyaxis left; plot(t, power); ylabel('功率 (W)');
yyaxis right; plot(t, current); ylabel('电流 (A)');
xlabel('时间 (s)'); title('电功率与电流时序');
grid on;

% --- 6c. 线速度响应 ---
subplot(3,3,4);
plot(t, linear_vel);
xlabel('时间 (s)'); ylabel('线速度 (m/s)'); title('线速度响应');
legend('u','v','w', 'Location','best'); grid on;

% --- 6d. 角速度响应 ---
subplot(3,3,5);
plot(t, angular_vel);
xlabel('时间 (s)'); ylabel('角速度 (deg/s)'); title('角速度响应');
legend('p','q','r', 'Location','best'); grid on;

% --- 6e. 姿态角 ---
subplot(3,3,6);
plot(t, attitude);
xlabel('时间 (s)'); ylabel('姿态角 (deg)'); title('姿态角');
legend('roll','pitch','yaw', 'Location','best'); grid on;

% --- 6f. 功率 vs 合推力散点图 ---
subplot(3,3,7);
scatter(total_force, power, 3, 'filled', 'MarkerFaceAlpha', 0.3);
xlabel('合推力幅值'); ylabel('功率 (W)'); title('功率 vs 合推力图');
grid on;

% --- 6g. Surge力 vs Surge速度 ---
subplot(3,3,8);
scatter(force_cmd(:,1), linear_vel(:,1), 3, 'filled', 'MarkerFaceAlpha', 0.3);
xlabel('Surge 力指令 (N)'); ylabel('u (m/s)'); title('Surge 力 vs 前进速度');
grid on;

% --- 6h. Yaw力矩 vs Yaw角速度 ---
subplot(3,3,9);
scatter(force_cmd(:,6), angular_vel(:,3), 3, 'filled', 'MarkerFaceAlpha', 0.3);
xlabel('Yaw 力矩指令 (Nm)'); ylabel('r (deg/s)'); title('Yaw 力矩 vs 转艏角速度');
grid on;

sgtitle('AUV 实验数据分析: 力/力矩输入-功率-运动响应');
saveas(gcf, 'assets/figures/debug_data_analysis.png');
fprintf('\n图表已保存: assets/figures/debug_data_analysis.png\n');

% === 7. 推进效率分析 ===
fprintf('\n========== 7. 推进效率分析 ==========\n');
% Surge方向：力→速度的稳态关系
mask_surge = abs(force_cmd(:,1)) > 100 & abs(linear_vel(:,1)) > 0.01;
if sum(mask_surge) > 50
    % 线性拟合 u = k * F_x + b
    X = [force_cmd(mask_surge,1), ones(sum(mask_surge),1)];
    beta = X \ linear_vel(mask_surge,1);
    fprintf('Surge 力→速度 线性模型: u = %.6f * F_x + %.4f\n', beta(1), beta(2));

    % 功率效率
    p_surge = power(mask_surge);
    p_base = min(power);  % 基础功耗
    useful_power = mean(abs(force_cmd(mask_surge,1) .* linear_vel(mask_surge,1)));  % F*v
    total_excess = mean(p_surge - p_base);
    fprintf('  有用推进功率 ≈ %.2f W, 超额电功率 ≈ %.2f W, 推进效率 ≈ %.1f%%\n', ...
        useful_power, total_excess, useful_power/total_excess*100);
end

% Yaw方向
mask_yaw = abs(force_cmd(:,6)) > 100 & abs(angular_vel(:,3)) > 1;
if sum(mask_yaw) > 50
    X = [force_cmd(mask_yaw,6), ones(sum(mask_yaw),1)];
    beta = X \ angular_vel(mask_yaw,3);
    fprintf('Yaw 力矩→角速度 线性模型: r = %.6f * N + %.4f\n', beta(1), beta(2));
end

% === 8. 交叉耦合定性分析 ===
fprintf('\n========== 8. 交叉耦合分析 ==========\n');
% Surge指令 ≠ 0 时，检查是否引起 Yaw 速度
mask_surge_only = abs(force_cmd(:,1)) > 500 & abs(force_cmd(:,6)) < 50;
mask_yaw_only   = abs(force_cmd(:,6)) > 500 & abs(force_cmd(:,1)) < 50;
fprintf('纯 Surge 指令期间 (|Fx|>500, |Nz|<50): %d 点\n', sum(mask_surge_only));
if sum(mask_surge_only) > 50
    fprintf('  平均 r (yaw角速度) = %.3f deg/s (应由0)\n', mean(angular_vel(mask_surge_only,3)));
    fprintf('  平均 v (sway速度)  = %.3f m/s\n', mean(linear_vel(mask_surge_only,2)));
end
fprintf('纯 Yaw 指令期间 (|Nz|>500, |Fx|<50): %d 点\n', sum(mask_yaw_only));
if sum(mask_yaw_only) > 50
    fprintf('  平均 u (surge速度) = %.3f m/s (应由0)\n', mean(linear_vel(mask_yaw_only,1)));
end

fprintf('\n分析完成.\n');
