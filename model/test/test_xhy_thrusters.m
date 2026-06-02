% 测试XHY垂推和侧推的稳态速度、角速度和力矩
% 更新: 使用新推力模型(辅推D=6cm) + CFD阻力模型
% 测试4种工况:
% 1. 垂推同向 → 最大沉浮速度w (注: 正常双垂推)
% 2. 垂推反向 → 稳态俯仰角theta (浮力恢复力矩平衡, q=0)
% 3. 侧推同向 → 最大横荡速度v
% 4. 侧推反向 → 最大偏航角速度r (无恢复力矩)

addpath('.', './Lib', './guidance', './controller/xhy', './controller/remus', './model', './eso', './post', './traj');

%% 参数设置
rho = 1026;
n_max  = 2500;
n_steps = 26;

n_test = linspace(0, n_max, n_steps);

T_sim = 200;  % 仿真时长 (s)
h = 0.05;     % 时间步长 (s)

Vc = 0; betaVc = 0; w_c = 0;

%% 工况1：垂推同向 → 最大沉浮速度w
fprintf('=== 工况1: 垂推同向 (最大沉浮速度) ===\n');
w_steady = zeros(size(n_test));
T_vert_arr = zeros(size(n_test));

for k = 1:length(n_test)
    n_vert = n_test(k);
    ui = [0; n_vert; n_vert; 0; 0];

    x = zeros(12, 1);
    for t = 0:h:T_sim
        xdot = xhy(x, ui, Vc, betaVc, w_c);
        x = x + h * xdot;
    end
    w_steady(k) = x(3);
    T_vert_arr(k) = 2 * thrust_aux(n_vert, rho);
end

%% 工况2：垂推反向 → 稳态俯仰角theta (浮力恢复, q→0)
fprintf('=== 工况2: 垂推反向 (稳态俯仰角) ===\n');
theta_steady = zeros(size(n_test));
M_pitch_arr = zeros(size(n_test));

x_vert_f = 0.344;   % 前垂推x位置 (m)
x_vert_r = -0.293;  % 后垂推x位置 (m)

for k = 1:length(n_test)
    n_vert = n_test(k);
    ui = [0; n_vert; -n_vert; 0; 0];

    x = zeros(12, 1);
    for t = 0:h:T_sim
        xdot = xhy(x, ui, Vc, betaVc, w_c);
        x = x + h * xdot;
    end
    theta_steady(k) = x(11);  % 稳态俯仰角 (rad)

    T_vert = thrust_aux(n_vert, rho);
    M_pitch_arr(k) = -T_vert * x_vert_f - (-T_vert) * x_vert_r;
end

%% 工况3：侧推同向 → 最大横荡速度v
fprintf('=== 工况3: 侧推同向 (最大横荡速度) ===\n');
v_steady = zeros(size(n_test));
T_side_arr = zeros(size(n_test));

for k = 1:length(n_test)
    n_side = n_test(k);
    ui = [0; 0; 0; n_side; n_side];

    x = zeros(12, 1);
    for t = 0:h:T_sim
        xdot = xhy(x, ui, Vc, betaVc, w_c);
        x = x + h * xdot;
    end
    v_steady(k) = x(2);
    T_side_arr(k) = 2 * thrust_aux(n_side, rho);
end

%% 工况4：侧推反向 → 最大偏航角速度r (无恢复力矩)
fprintf('=== 工况4: 侧推反向 (最大偏航角速度) ===\n');
r_steady = zeros(size(n_test));
N_yaw_arr = zeros(size(n_test));

x_side_f = 0.424;   % 前侧推x位置 (m)
x_side_r = -0.376;  % 后侧推x位置 (m)

for k = 1:length(n_test)
    n_side = n_test(k);
    ui = [0; 0; 0; n_side; -n_side];

    x = zeros(12, 1);
    for t = 0:h:T_sim
        xdot = xhy(x, ui, Vc, betaVc, w_c);
        x = x + h * xdot;
    end
    r_steady(k) = x(6);

    T_side = thrust_aux(n_side, rho);
    N_yaw_arr(k) = T_side * x_side_f + (-T_side) * x_side_r;
end

%% 绘图1: 线速度
fig1 = figure('Position', [100, 100, 1400, 500]);

subplot(1, 3, 1);
plot(n_test, w_steady, 'b-o', 'LineWidth', 1.5, 'MarkerSize', 4);
xlabel('垂推转速 (RPM)'); ylabel('稳态沉浮速度 w (m/s)');
title(sprintf('垂推同向→沉浮速度 (max %.3f m/s)', max(w_steady)));
grid on;

subplot(1, 3, 2);
plot(n_test, v_steady, 'g-o', 'LineWidth', 1.5, 'MarkerSize', 4);
xlabel('侧推转速 (RPM)'); ylabel('稳态横荡速度 v (m/s)');
title(sprintf('侧推同向→横荡速度 (max %.3f m/s)', max(v_steady)));
grid on;

subplot(1, 3, 3);
plot(T_vert_arr, w_steady, 'b-s', 'LineWidth', 1.5, 'MarkerSize', 4); hold on;
plot(T_side_arr, v_steady, 'g-s', 'LineWidth', 1.5, 'MarkerSize', 4);
xlabel('合力 (N)'); ylabel('稳态线速度 (m/s)');
title('推力 vs 线速度');
legend('垂推→w', '侧推→v', 'Location', 'northwest');
grid on;

sgtitle('XHY 垂推/侧推 稳态线速度 (辅推D=6cm, CFD阻力)');

pic_dir = 'pic';
if ~exist(pic_dir, 'dir'), mkdir(pic_dir); end
saveas(fig1, fullfile(pic_dir, 'xhy_thrusters_linear.png'));
fprintf('图像1已保存\n');

%% 绘图2: 角运动 (俯仰角度 + 偏航角速度)
fig2 = figure('Position', [100, 100, 1400, 500]);

subplot(1, 3, 1);
plot(n_test, rad2deg(theta_steady), 'r-o', 'LineWidth', 1.5, 'MarkerSize', 4);
xlabel('垂推转速 (RPM)'); ylabel('稳态俯仰角 θ (deg)');
title(sprintf('垂推反向→俯仰角 (max %.1f°)', rad2deg(max(abs(theta_steady)))));
grid on;

subplot(1, 3, 2);
plot(n_test, rad2deg(r_steady), 'm-o', 'LineWidth', 1.5, 'MarkerSize', 4);
xlabel('侧推转速 (RPM)'); ylabel('稳态偏航角速度 r (deg/s)');
title(sprintf('侧推反向→偏航角速度 (max %.1f °/s)', rad2deg(max(r_steady))));
grid on;

subplot(1, 3, 3);
yyaxis left;
plot(M_pitch_arr, rad2deg(theta_steady), 'r-s', 'LineWidth', 1.5, 'MarkerSize', 4);
ylabel('稳态俯仰角 θ (deg)');
yyaxis right;
plot(N_yaw_arr, rad2deg(r_steady), 'm-s', 'LineWidth', 1.5, 'MarkerSize', 4);
ylabel('稳态偏航角速度 r (deg/s)');
xlabel('力矩 (N·m)');
title('力矩 vs 角运动');
legend('俯仰M→θ', '偏航N→r', 'Location', 'northwest');
grid on;

sgtitle('XHY 俯仰/偏航 稳态响应 (辅推D=6cm, CFD阻力)');

saveas(fig2, fullfile(pic_dir, 'xhy_thrusters_angular.png'));
fprintf('图像2已保存\n');

%% 打印结果
fprintf('\n========== 结果汇总 ==========\n');
fprintf('推进器: 辅推D=6cm KT=0.22 | 主推D=10cm KT_f=0.33 KT_r=0.22\n');
fprintf('阻力: CFD二次拟合 (surge:3.77v²+0.76v, sway:142.85v², heave:45.04v²+1.45v)\n\n');

fprintf('--- 线速度 ---\n');
fprintf('%-10s %-12s %-12s %-12s %-12s\n', 'RPM', 'w(m/s)', 'v(m/s)', 'Z(N)', 'Y(N)');
fprintf('%s\n', repmat('-', 1, 60));
for k = 1:3:n_steps
    fprintf('%-10.0f %-12.4f %-12.4f %-12.2f %-12.2f\n', ...
        n_test(k), w_steady(k), v_steady(k), T_vert_arr(k), T_side_arr(k));
end

fprintf('\n--- 角运动 ---\n');
fprintf('%-10s %-16s %-16s %-12s %-12s\n', ...
    'RPM', 'theta(deg)', 'r(deg/s)', 'M(N·m)', 'N(N·m)');
fprintf('%s\n', repmat('-', 1, 66));
for k = 1:3:n_steps
    fprintf('%-10.0f %-16.2f %-16.2f %-12.2f %-12.2f\n', ...
        n_test(k), rad2deg(theta_steady(k)), rad2deg(r_steady(k)), ...
        M_pitch_arr(k), N_yaw_arr(k));
end

fprintf('\n--- 关键指标 ---\n');
fprintf('最大沉浮速度   w_max = %.4f m/s  @ RPM=%.0f\n', max(w_steady), n_max);
fprintf('最大横荡速度   v_max = %.4f m/s  @ RPM=%.0f\n', max(v_steady), n_max);
fprintf('最大俯仰角   θ_max = %.2f°     @ RPM=%.0f\n', rad2deg(max(abs(theta_steady))), n_max);
fprintf('最大偏航角速度 r_max = %.2f°/s  @ RPM=%.0f\n', rad2deg(max(r_steady)), n_max);

% 推进器推力效率
fprintf('\n--- 推力效率 (速度/推力) ---\n');
fprintf('Surge:  %.2f (m/s)/N  @ 2500RPM\n', 3.8491 / 58.78);
fprintf('Sway:   %.4f (m/s)/N  @ 2500RPM (阻力很大)\n', max(v_steady) / max(T_side_arr));
fprintf('Heave:  %.4f (m/s)/N  @ 2500RPM\n', max(w_steady) / max(T_vert_arr));
fprintf('Yaw:    %.2f (°/s)/N·m @ 2500RPM\n', rad2deg(max(r_steady)) / max(N_yaw_arr));
fprintf('Pitch:  %.2f °/N·m     @ 2500RPM (稳态角度,非角速度)\n', ...
    rad2deg(max(abs(theta_steady))) / max(abs(M_pitch_arr)));

fprintf('\n测试完成!\n');
