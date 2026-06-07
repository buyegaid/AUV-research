% 测试XHY实物辅推 M060 在 24V 下的 PWM 稳态速度、角速度和力矩
% 输入为 PWM 脉宽和电压, 通过 m060_thruster_model 得到推力后驱动 XHY 动力学。
% 测试4种工况:
% 1. 垂推同向 → 最大沉浮速度w (注: 正常双垂推)
% 2. 垂推反向 → 稳态俯仰角theta (浮力恢复力矩平衡, q=0)
% 3. 侧推同向 → 最大横荡速度v
% 4. 侧推反向 → 最大偏航角速度r (无恢复力矩)

script_dir = fileparts(mfilename('fullpath'));
project_root = fullfile(script_dir, '..', '..');
project_root = setup_paths();

%% 参数设置
n_steps = 26;
voltage_v = 24;
params_m060 = m060_thruster_params();

pwm_test = linspace(params_m060.pwm_mid_us, params_m060.pwm_max_us, n_steps);
pwm_reverse = params_m060.pwm_mid_us - (pwm_test - params_m060.pwm_mid_us);

T_sim = 200;  % 仿真时长 (s)
h = 0.05;     % 时间步长 (s)

Vc = 0; betaVc = 0; w_c = 0;

%% 工况1：垂推同向 → 最大沉浮速度w
fprintf('=== 工况1: 垂推同向 (最大沉浮速度) ===\n');
w_steady = zeros(size(pwm_test));
T_vert_arr = zeros(size(pwm_test));

for k = 1:length(pwm_test)
    pwm = [1500; pwm_test(k); pwm_test(k); 1500; 1500];

    x = zeros(12, 1);
    for t = 0:h:T_sim
        [xdot, ~, thr] = xhy_pwm_test_step(x, pwm, voltage_v, Vc, betaVc, w_c);
        x = x + h * xdot;
    end
    w_steady(k) = x(3);
    T_vert_arr(k) = thr.vert1.thrust_n + thr.vert2.thrust_n;
end

%% 工况2：垂推反向 → 稳态俯仰角theta (浮力恢复, q→0)
fprintf('=== 工况2: 垂推反向 (稳态俯仰角) ===\n');
theta_steady = zeros(size(pwm_test));
M_pitch_arr = zeros(size(pwm_test));

x_vert_f = 0.344;   % 前垂推x位置 (m)
x_vert_r = -0.293;  % 后垂推x位置 (m)

for k = 1:length(pwm_test)
    pwm = [1500; pwm_test(k); pwm_reverse(k); 1500; 1500];

    x = zeros(12, 1);
    for t = 0:h:T_sim
        [xdot, tau_thr] = xhy_pwm_test_step(x, pwm, voltage_v, Vc, betaVc, w_c);
        x = x + h * xdot;
    end
    theta_steady(k) = x(11);  % 稳态俯仰角 (rad)
    M_pitch_arr(k) = tau_thr(5);
end

%% 工况3：侧推同向 → 最大横荡速度v
fprintf('=== 工况3: 侧推同向 (最大横荡速度) ===\n');
v_steady = zeros(size(pwm_test));
T_side_arr = zeros(size(pwm_test));

for k = 1:length(pwm_test)
    pwm = [1500; 1500; 1500; pwm_test(k); pwm_test(k)];

    x = zeros(12, 1);
    for t = 0:h:T_sim
        [xdot, ~, thr] = xhy_pwm_test_step(x, pwm, voltage_v, Vc, betaVc, w_c);
        x = x + h * xdot;
    end
    v_steady(k) = x(2);
    T_side_arr(k) = thr.side1.thrust_n + thr.side2.thrust_n;
end

%% 工况4：侧推反向 → 最大偏航角速度r (无恢复力矩)
fprintf('=== 工况4: 侧推反向 (最大偏航角速度) ===\n');
r_steady = zeros(size(pwm_test));
N_yaw_arr = zeros(size(pwm_test));

x_side_f = 0.424;   % 前侧推x位置 (m)
x_side_r = -0.376;  % 后侧推x位置 (m)

for k = 1:length(pwm_test)
    pwm = [1500; 1500; 1500; pwm_test(k); pwm_reverse(k)];

    x = zeros(12, 1);
    for t = 0:h:T_sim
        [xdot, tau_thr] = xhy_pwm_test_step(x, pwm, voltage_v, Vc, betaVc, w_c);
        x = x + h * xdot;
    end
    r_steady(k) = x(6);
    N_yaw_arr(k) = tau_thr(6);
end

%% 绘图1: 线速度
fig1 = figure('Position', [100, 100, 1400, 500]);

subplot(1, 3, 1);
plot(pwm_test, w_steady, 'b-o', 'LineWidth', 1.5, 'MarkerSize', 4);
xlabel('M060 垂推 PWM (us)'); ylabel('稳态沉浮速度 w (m/s)');
title(sprintf('垂推同向→沉浮速度 (max %.3f m/s)', max(w_steady)));
grid on;

subplot(1, 3, 2);
plot(pwm_test, v_steady, 'g-o', 'LineWidth', 1.5, 'MarkerSize', 4);
xlabel('M060 侧推 PWM (us)'); ylabel('稳态横荡速度 v (m/s)');
title(sprintf('侧推同向→横荡速度 (max %.3f m/s)', max(v_steady)));
grid on;

subplot(1, 3, 3);
plot(T_vert_arr, w_steady, 'b-s', 'LineWidth', 1.5, 'MarkerSize', 4); hold on;
plot(T_side_arr, v_steady, 'g-s', 'LineWidth', 1.5, 'MarkerSize', 4);
xlabel('合力 (N)'); ylabel('稳态线速度 (m/s)');
title('推力 vs 线速度');
legend('垂推→w', '侧推→v', 'Location', 'northwest');
grid on;

sgtitle(sprintf('XHY 辅推 M060 PWM 稳态线速度 (%.0fV, CFD阻力)', voltage_v));

pic_dir = fullfile(project_root, 'assets', 'figures');
if ~exist(pic_dir, 'dir'), mkdir(pic_dir); end
saveas(fig1, fullfile(pic_dir, 'xhy_thrusters_linear.png'));
fprintf('图像1已保存\n');

%% 绘图2: 角运动 (俯仰角度 + 偏航角速度)
fig2 = figure('Position', [100, 100, 1400, 500]);

subplot(1, 3, 1);
plot(pwm_test, rad2deg(theta_steady), 'r-o', 'LineWidth', 1.5, 'MarkerSize', 4);
xlabel('M060 垂推正向 PWM (us)'); ylabel('稳态俯仰角 θ (deg)');
title(sprintf('垂推反向→俯仰角 (max %.1f°)', rad2deg(max(abs(theta_steady)))));
grid on;

subplot(1, 3, 2);
plot(pwm_test, rad2deg(r_steady), 'm-o', 'LineWidth', 1.5, 'MarkerSize', 4);
xlabel('M060 侧推正向 PWM (us)'); ylabel('稳态偏航角速度 r (deg/s)');
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

sgtitle(sprintf('XHY 辅推 M060 PWM 俯仰/偏航稳态响应 (%.0fV, CFD阻力)', voltage_v));

saveas(fig2, fullfile(pic_dir, 'xhy_thrusters_angular.png'));
fprintf('图像2已保存\n');

%% 打印结果
fprintf('\n========== 结果汇总 ==========\n');
fprintf('推进器: 主推 M080, 辅推 M060, 输入电压 %.1f V\n', voltage_v);
fprintf('测试输入: PWM 脉宽, 推力模型: m080_thruster_model.m / m060_thruster_model.m\n');
fprintf('阻力: CFD二次拟合 (surge:3.77v²+0.76v, sway:142.85v², heave:45.04v²+1.45v)\n\n');

fprintf('--- 线速度 ---\n');
fprintf('%-10s %-12s %-12s %-12s %-12s\n', 'PWM', 'w(m/s)', 'v(m/s)', 'Z(N)', 'Y(N)');
fprintf('%s\n', repmat('-', 1, 60));
for k = 1:3:n_steps
    fprintf('%-10.0f %-12.4f %-12.4f %-12.2f %-12.2f\n', ...
        pwm_test(k), w_steady(k), v_steady(k), T_vert_arr(k), T_side_arr(k));
end

fprintf('\n--- 角运动 ---\n');
fprintf('%-10s %-16s %-16s %-12s %-12s\n', ...
    'PWM', 'theta(deg)', 'r(deg/s)', 'M(N·m)', 'N(N·m)');
fprintf('%s\n', repmat('-', 1, 66));
for k = 1:3:n_steps
    fprintf('%-10.0f %-16.2f %-16.2f %-12.2f %-12.2f\n', ...
        pwm_test(k), rad2deg(theta_steady(k)), rad2deg(r_steady(k)), ...
        M_pitch_arr(k), N_yaw_arr(k));
end

fprintf('\n--- 关键指标 ---\n');
fprintf('最大沉浮速度   w_max = %.4f m/s  @ PWM=%.0f us\n', max(w_steady), params_m060.pwm_max_us);
fprintf('最大横荡速度   v_max = %.4f m/s  @ PWM=%.0f us\n', max(v_steady), params_m060.pwm_max_us);
fprintf('最大俯仰角   θ_max = %.2f°     @ PWM=%.0f us\n', rad2deg(max(abs(theta_steady))), params_m060.pwm_max_us);
fprintf('最大偏航角速度 r_max = %.2f°/s  @ PWM=%.0f us\n', rad2deg(max(r_steady)), params_m060.pwm_max_us);

% 推进器推力效率
fprintf('\n--- 推力效率 (速度/推力) ---\n');
fprintf('Sway:   %.4f (m/s)/N  @ PWM=%.0f us (阻力很大)\n', max(v_steady) / max(T_side_arr), params_m060.pwm_max_us);
fprintf('Heave:  %.4f (m/s)/N  @ PWM=%.0f us\n', max(w_steady) / max(T_vert_arr), params_m060.pwm_max_us);
fprintf('Yaw:    %.2f (°/s)/N·m @ PWM=%.0f us\n', rad2deg(max(r_steady)) / max(N_yaw_arr), params_m060.pwm_max_us);
fprintf('Pitch:  %.2f °/N·m     @ PWM=%.0f us (稳态角度,非角速度)\n', ...
    rad2deg(max(abs(theta_steady))) / max(abs(M_pitch_arr)), params_m060.pwm_max_us);

fprintf('\n测试完成!\n');
