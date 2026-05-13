% 测试XHY垂推和侧推的最大速度、角速度和力矩
% 测试4种工况：
% 1. 垂推同向 → 最大沉浮速度w
% 2. 垂推反向 → 最大俯仰角速度q
% 3. 侧推同向 → 最大横荡速度v
% 4. 侧推反向 → 最大偏航角速度r

addpath('.', './Lib', './guidance', './controller/xhy', './controller/remus', './model', './eso', './post', './traj');

%% 参数设置
n_min  = 0;
n_max  = 2500;
n_steps = 26;
n_test = linspace(n_min, n_max, n_steps);

T_sim = 200;  % 仿真时长 (s)
h = 0.05;     % 时间步长 (s)

% 无海流
Vc = 0; betaVc = 0; w_c = 0;

% 推力计算参数（与xhy.m一致）
rho = 1026;
D_prop = 0.10;
KT = 0.22;

%% 工况1：垂推同向 → 最大沉浮速度w
fprintf('测试工况1：垂推同向（最大沉浮速度）...\n');
w_steady = zeros(size(n_test));
tau_z_vert = zeros(size(n_test));

for k = 1:length(n_test)
    n_vert = n_test(k);
    ui = [0; n_vert; n_vert; 0; 0];  % 两个垂推同向

    x = zeros(12, 1);
    for t = 0:h:T_sim
        [xdot, ~, ~, ~, ~, ~, tau] = xhy(x, ui, Vc, betaVc, w_c);
        x = x + h * xdot;
    end

    w_steady(k) = x(3);  % 沉浮速度 w (m/s)

    % 计算推力
    n_rps = n_vert / 60;
    T_vert = rho * D_prop^4 * KT * abs(n_rps) * n_rps;
    tau_z_vert(k) = 2 * T_vert;  % 两个垂推的总Z力
end

%% 工况2：垂推反向 → 最大俯仰角速度q
fprintf('测试工况2：垂推反向（最大俯仰角速度）...\n');
q_steady = zeros(size(n_test));
tau_m_vert = zeros(size(n_test));

for k = 1:length(n_test)
    n_vert = n_test(k);
    ui = [0; n_vert; -n_vert; 0; 0];  % 垂推反向

    x = zeros(12, 1);
    for t = 0:h:T_sim
        xdot = xhy(x, ui, Vc, betaVc, w_c);
        x = x + h * xdot;
    end

    q_steady(k) = x(5);  % 俯仰角速度 q (rad/s)

    % 计算力矩（垂推位置见xhy.m）
    n_rps = n_vert / 60;
    T_vert = rho * D_prop^4 * KT * abs(n_rps) * n_rps;
    x_vert_f = 0.344;
    x_vert_r = -0.293;
    tau_m_vert(k) = -T_vert * x_vert_f - (-T_vert) * x_vert_r;  % M力矩
end

%% 工况3：侧推同向 → 最大横荡速度v
fprintf('测试工况3：侧推同向（最大横荡速度）...\n');
v_steady = zeros(size(n_test));
tau_y_side = zeros(size(n_test));

for k = 1:length(n_test)
    n_side = n_test(k);
    ui = [0; 0; 0; n_side; n_side];  % 两个侧推同向

    x = zeros(12, 1);
    for t = 0:h:T_sim
        xdot = xhy(x, ui, Vc, betaVc, w_c);
        x = x + h * xdot;
    end

    v_steady(k) = x(2);  % 横荡速度 v (m/s)

    % 计算推力
    n_rps = n_side / 60;
    T_side = rho * D_prop^4 * KT * abs(n_rps) * n_rps;
    tau_y_side(k) = 2 * T_side;  % 两个侧推的总Y力
end

%% 工况4：侧推反向 → 最大偏航角速度r
fprintf('测试工况4：侧推反向（最大偏航角速度）...\n');
r_steady = zeros(size(n_test));
tau_n_side = zeros(size(n_test));

for k = 1:length(n_test)
    n_side = n_test(k);
    ui = [0; 0; 0; n_side; -n_side];  % 侧推反向

    x = zeros(12, 1);
    for t = 0:h:T_sim
        xdot = xhy(x, ui, Vc, betaVc, w_c);
        x = x + h * xdot;
    end

    r_steady(k) = x(6);  % 偏航角速度 r (rad/s)

    % 计算力矩（侧推位置见xhy.m）
    n_rps = n_side / 60;
    T_side = rho * D_prop^4 * KT * abs(n_rps) * n_rps;
    x_side_f = 0.424;
    x_side_r = -0.376;
    tau_n_side(k) = T_side * x_side_f + (-T_side) * x_side_r;  % N力矩
end

%% 绘图
fig = figure('Position', [100, 100, 1200, 800]);

% 子图1：垂推同向 - 沉浮速度
subplot(2, 3, 1);
plot(n_test, w_steady, 'b-o', 'LineWidth', 1.5, 'MarkerSize', 4);
xlabel('垂推转速 (RPM)');
ylabel('稳态沉浮速度 w (m/s)');
title('垂推同向 → 沉浮速度');
grid on;

% 子图2：垂推同向 - Z力 vs 速度
subplot(2, 3, 2);
plot(tau_z_vert, w_steady, 'b-s', 'LineWidth', 1.5, 'MarkerSize', 4);
xlabel('Z方向力 (N)');
ylabel('稳态沉浮速度 w (m/s)');
title('Z力 → 沉浮速度');
grid on;

% 子图3：垂推反向 - 俯仰角速度
subplot(2, 3, 3);
plot(n_test, rad2deg(q_steady), 'r-o', 'LineWidth', 1.5, 'MarkerSize', 4);
xlabel('垂推转速 (RPM)');
ylabel('稳态俯仰角速度 q (deg/s)');
title('垂推反向 → 俯仰角速度');
grid on;

% 子图4：垂推反向 - M力矩 vs 角速度
subplot(2, 3, 4);
plot(tau_m_vert, rad2deg(q_steady), 'r-s', 'LineWidth', 1.5, 'MarkerSize', 4);
xlabel('M力矩 (N·m)');
ylabel('稳态俯仰角速度 q (deg/s)');
title('M力矩 → 俯仰角速度');
grid on;

% 子图5：侧推同向 - 横荡速度
subplot(2, 3, 5);
plot(n_test, v_steady, 'g-o', 'LineWidth', 1.5, 'MarkerSize', 4);
xlabel('侧推转速 (RPM)');
ylabel('稳态横荡速度 v (m/s)');
title('侧推同向 → 横荡速度');
grid on;

% 子图6：侧推同向 - Y力 vs 速度
subplot(2, 3, 6);
plot(tau_y_side, v_steady, 'g-s', 'LineWidth', 1.5, 'MarkerSize', 4);
xlabel('Y方向力 (N)');
ylabel('稳态横荡速度 v (m/s)');
title('Y力 → 横荡速度');
grid on;

sgtitle('XHY AUV 垂推和侧推稳态性能（无海流）');

%% 第二个图：侧推反向（偏航）
fig2 = figure('Position', [150, 150, 800, 400]);

% 子图1：侧推反向 - 偏航角速度
subplot(1, 2, 1);
plot(n_test, rad2deg(r_steady), 'm-o', 'LineWidth', 1.5, 'MarkerSize', 4);
xlabel('侧推转速 (RPM)');
ylabel('稳态偏航角速度 r (deg/s)');
title('侧推反向 → 偏航角速度');
grid on;

% 子图2：侧推反向 - N力矩 vs 角速度
subplot(1, 2, 2);
plot(tau_n_side, rad2deg(r_steady), 'm-s', 'LineWidth', 1.5, 'MarkerSize', 4);
xlabel('N力矩 (N·m)');
ylabel('稳态偏航角速度 r (deg/s)');
title('N力矩 → 偏航角速度');
grid on;

sgtitle('XHY AUV 侧推反向稳态性能（无海流）');

%% 保存图像
pic_dir = 'pic';
if ~exist(pic_dir, 'dir')
    mkdir(pic_dir);
end

figure(fig);
saveas(fig, fullfile(pic_dir, 'xhy_thrusters_performance.png'));
fprintf('图像已保存至 pic/xhy_thrusters_performance.png\n');

figure(fig2);
saveas(fig2, fullfile(pic_dir, 'xhy_yaw_performance.png'));
fprintf('图像已保存至 pic/xhy_yaw_performance.png\n');

%% 打印结果表格
fprintf('\n========== 工况1：垂推同向（沉浮速度） ==========\n');
fprintf('%-12s %-12s %-12s\n', 'RPM', 'Z力(N)', '速度w(m/s)');
fprintf('%s\n', repmat('-', 1, 38));
for k = 1:5:length(n_test)
    fprintf('%-12.1f %-12.3f %-12.4f\n', n_test(k), tau_z_vert(k), w_steady(k));
end

fprintf('\n========== 工况2：垂推反向（俯仰角速度） ==========\n');
fprintf('%-12s %-12s %-12s\n', 'RPM', 'M力矩(N·m)', '角速度q(deg/s)');
fprintf('%s\n', repmat('-', 1, 38));
for k = 1:5:length(n_test)
    fprintf('%-12.1f %-12.3f %-12.4f\n', n_test(k), tau_m_vert(k), rad2deg(q_steady(k)));
end

fprintf('\n========== 工况3：侧推同向（横荡速度） ==========\n');
fprintf('%-12s %-12s %-12s\n', 'RPM', 'Y力(N)', '速度v(m/s)');
fprintf('%s\n', repmat('-', 1, 38));
for k = 1:5:length(n_test)
    fprintf('%-12.1f %-12.3f %-12.4f\n', n_test(k), tau_y_side(k), v_steady(k));
end

fprintf('\n========== 工况4：侧推反向（偏航角速度） ==========\n');
fprintf('%-12s %-12s %-12s\n', 'RPM', 'N力矩(N·m)', '角速度r(deg/s)');
fprintf('%s\n', repmat('-', 1, 38));
for k = 1:5:length(n_test)
    fprintf('%-12.1f %-12.3f %-12.4f\n', n_test(k), tau_n_side(k), rad2deg(r_steady(k)));
end

fprintf('\n测试完成！\n');

