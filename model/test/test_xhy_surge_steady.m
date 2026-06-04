% 测试XHY主推进器 M080 在 24V 下的 PWM 稳态纵荡响应
% 输入为 PWM 脉宽和电压, 通过 m080_thruster_model 得到推力后驱动 XHY 动力学。

script_dir = fileparts(mfilename('fullpath'));
project_root = fullfile(script_dir, '..', '..');
addpath(project_root, fullfile(project_root, 'Lib'), fullfile(project_root, 'guidance'), ...
    fullfile(project_root, 'controller', 'xhy'), fullfile(project_root, 'controller', 'remus'), ...
    fullfile(project_root, 'model'), fullfile(project_root, 'model', 'params'), ...
    fullfile(project_root, 'model', 'test'), fullfile(project_root, 'eso'), ...
    fullfile(project_root, 'post'), fullfile(project_root, 'traj'));

%% 参数设置
n_steps = 26;
voltage_v = 24;
params_m080 = m080_thruster_params();

pwm_fwd = linspace(params_m080.pwm_mid_us, params_m080.pwm_max_us, n_steps);
pwm_rev = linspace(params_m080.pwm_mid_us, params_m080.pwm_min_us, n_steps);

T_sim  = 200;     % 每个工况仿真时长 (s)
h      = 0.05;    % 时间步长 (s)

Vc = 0; betaVc = 0; w_c = 0;

%% 正向RPM测试
fprintf('=== M080 正向 PWM 测试 ===\n');
u_fwd = zeros(size(pwm_fwd));
T_fwd = zeros(size(pwm_fwd));
rpm_fwd = zeros(size(pwm_fwd));

for k = 1:length(pwm_fwd)
    pwm = [pwm_fwd(k); 1500; 1500; 1500; 1500];

    x = zeros(12, 1);
    for t = 0:h:T_sim
        [xdot, ~, thr] = xhy_pwm_test_step(x, pwm, voltage_v, Vc, betaVc, w_c);
        x = x + h * xdot;
    end
    u_fwd(k) = x(1);
    T_fwd(k) = thr.main.thrust_n;
    rpm_fwd(k) = thr.main.rpm;
end

%% 反向RPM测试
fprintf('=== M080 反向 PWM 测试 ===\n');
u_rev = zeros(size(pwm_rev));
T_rev = zeros(size(pwm_rev));
rpm_rev = zeros(size(pwm_rev));

for k = 1:length(pwm_rev)
    pwm = [pwm_rev(k); 1500; 1500; 1500; 1500];

    x = zeros(12, 1);
    for t = 0:h:T_sim
        [xdot, ~, thr] = xhy_pwm_test_step(x, pwm, voltage_v, Vc, betaVc, w_c);
        x = x + h * xdot;
    end
    u_rev(k) = x(1);
    T_rev(k) = thr.main.thrust_n;
    rpm_rev(k) = thr.main.rpm;
end

%% 绘图
fig = figure('Position', [100, 100, 1200, 500]);

% 子图1：PWM vs 稳态速度（正向+反向）
subplot(1, 3, 1);
plot(pwm_fwd, u_fwd, 'b-o', 'LineWidth', 1.5, 'MarkerSize', 4); hold on;
plot(pwm_rev, u_rev, 'r-s', 'LineWidth', 1.5, 'MarkerSize', 4);
xlabel('M080 PWM (us)'); ylabel('稳态纵荡速度 u (m/s)');
title('主推PWM vs 稳态速度');
legend('正向', '反向', 'Location', 'northwest');
grid on;

% 子图2：推力 vs 稳态速度（正向+反向）
subplot(1, 3, 2);
plot(T_fwd, u_fwd, 'b-o', 'LineWidth', 1.5, 'MarkerSize', 4); hold on;
plot(abs(T_rev), u_rev, 'r-s', 'LineWidth', 1.5, 'MarkerSize', 4);
xlabel('主推推力 |T| (N)'); ylabel('稳态纵荡速度 u (m/s)');
title('主推推力 vs 稳态速度');
legend('正向', '反向', 'Location', 'northwest');
grid on;

% 子图3：PWM 与估计转速对比
subplot(1, 3, 3);
plot(pwm_fwd, u_fwd, 'b-o', 'LineWidth', 1.5, 'MarkerSize', 4, 'DisplayName', '正向速度'); hold on;
plot(pwm_rev, u_rev, 'r-s', 'LineWidth', 1.5, 'MarkerSize', 4, 'DisplayName', '反向速度');
yyaxis right;
plot(pwm_fwd, rpm_fwd, 'b--^', 'LineWidth', 1, 'MarkerSize', 3, 'DisplayName', '正向估计RPM');
plot(pwm_rev, rpm_rev, 'r--v', 'LineWidth', 1, 'MarkerSize', 3, 'DisplayName', '反向估计RPM');
xlabel('M080 PWM (us)');
legend('Location', 'northwest'); grid on;

sgtitle(sprintf(['XHY 主推 M080 PWM 稳态速度特性 (%.0fV, CFD阻力)\n' ...
    '最大正向速度: %.2f m/s, 最大反向速度: %.2f m/s'], ...
    voltage_v, max(u_fwd), min(u_rev)));

% 保存
pic_dir = fullfile(project_root, 'pic');
if ~exist(pic_dir, 'dir'), mkdir(pic_dir); end
saveas(fig, fullfile(pic_dir, 'xhy_surge_steady_speed.png'));
fprintf('图像已保存至 pic/xhy_surge_steady_speed.png\n');

%% 打印结果表格
fprintf('\nM080 PWM vs 稳态速度：\n');
fprintf('输入电压: %.1f V, 推进器模型: m080_thruster_model.m\n', voltage_v);
fprintf('%-10s %-12s %-12s %-10s %-12s %-12s\n', ...
    'PWM_f', 'T_fwd(N)', 'u_fwd(m/s)', 'PWM_r', 'T_rev(N)', 'u_rev(m/s)');
fprintf('%s\n', repmat('-', 1, 72));
for k = 1:3:length(pwm_fwd)
    fprintf('%-10.0f %-12.3f %-12.4f %-10.0f %-12.3f %-12.4f\n', ...
        pwm_fwd(k), T_fwd(k), u_fwd(k), pwm_rev(k), T_rev(k), u_rev(k));
end
fprintf('\n');
fprintf('最大正向速度 @ PWM=%.0f us: u=%.4f m/s (T=%.2f N)\n', params_m080.pwm_max_us, max(u_fwd), max(T_fwd));
fprintf('最大反向速度 @ PWM=%.0f us: u=%.4f m/s (T=%.2f N)\n', params_m080.pwm_min_us, min(u_rev), min(T_rev));
