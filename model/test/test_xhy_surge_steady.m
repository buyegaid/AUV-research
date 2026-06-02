% 测试XHY主推进器 RPM 从最小到最大时的稳态纵荡速度
% 更新: 使用新推力模型(KT_fwd=0.33, KT_rev=0.22) + CFD阻力模型

addpath('.', './Lib', './guidance', './controller/xhy', './controller/remus', './model', './eso', './post', './traj');

%% 参数设置
rho = 1026;
n_max  = 2500;
n_steps = 26;

n_fwd = linspace(0, n_max, n_steps);      % 正向 RPM
n_rev = linspace(0, -n_max, n_steps);      % 反向 RPM

T_sim  = 200;     % 每个工况仿真时长 (s)
h      = 0.05;    % 时间步长 (s)

Vc = 0; betaVc = 0; w_c = 0;

%% 正向RPM测试
fprintf('=== 正向 RPM 测试 ===\n');
u_fwd = zeros(size(n_fwd));
T_fwd = zeros(size(n_fwd));

for k = 1:length(n_fwd)
    n_main = n_fwd(k);
    ui = [n_main; 0; 0; 0; 0];

    x = zeros(12, 1);
    for t = 0:h:T_sim
        xdot = xhy(x, ui, Vc, betaVc, w_c);
        x = x + h * xdot;
    end
    u_fwd(k) = x(1);
    T_fwd(k) = thrust_main(n_main, rho);
end

%% 反向RPM测试
fprintf('=== 反向 RPM 测试 ===\n');
u_rev = zeros(size(n_rev));
T_rev = zeros(size(n_rev));

for k = 1:length(n_rev)
    n_main = n_rev(k);
    ui = [n_main; 0; 0; 0; 0];

    x = zeros(12, 1);
    for t = 0:h:T_sim
        xdot = xhy(x, ui, Vc, betaVc, w_c);
        x = x + h * xdot;
    end
    u_rev(k) = x(1);
    T_rev(k) = thrust_main(n_main, rho);
end

%% 绘图
fig = figure('Position', [100, 100, 1200, 500]);

% 子图1：RPM vs 稳态速度（正向+反向）
subplot(1, 3, 1);
plot(n_fwd, u_fwd, 'b-o', 'LineWidth', 1.5, 'MarkerSize', 4); hold on;
plot(abs(n_rev), u_rev, 'r-s', 'LineWidth', 1.5, 'MarkerSize', 4);
xlabel('主推转速 |n| (RPM)'); ylabel('稳态纵荡速度 u (m/s)');
title('主推RPM vs 稳态速度');
legend('正向 (n>0)', '反向 (n<0)', 'Location', 'northwest');
grid on;

% 子图2：推力 vs 稳态速度（正向+反向）
subplot(1, 3, 2);
plot(T_fwd, u_fwd, 'b-o', 'LineWidth', 1.5, 'MarkerSize', 4); hold on;
plot(abs(T_rev), u_rev, 'r-s', 'LineWidth', 1.5, 'MarkerSize', 4);
xlabel('主推推力 |T| (N)'); ylabel('稳态纵荡速度 u (m/s)');
title('主推推力 vs 稳态速度');
legend('正向', '反向', 'Location', 'northwest');
grid on;

% 子图3：正反向推力和速度对比（RPM绝对值对比）
subplot(1, 3, 3);
plot(n_fwd, u_fwd, 'b-o', 'LineWidth', 1.5, 'MarkerSize', 4, 'DisplayName', '正向速度'); hold on;
plot(abs(n_rev), abs(u_rev), 'r-s', 'LineWidth', 1.5, 'MarkerSize', 4, 'DisplayName', '反向速度|u|');
yyaxis right;
plot(n_fwd, T_fwd, 'b--^', 'LineWidth', 1, 'MarkerSize', 3, 'DisplayName', '正向推力');
plot(abs(n_rev), abs(T_rev), 'r--v', 'LineWidth', 1, 'MarkerSize', 3, 'DisplayName', '反向推力|T|');
xlabel('主推转速 |n| (RPM)');
legend('Location', 'northwest'); grid on;

sgtitle(sprintf(['XHY 主推稳态速度特性 (KT_f=%.2f, KT_r=%.2f, CFD阻力)\n' ...
    '最大正向速度: %.2f m/s, 最大反向速度: %.2f m/s'], ...
    0.33, 0.22, max(u_fwd), min(u_rev)));

% 保存
pic_dir = 'pic';
if ~exist(pic_dir, 'dir'), mkdir(pic_dir); end
saveas(fig, fullfile(pic_dir, 'xhy_surge_steady_speed.png'));
fprintf('图像已保存至 pic/xhy_surge_steady_speed.png\n');

%% 打印结果表格
fprintf('\n主推RPM vs 稳态速度：\n');
fprintf('%-10s %-12s %-12s %-10s %-12s %-12s\n', ...
    'RPM_f', 'T_fwd(N)', 'u_fwd(m/s)', 'RPM_r', 'T_rev(N)', 'u_rev(m/s)');
fprintf('%s\n', repmat('-', 1, 72));
for k = 1:3:length(n_fwd)
    fprintf('%-10.0f %-12.3f %-12.4f %-10.0f %-12.3f %-12.4f\n', ...
        n_fwd(k), T_fwd(k), u_fwd(k), abs(n_rev(k)), abs(T_rev(k)), u_rev(k));
end
fprintf('\n');
fprintf('最大正向速度 @ RPM=%.0f: u=%.4f m/s (T=%.2f N)\n', n_max, max(u_fwd), max(T_fwd));
fprintf('最大反向速度 @ RPM=%.0f: u=%.4f m/s (T=%.2f N)\n', -n_max, min(u_rev), min(T_rev));
