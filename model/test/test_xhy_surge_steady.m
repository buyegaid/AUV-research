% 测试XHY主推进器RPM从最小到最大时的稳态纵荡速度
% 方法：对每个RPM值，积分动力学方程直到速度收敛，记录稳态u

addpath('.', './Lib', './guidance', './controller/xhy', './controller/remus', './model', './eso', './post', './traj');

%% 参数设置
n_min  = 0;       % 最小RPM
n_max  = 2500;    % 最大RPM（与xhy.m中n_max一致）
n_steps = 26;     % 测试点数量

n_test = linspace(n_min, n_max, n_steps);

T_sim  = 200;     % 每个工况仿真时长 (s)
h      = 0.05;    % 时间步长 (s)

% 无海流
Vc = 0; betaVc = 0; w_c = 0;

%% 稳态速度计算
u_steady = zeros(size(n_test));

for k = 1:length(n_test)
    n_main = n_test(k);
    ui = [n_main; 0; 0; 0; 0];  % 仅主推，其余为0

    x = zeros(12, 1);  % 初始状态全零

    for t = 0:h:T_sim
        xdot = xhy(x, ui, Vc, betaVc, w_c);
        x = x + h * xdot;  % 欧拉积分
    end

    u_steady(k) = x(1);  % 稳态纵荡速度 u (m/s)
end

%% 绘图
fig = figure('Position', [100, 100, 900, 500]);

% 子图1：RPM vs 稳态速度
subplot(1, 2, 1);
plot(n_test, u_steady, 'b-o', 'LineWidth', 1.5, 'MarkerSize', 5);
xlabel('主推转速 n_{main} (RPM)');
ylabel('稳态纵荡速度 u (m/s)');
title('主推RPM vs 稳态速度');
grid on;

% 子图2：推力 vs 稳态速度（推力由RPM计算）
rho   = 1026;
D_prop = 0.10;
KT    = 0.22;
n_rps = n_test / 60;
T_main = rho * D_prop^4 * KT * abs(n_rps) .* n_rps;  % 推力 (N)

subplot(1, 2, 2);
plot(T_main, u_steady, 'r-s', 'LineWidth', 1.5, 'MarkerSize', 5);
xlabel('主推推力 T (N)');
ylabel('稳态纵荡速度 u (m/s)');
title('主推推力 vs 稳态速度');
grid on;

sgtitle('XHY AUV 主推稳态速度特性（无海流）');

%% 保存图像
pic_dir = 'pic';
if ~exist(pic_dir, 'dir')
    mkdir(pic_dir);
end
saveas(fig, fullfile(pic_dir, 'xhy_surge_steady_speed.png'));
fprintf('图像已保存至 pic/xhy_surge_steady_speed.png\n');

%% 打印结果表格
fprintf('\n主推RPM vs 稳态速度：\n');
fprintf('%-12s %-12s %-12s\n', 'RPM', '推力(N)', '速度(m/s)');
fprintf('%s\n', repmat('-', 1, 38));
for k = 1:length(n_test)
    fprintf('%-12.1f %-12.3f %-12.4f\n', n_test(k), T_main(k), u_steady(k));
end
