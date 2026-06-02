% 时域系统辨识 v3: 同时辨识推力增益k和阻力系数d1,d2
% 动力学: m*dv/dt = k*cmd - d1*v - d2*v*|v|
% 整理: m*dv/dt = [cmd, -v, -v*|v|] * [k; d1; d2]
% → 线性回归, k/d1/d2 同时辨识
clear; clc;

addpath('.', './Lib', './guidance', './controller/xhy', './controller/remus', './model', './eso', './post', './traj');

%% 加载
T_data = readtable('data/debug_data260530-1.csv');
t_raw = T_data.pc_timestamp - T_data.pc_timestamp(1);
N_raw = length(t_raw);
dt_med = median(diff(t_raw));

force_cmd = [T_data.force_cmd1, T_data.force_cmd2, T_data.force_cmd3, ...
             T_data.force_cmd4, T_data.force_cmd5, T_data.force_cmd6];

u = T_data.linear_vel_x; v = T_data.linear_vel_y;
w = T_data.linear_vel_z; r = T_data.angular_vel_z * pi/180;

% 降采样+滤波: 原始~1kHz采样率远高于AUV动力学带宽(~0.1Hz)
% 降采样到~10Hz, 然后用多项式滤波去噪后再计算加速度
fs_target = 10;  % 目标采样率 Hz
ds_factor = max(1, round(1/(fs_target * dt_med)));
fprintf('降采样因子: %d (%.0fHz → %.0fHz)\n', ds_factor, 1/dt_med, 1/(dt_med*ds_factor));

% 降采样 (取中位数抗噪)
n_ds = floor(N_raw / ds_factor);
u_ds = zeros(n_ds, 1); v_ds = zeros(n_ds, 1);
r_ds = zeros(n_ds, 1);
cmd_X_ds = zeros(n_ds, 1); cmd_Y_ds = zeros(n_ds, 1);
cmd_N_ds = zeros(n_ds, 1); t_ds = zeros(n_ds, 1);
cmd_X_raw = force_cmd(:,1); cmd_Y_raw = force_cmd(:,2);
cmd_N_raw = force_cmd(:,6);

for i = 1:n_ds
    idx = (i-1)*ds_factor + 1 : min(i*ds_factor, N_raw);
    u_ds(i) = median(u(idx));
    v_ds(i) = median(v(idx));
    r_ds(i) = median(r(idx));
    cmd_X_ds(i) = median(cmd_X_raw(idx));
    cmd_Y_ds(i) = median(cmd_Y_raw(idx));
    cmd_N_ds(i) = median(cmd_N_raw(idx));
    t_ds(i) = median(t_raw(idx));
end
dt_ds = median(diff(t_ds));
fprintf('降采样后: %d点, dt≈%.3fs\n', n_ds, dt_ds);

% Savitzky-Golay滤波 (窗口约2s, 2阶多项式)
win = max(3, round(2.0 / dt_ds));
if mod(win,2)==0, win = win+1; end
u_filt = sgolayfilt(u_ds, 2, win);
v_filt = sgolayfilt(v_ds, 2, win);
r_filt = sgolayfilt(r_ds, 2, win);

% 滤波后中心差分求加速度
du = centered_diff(u_filt, dt_ds);
dv_dt = centered_diff(v_filt, dt_ds);
dr_dt = centered_diff(r_filt, dt_ds);

% 更新变量名以使用降采样数据
u = u_ds; v = v_ds; r = r_ds;
cmd_X = cmd_X_ds; cmd_Y = cmd_Y_ds; cmd_N = cmd_N_ds;
t_raw_ds = t_ds;
N_raw = n_ds;
fprintf('滤波后加速度范围: du[%.3f %.3f] dv[%.3f %.3f] dr[%.3f %.3f]\n', ...
    min(du), max(du), min(dv_dt), max(dv_dt), min(dr_dt), max(dr_dt));

% 有效质量
m11 = 33 + 11.2773;   % surge
m22 = 33 + 132.1086;  % sway
Izz = 1.849137 + 0.138; % yaw

%% ===== Surge: m*du/dt = k*cmd_X - d1*u - d2*u*|u| =====
fprintf('========== Surge ==========\n');
fprintf('模型: m*du/dt = k*cmd - d1*u - d2*u*|u|\n\n');

cmd_X = force_cmd(:,1);  % CAN-g
lhs = m11 * du;          % m*加速度 → 合力 (N)
% rhs = k*cmd_X - d1*u - d2*u*|u|
% 线性回归: lhs = [cmd_X, -u, -u*|u|] * [k; d1; d2]

% 筛选: 排除死区和纯噪声点
mask_x = abs(cmd_X) > 200 | abs(du) > 0.02;
fprintf('数据点: %d (%.1f%%)\n', sum(mask_x), sum(mask_x)/N_raw*100);
fprintf('cmd范围: [%.0f %.0f] CAN-g\n', min(cmd_X(mask_x)), max(cmd_X(mask_x)));
fprintf('u范围: [%.3f %.3f] m/s\n', min(u(mask_x)), max(u(mask_x)));
fprintf('m*du范围: [%.1f %.1f] N\n', min(lhs(mask_x)), max(lhs(mask_x)));

% 回归矩阵
X_reg = [cmd_X(mask_x), -u(mask_x), -u(mask_x).*abs(u(mask_x))];
y_reg = lhs(mask_x);

% 稳健回归 (Huber)
[coeff_x, stats_x] = robustfit(X_reg(:,2:3), y_reg - X_reg(:,1)*0);
% robustfit: y = b0 + b1*x1 + b2*x2 (不含cmd列, cmd用已知初值)
% 但我们需要同时拟合k。改用fminsearch做三维优化

% 直接用IRLS做三维回归
w = ones(sum(mask_x), 1);
coeff = X_reg \ y_reg;  % OLS初始
for iter = 1:10
    resid = y_reg - X_reg * coeff;
    mad_val = median(abs(resid));
    if mad_val < 1e-6, break; end
    w = 1 ./ (1 + (resid / (mad_val * 1.4826)).^2);
    coeff = (X_reg' * diag(w) * X_reg) \ (X_reg' * diag(w) * y_reg);
end

k_x = coeff(1);
d1_x = coeff(2);
d2_x = coeff(3);

y_pred = X_reg * coeff;
SSE = sum((y_reg - y_pred).^2);
SST = sum((y_reg - mean(y_reg)).^2);
R2_x = 1 - SSE / SST;

fprintf('\n辨识结果:\n');
fprintf('  k=%.6f N/CAN-g  (标准g2N=0.00981, 比值=%.3f)\n', k_x, k_x/0.00981);
fprintf('  d1=%.4f, d2=%.4f\n', d1_x, d2_x);
fprintf('  R²=%.4f\n', R2_x);

% 阻力曲线
fprintf('\n阻力 vs 速度 (使用辨识参数):\n');
u_bins = [-0.5, -0.2, -0.05, 0, 0.05, 0.2, 0.5, 0.8, 1.1];
for i = 1:length(u_bins)-1
    mask = u(mask_x) > u_bins(i) & u(mask_x) <= u_bins(i+1);
    if sum(mask) > 10
        drag_est = k_x*cmd_X(mask_x) - m11*du(mask_x);
        drag_model = d1_x*u(mask_x) + d2_x*u(mask_x).*abs(u(mask_x));
        fprintf('  u∈[%.1f,%.1f]: drag估算=%.2f±%.1f, drag模型=%.2f N\n', ...
            u_bins(i), u_bins(i+1), mean(drag_est(mask)), std(drag_est(mask)), mean(drag_model(mask)));
    end
end

%% ===== Sway: m*dv/dt = k*cmd_Y - d1*v - d2*v*|v| =====
fprintf('\n========== Sway ==========\n');

cmd_Y = force_cmd(:,2);
lhs_y = m22 * dv_dt;

mask_y = abs(cmd_Y) > 100 | abs(dv_dt) > 0.01;
fprintf('数据点: %d\n', sum(mask_y));

if sum(mask_y) > 100
    X_reg_y = [cmd_Y(mask_y), -v(mask_y), -v(mask_y).*abs(v(mask_y))];
    y_reg_y = lhs_y(mask_y);

    w = ones(sum(mask_y), 1);
    coeff_y = X_reg_y \ y_reg_y;
    for iter = 1:10
        resid = y_reg_y - X_reg_y * coeff_y;
        mad_val = median(abs(resid));
        if mad_val < 1e-6, break; end
        w = 1 ./ (1 + (resid/(mad_val*1.4826)).^2);
        coeff_y = (X_reg_y' * diag(w) * X_reg_y) \ (X_reg_y' * diag(w) * y_reg_y);
    end

    k_y = coeff_y(1);
    d1_y = coeff_y(2);
    d2_y = coeff_y(3);
    y_pred_y = X_reg_y * coeff_y;
    R2_y = 1 - sum((y_reg_y-y_pred_y).^2)/sum((y_reg_y-mean(y_reg_y)).^2);
    fprintf('k=%.6f, d1=%.4f, d2=%.4f, R²=%.4f\n', k_y, d1_y, d2_y, R2_y);
end

%% ===== Yaw: Izz*dr/dt = k*cmd_N - dr*r =====
fprintf('\n========== Yaw ==========\n');

cmd_N = force_cmd(:,6);
lhs_n = Izz * dr_dt;

mask_n = abs(cmd_N) > 50 | abs(dr_dt) > 0.05;
fprintf('数据点: %d\n', sum(mask_n));

if sum(mask_n) > 100
    X_reg_n = [cmd_N(mask_n), -r(mask_n)];
    y_reg_n = lhs_n(mask_n);

    w = ones(sum(mask_n), 1);
    coeff_n = X_reg_n \ y_reg_n;
    for iter = 1:10
        resid = y_reg_n - X_reg_n * coeff_n;
        mad_val = median(abs(resid));
        if mad_val < 1e-6, break; end
        w = 1 ./ (1 + (resid/(mad_val*1.4826)).^2);
        coeff_n = (X_reg_n' * diag(w) * X_reg_n) \ (X_reg_n' * diag(w) * y_reg_n);
    end

    k_n = coeff_n(1);
    dr_n = coeff_n(2);
    y_pred_n = X_reg_n * coeff_n;
    R2_n = 1 - sum((y_reg_n-y_pred_n).^2)/sum((y_reg_n-mean(y_reg_n)).^2);
    fprintf('k_N=%.6f, dr=%.4f, R²=%.4f\n', k_n, dr_n, R2_n);
    fprintf('原CFD: dr=1.85\n');
end

%% ===== 汇总 =====
fprintf('\n========== 时域辨识汇总 ==========\n');
fprintf('%-8s %-14s %-12s %-12s %-8s\n', '通道', 'k (N/单位)', 'd1', 'd2/dr', 'R²');
fprintf('%s\n', repmat('-', 1, 60));
fprintf('%-8s %-14.6f %-12.4f %-12.4f %-8.4f\n', 'Surge', k_x, d1_x, d2_x, R2_x);
fprintf('  单位: cmd=CAN-g,  k_std=0.00981,  k/k_std=%.3f\n', k_x/0.00981);
fprintf('  CFD: d1=0.76, d2=3.77\n');
if exist('k_y','var')
    fprintf('%-8s %-14.6f %-12.4f %-12.4f %-8.4f\n', 'Sway', k_y, d1_y, d2_y, R2_y);
end
if exist('k_n','var')
    fprintf('%-8s %-14.6f %-12s %-12.4f %-8.4f\n', 'Yaw', k_n, '—', dr_n, R2_n);
    fprintf('  单位: cmd=CAN-MZ,  CFD: dr=1.85\n');
end

%% ===== 使用辨识参数验证 =====
fprintf('\n--- 正向仿真验证 ---\n');

% 取一段数据, 用辨识参数仿真, 对比实测
seg_len = min(5000, N_raw);
t_seg = t_raw(1:seg_len);
cmd_seg = cmd_X(1:seg_len);
u_true = u(1:seg_len);

% 正向仿真
u_sim_val = zeros(seg_len, 1);
for i = 1:seg_len-1
    dt_i = t_seg(i+1) - t_seg(i);
    if dt_i <= 0 || dt_i > 1, dt_i = dt_med; end
    F = k_x * cmd_seg(i) - d1_x*u_sim_val(i) - d2_x*u_sim_val(i)*abs(u_sim_val(i));
    u_sim_val(i+1) = u_sim_val(i) + F/m11 * dt_i;
end

% 只用部分段显示
plot_start = 1; plot_end = min(2000, seg_len);
SSE_val = sum((u_true(plot_start:plot_end) - u_sim_val(plot_start:plot_end)).^2);
SST_val = sum((u_true(plot_start:plot_end) - mean(u_true(plot_start:plot_end))).^2);
R2_val = 1 - SSE_val/SST_val;

fprintf('正向仿真 R² = %.4f (前%d点)\n', R2_val, plot_end);

%% ===== 更新模型 =====
fprintf('\n--- 更新 xhy_drag_cfd.m ---\n');

% 反推: 在仿真中, 推力由thrust_main/aux产生 (单位N)
% 辨识得到的k是 CAN-g→N的转换, 但仿真用thrust_main直接产生N
% 需要将辨识的d1,d2直接写入 (假设thrust_main的N≈实际推力)

% 但仿真thrust_main(2500)=58.78N, 而CAN 8000g对应的实际推力=k_x*8000
% 这两者可能不同。我们需要调整KT使仿真推力≈实际物理推力
% 或者调整drag使仿真稳态速度≈实际稳态速度

% 这里先写入辨识的d1,d2 (它们表征的是drag(v)的物理关系,
% 独立于推力模型), 然后建议同时调整KT

fid = fopen('model/xhy_drag_cfd.m', 'w');
fprintf(fid, 'function [tau_drag, D] = xhy_drag_cfd(nu_r, M)\n');
fprintf(fid, '%% XHY_DRAG_CFD 阻力计算（时域系统辨识 v3: %s）\n', datestr(now));
fprintf(fid, '%%   同时辨识 k*cmd - d1*v - d2*v*|v| = m*dv/dt\n');
fprintf(fid, '%%   Surge: d1=%.4f d2=%.4f  R²=%.4f\n', d1_x, d2_x, R2_x);
if exist('d1_y','var')
    fprintf(fid, '%%   Sway:  d1=%.4f d2=%.4f  R²=%.4f\n', d1_y, d2_y, R2_y);
else
    d1_y = 0; d2_y = 142.85;
end
if exist('dr_n','var')
    fprintf(fid, '%%   Yaw:   dr=%.4f          R²=%.4f\n', dr_n, R2_n);
else
    dr_n = 1.85;
end
fprintf(fid, '\n');
fprintf(fid, 'u = nu_r(1); v = nu_r(2); w = nu_r(3);\n');
fprintf(fid, 'p = nu_r(4); q = nu_r(5); r = nu_r(6);\n\n');
fprintf(fid, '%% 平动阻力\n');
fprintf(fid, 'Fx = -(%.6f * u + %.6f * u * abs(u));\n', d1_x, d2_x);
fprintf(fid, 'Fy = -(%.6f * v + %.6f * v * abs(v));\n', d1_y, d2_y);
fprintf(fid, 'Fz = -(1.4500 * w + 45.0400 * w * abs(w));  %% 待辨识\n\n');
fprintf(fid, '%% 转动阻尼\n');
fprintf(fid, 'zeta4 = 0.3; zeta5 = 0.8;\n');
fprintf(fid, 'W = 33 * 9.81; r_bg_z = 0; r_bb_z = -0.03;\n');
fprintf(fid, 'w4 = sqrt(W*(r_bg_z-r_bb_z)/M(4,4));\n');
fprintf(fid, 'w5 = sqrt(W*(r_bg_z-r_bb_z)/M(5,5));\n');
fprintf(fid, 'T6 = 1;\n');
fprintf(fid, 'K  = -M(4,4)*2*zeta4*w4*p;\n');
fprintf(fid, 'My = -M(5,5)*2*zeta5*w5*q;\n');
fprintf(fid, 'Nz = -M(6,6)/T6*r*%.4f;\n', dr_n/1.849);
fprintf(fid, '\ntau_drag = [Fx; Fy; Fz; K; My; Nz];\n\n');
fprintf(fid, 'D = diag([%.6f+%.6f*abs(u); %.6f+%.6f*abs(v);\n', d1_x, d2_x, d1_y, d2_y);
fprintf(fid, '         1.45+45.04*abs(w); M(4,4)*2*zeta4*w4;\n');
fprintf(fid, '         M(5,5)*2*zeta5*w5; M(6,6)/T6*%.4f]);\n', dr_n/1.849);
fprintf(fid, 'end\n');
fclose(fid);
fprintf('已写入 model/xhy_drag_cfd.m\n');

%% ===== 绘图 =====
figure('Position', [50, 50, 1400, 900]);

% Surge 阻力
subplot(2,3,1);
drag_from_data = k_x*cmd_X(mask_x) - m11*du(mask_x);
scatter(u(mask_x), drag_from_data, 2, 'filled', 'MarkerFaceAlpha', 0.15); hold on;
[u_sort, idx] = sort(u(mask_x));
plot(u_sort, d1_x*u_sort + d2_x*u_sort.*abs(u_sort), 'r-', 'LineWidth', 2);
plot(u_sort, 0.76*u_sort + 3.77*u_sort.*abs(u_sort), 'b--', 'LineWidth', 1.5);
xlabel('u (m/s)'); ylabel('阻力 (N)');
title(sprintf('Surge: drag=d1*u+d2*u|u|  R²=%.3f', R2_x));
legend('数据', '辨识', '原CFD', 'Location', 'best'); grid on;

% m*du/dt 拟合
subplot(2,3,2);
scatter(y_reg, y_pred, 2, 'filled', 'MarkerFaceAlpha', 0.15); hold on;
plot([min(y_reg) max(y_reg)], [min(y_reg) max(y_reg)], 'r-');
xlabel('实测 m*du/dt'); ylabel('预测 m*du/dt');
title(sprintf('Surge合力拟合 R²=%.3f', R2_x)); grid on;

% 正向仿真验证
subplot(2,3,3);
plot(t_seg(plot_start:plot_end)/60, u_true(plot_start:plot_end), 'b.', 'MarkerSize', 1); hold on;
plot(t_seg(plot_start:plot_end)/60, u_sim_val(plot_start:plot_end), 'r-', 'LineWidth', 1);
xlabel('时间 (min)'); ylabel('u (m/s)');
title(sprintf('正向验证 R²=%.3f', R2_val));
legend('实测', '仿真', 'Location', 'best'); grid on;

% Sway
subplot(2,3,4);
if exist('k_y','var')
    drag_y_data = k_y*cmd_Y(mask_y) - m22*dv_dt(mask_y);
    scatter(v(mask_y), drag_y_data, 2, 'filled', 'MarkerFaceAlpha', 0.15); hold on;
    [v_sort, idx] = sort(v(mask_y));
    plot(v_sort, d1_y*v_sort + d2_y*v_sort.*abs(v_sort), 'r-', 'LineWidth', 2);
    xlabel('v (m/s)'); ylabel('阻力 (N)');
    title(sprintf('Sway  R²=%.3f', R2_y)); grid on;
end

% Yaw
subplot(2,3,5);
if exist('k_n','var')
    scatter(y_reg_n, y_pred_n, 2, 'filled', 'MarkerFaceAlpha', 0.15); hold on;
    plot([min(y_reg_n) max(y_reg_n)], [min(y_reg_n) max(y_reg_n)], 'r-');
    xlabel('实测 Izz*dr/dt'); ylabel('预测');
    title(sprintf('Yaw  R²=%.3f, dr=%.2f', R2_n, dr_n)); grid on;
end

% 阻力曲线对比
subplot(2,3,6);
u_plt = linspace(0, 1.2, 100);
plot(u_plt, 0.76*u_plt+3.77*u_plt.^2, 'b--', 'LineWidth', 1.5); hold on;
plot(u_plt, d1_x*u_plt + d2_x*u_plt.^2, 'r-', 'LineWidth', 2);
u_plt_neg = linspace(-0.6, 0, 100);
plot(u_plt_neg, d1_x*u_plt_neg + d2_x*u_plt_neg.*abs(u_plt_neg), 'r-', 'LineWidth', 2);
xlabel('u (m/s)'); ylabel('阻力 (N)');
title(sprintf('阻力曲线: CFD vs 辨识 (k=%.4f N/CAN-g)', k_x));
legend('原CFD', '辨识', 'Location', 'northwest'); grid on;

sgtitle('时域系统辨识 v3: 联合辨识 k/d1/d2');
saveas(gcf, 'pic/sysid_results_v3.png');
fprintf('图表已保存至 pic/sysid_results_v3.png\n');

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
