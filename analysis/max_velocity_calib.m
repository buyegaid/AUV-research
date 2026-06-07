% max_velocity_calib.m — 补充: 最大速度匹配法校准阻力系数
% 作为主分析的补充, 用于获得物理上有意义的阻力参数
clear; clc;
project_root = setup_paths();

load('results/pool_test_20260601_results.mat', 'results');

T_data = readtable('data/raw/debug_data260601-1.csv');
t_raw = T_data.pc_timestamp - T_data.pc_timestamp(1);
N_raw = length(t_raw);
dt_raw = median(diff(t_raw));

% CAN矩阵
K_thr = [0,0,-0.5,0,0.0083,0; 0,0,-0.5,0,-0.0083,0; 0,-0.5,0,0,0,-0.0083; 0,-0.5,0,0,0,0.0083; 1,0,0,0,0,0];
force_cmd = [T_data.force_cmd1, T_data.force_cmd2, T_data.force_cmd3, T_data.force_cmd4, T_data.force_cmd5, T_data.force_cmd6];

% 计算推进器推力
thr = zeros(N_raw, 5);
for i = 1:N_raw
    thr(i,:) = (K_thr * force_cmd(i,:)')';
end

u = T_data.linear_vel_x; v = T_data.linear_vel_y;
w = T_data.linear_vel_z; r_raw_deg = T_data.angular_vel_z;

% 降采样
ds = max(1, round(1/(5*dt_raw)));
n_ds = floor(N_raw/ds);
u_ds = zeros(n_ds,1); v_ds = zeros(n_ds,1); w_ds = zeros(n_ds,1); r_ds = zeros(n_ds,1);
thr_ds = zeros(n_ds,5); t_ds = zeros(n_ds,1);
for i = 1:n_ds
    idx = (i-1)*ds+1:min(i*ds,N_raw);
    u_ds(i)=median(u(idx)); v_ds(i)=median(v(idx)); w_ds(i)=median(w(idx));
    r_ds(i)=median(r_raw_deg(idx)); thr_ds(i,:)=median(thr(idx,:),1);
    t_ds(i)=median(t_raw(idx));
end
dt_ds = median(diff(t_ds));
fprintf('降采样: %d点, dt=%.3fs\n', n_ds, dt_ds);

% 计算加速度 (用于稳态检测)
du_dt = centered_diff(u_ds, dt_ds);
dv_dt = centered_diff(v_ds, dt_ds);
dw_dt = centered_diff(w_ds, dt_ds);
dr_dt = centered_diff(r_ds, dt_ds);

% 使用上次校准的k值作为参考
k_x_prev = 0.00080;  % N/CAN-g
k_y_prev = 0.0038;   % 上次Sway估算 (从max velocity反推)
k_z_prev = 0.00062;  % 上次Heave估算
k_n_prev = 0.00014;  % 上次Yaw估算

%% Surge: 最大速度匹配
fprintf('\n=== Surge 最大速度匹配 ===\n');
mask_steady = abs(du_dt) < 0.03 & abs(thr_ds(:,5)) > 500;
if sum(mask_steady) > 10
    [u_max, idx_max] = max(abs(u_ds(mask_steady)));
    u_vals = u_ds(mask_steady);
    thr_vals = thr_ds(mask_steady, 5);
    % 找几个最大速度点
    [~, sort_idx] = sort(abs(u_vals), 'descend');
    top_n = min(20, length(sort_idx));
    u_top = u_vals(sort_idx(1:top_n));
    thr_top = abs(thr_vals(sort_idx(1:top_n)));
    % 用k*cmd = d1*u + d2*u^2
    % 至少需要2个速度点来解d1,d2, 但我们可以假设d1已知或设d1=0
    % 简化: 用本次辨识的k_x, 计算d1,d2
    k_use = results.surge.k;
    F_est = abs(k_use * thr_top);
    % d1*u + d2*u^2 = F, 用最小二乘含约束
    X_v = [abs(u_top), u_top.^2];
    coeff_v = lsqnonneg(X_v, F_est);
    d1_v = coeff_v(1); d2_v = coeff_v(2);
    fprintf('  k_x=%f, top%d |u|: %.3f~%.3f m/s, |cmd|: %.0f~%.0f\n', ...
        k_use, top_n, min(abs(u_top)), max(abs(u_top)), min(thr_top), max(thr_top));
    fprintf('  非负LS: d1=%.4f, d2=%.4f\n', d1_v, d2_v);
    % 用上次的d1作为先验,只估计d2
    d1_fixed = 1.35;
    d2_only = mean((F_est - d1_fixed*abs(u_top)) ./ (u_top.^2));
    fprintf('  固定d1=1.35: d2=%.4f\n', max(0, d2_only));
end

%% Sway: 最大速度匹配
fprintf('\n=== Sway 最大速度匹配 ===\n');
mask_sway = abs(dv_dt) < 0.02 & abs(thr_ds(:,3)+thr_ds(:,4)) > 200;
if sum(mask_sway) > 10
    v_vals = v_ds(mask_sway);
    cmd_vals = abs(thr_ds(mask_sway,3) + thr_ds(mask_sway,4));
    [~, sort_idx] = sort(abs(v_vals), 'descend');
    top_n = min(20, length(sort_idx));
    v_top = v_vals(sort_idx(1:top_n));
    cmd_top = cmd_vals(sort_idx(1:top_n));
    F_y = abs(results.sway.k) * cmd_top;
    X_vy = [abs(v_top), v_top.^2];
    coeff_vy = lsqnonneg(X_vy, F_y);
    fprintf('  非负LS: d1=%.4f, d2=%.4f\n', coeff_vy(1), coeff_vy(2));
end

%% Heave: 最大速度匹配
fprintf('\n=== Heave 最大速度匹配 ===\n');
mask_heave = abs(dw_dt) < 0.02 & abs(thr_ds(:,1)+thr_ds(:,2)) > 100;
if sum(mask_heave) > 10
    w_vals = w_ds(mask_heave);
    cmd_vals = abs(thr_ds(mask_heave,1) + thr_ds(mask_heave,2));
    [~, sort_idx] = sort(abs(w_vals), 'descend');
    top_n = min(20, length(sort_idx));
    w_top = w_vals(sort_idx(1:top_n));
    cmd_top = cmd_vals(sort_idx(1:top_n));
    F_z = abs(results.heave.k) * cmd_top;
    X_vz = [abs(w_top), w_top.^2];
    coeff_vz = lsqnonneg(X_vz, F_z);
    fprintf('  非负LS: d1=%.4f, d2=%.4f\n', coeff_vz(1), coeff_vz(2));
end

%% Yaw: 最大角速度匹配
fprintf('\n=== Yaw 最大角速度匹配 ===\n');
mask_yaw = abs(dr_dt) < 2.0 & abs(thr_ds(:,3)-thr_ds(:,4)) > 100;
if sum(mask_yaw) > 10
    r_vals = abs(r_ds(mask_yaw));
    cmd_vals = abs(thr_ds(mask_yaw,3) - thr_ds(mask_yaw,4));
    [~, sort_idx] = sort(r_vals, 'descend');
    top_n = min(20, length(sort_idx));
    r_top = r_vals(sort_idx(1:top_n));
    cmd_top = cmd_vals(sort_idx(1:top_n));
    Nz = abs(results.yaw.k) * cmd_top;
    % Nz = dr * r
    dr_est = mean(Nz ./ (r_top*pi/180 + 1e-6));
    fprintf('  估算 dr=%.4f N·m·s/rad\n', dr_est);
end

%% 汇总
fprintf('\n=== 汇总: 时域辨识 vs 最大速度匹配 ===\n');
fprintf('通道   | 时域辨识 k      | 上次 k        | 变化\n');
fprintf('Surge  | %.6f         | 0.000800      | %+.1f%%\n', results.surge.k, (results.surge.k/0.000800-1)*100);
fprintf('Sway   | %.6f         | ~0.0038       | —\n', abs(results.sway.k));
fprintf('Heave  | %.6f         | ~0.00062      | —\n', abs(results.heave.k));
fprintf('Yaw    | %.6f         | ~0.00014      | —\n', abs(results.yaw.k));

function dv = centered_diff(v, dt)
    n = length(v); dv = zeros(n, 1);
    dv(1) = (v(2)-v(1))/dt;
    for i = 2:n-1, dv(i) = (v(i+1)-v(i-1))/(2*dt); end
    dv(n) = (v(n)-v(n-1))/dt;
end
