% TEST_XHY_DRAG 测试XHY阻力模型
% 测试速度范围：0-1 m/s（平动），0-2 rad/s（转动）

clear; close all;

% 质量矩阵（从xhy.m）
m = 33;
Ix = 0.540804; Iy = 2.107488; Iz = 1.849137;
MRB = diag([m m m Ix Iy Iz]);
MA = diag([11.2773 132.1086 47.104 0.006 0.043 0.138]);
M = MRB + MA;

% 测试速度范围
u_range = linspace(0, 1, 50);
v_range = linspace(0, 1, 50);
w_range = linspace(0, 1, 50);
p_range = linspace(0, 2, 50);
q_range = linspace(0, 2, 50);
r_range = linspace(0, 2, 50);

%% 测试1：纵荡阻力 Fx(u)
Fx_data = zeros(size(u_range));
for i = 1:length(u_range)
    nu_r = [u_range(i); 0; 0; 0; 0; 0];
    [tau_drag, ~] = xhy_drag_cfd(nu_r, M);
    Fx_data(i) = tau_drag(1);
end

%% 测试2：横荡阻力 Fy(v)
Fy_data = zeros(size(v_range));
for i = 1:length(v_range)
    nu_r = [0; v_range(i); 0; 0; 0; 0];
    [tau_drag, ~] = xhy_drag_cfd(nu_r, M);
    Fy_data(i) = tau_drag(2);
end

%% 测试3：垂荡阻力 Fz(w)
Fz_data = zeros(size(w_range));
for i = 1:length(w_range)
    nu_r = [0; 0; w_range(i); 0; 0; 0];
    [tau_drag, ~] = xhy_drag_cfd(nu_r, M);
    Fz_data(i) = tau_drag(3);
end

%% 测试4：横滚阻尼 K(p)
K_data = zeros(size(p_range));
for i = 1:length(p_range)
    nu_r = [0; 0; 0; p_range(i); 0; 0];
    [tau_drag, ~] = xhy_drag_cfd(nu_r, M);
    K_data(i) = tau_drag(4);
end

%% 测试5：俯仰阻尼 M(q)
M_data = zeros(size(q_range));
for i = 1:length(q_range)
    nu_r = [0; 0; 0; 0; q_range(i); 0];
    [tau_drag, ~] = xhy_drag_cfd(nu_r, M);
    M_data(i) = tau_drag(5);
end

%% 测试6：偏航阻尼 N(r)
N_data = zeros(size(r_range));
for i = 1:length(r_range)
    nu_r = [0; 0; 0; 0; 0; r_range(i)];
    [tau_drag, ~] = xhy_drag_cfd(nu_r, M);
    N_data(i) = tau_drag(6);
end

%% 绘图
figure('Position', [100 100 1200 800]);

subplot(2,3,1)
plot(u_range, Fx_data, 'b-', 'LineWidth', 2);
grid on; xlabel('u (m/s)'); ylabel('Fx (N)');
title('纵荡阻力 Fx(u)');

subplot(2,3,2)
plot(v_range, Fy_data, 'r-', 'LineWidth', 2);
grid on; xlabel('v (m/s)'); ylabel('Fy (N)');
title('横荡阻力 Fy(v)');

subplot(2,3,3)
plot(w_range, Fz_data, 'g-', 'LineWidth', 2);
grid on; xlabel('w (m/s)'); ylabel('Fz (N)');
title('垂荡阻力 Fz(w)');

subplot(2,3,4)
plot(p_range, K_data, 'b-', 'LineWidth', 2);
grid on; xlabel('p (rad/s)'); ylabel('K (N·m)');
title('横滚阻尼 K(p)');

subplot(2,3,5)
plot(q_range, M_data, 'r-', 'LineWidth', 2);
grid on; xlabel('q (rad/s)'); ylabel('M (N·m)');
title('俯仰阻尼 M(q)');

subplot(2,3,6)
plot(r_range, N_data, 'g-', 'LineWidth', 2);
grid on; xlabel('r (rad/s)'); ylabel('N (N·m)');
title('偏航阻尼 N(r)');

sgtitle('XHY阻力模型测试（CFD+时间常数法）');

%% 输出关键数据
fprintf('=== XHY阻力模型测试结果 ===\n');
fprintf('纵荡 u=1.0m/s: Fx = %.2f N\n', Fx_data(end));
fprintf('横荡 v=1.0m/s: Fy = %.2f N\n', Fy_data(end));
fprintf('垂荡 w=1.0m/s: Fz = %.2f N\n', Fz_data(end));
fprintf('横滚 p=2.0rad/s: K = %.4f N·m\n', K_data(end));
fprintf('俯仰 q=2.0rad/s: M = %.4f N·m\n', M_data(end));
fprintf('偏航 r=2.0rad/s: N = %.4f N·m\n', N_data(end));
