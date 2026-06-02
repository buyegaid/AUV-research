% 诊断垂推反向俯仰角速度为何为0
addpath('.', './Lib', './guidance', './controller/xhy', './controller/remus', './model', './eso', './post', './traj');

rho = 1026;
n_test = 1500;  % 中等RPM
Vc = 0; betaVc = 0; w_c = 0;
T_sim = 200;
h = 0.05;

% 工况: 垂推反向
ui = [0; n_test; -n_test; 0; 0];

T1 = thrust_aux(n_test, rho);
T2 = thrust_aux(-n_test, rho);
fprintf('T1(前垂)=%.3f N, T2(后垂)=%.3f N\n', T1, T2);

x = zeros(12, 1);
q_hist = zeros(T_sim/h, 1);
tau_hist = zeros(T_sim/h, 6);
t_hist = zeros(T_sim/h, 1);

idx = 1;
for t = 0:h:T_sim
    [xdot, nu_r, M_mat, C_mat, D_mat, g_vec, tau_out] = xhy(x, ui, Vc, betaVc, w_c);
    x = x + h * xdot;
    q_hist(idx) = x(5);
    tau_hist(idx,:) = tau_out';
    t_hist(idx) = t;
    idx = idx + 1;
end

fprintf('\n稳态状态:\n');
fprintf('  u=%.4f, v=%.4f, w=%.4f m/s\n', x(1), x(2), x(3));
fprintf('  p=%.4f, q=%.4f, r=%.4f rad/s\n', x(4), x(5), x(6));
fprintf('  q=%.2f deg/s\n', rad2deg(x(5)));
fprintf('  theta=%.4f rad (%.2f deg)\n', x(11), rad2deg(x(11)));

% 分析最后的tau_out
[xdot_final, nu_r_final, M_final, C_final, D_final, g_final, tau_final] = xhy(x, ui, Vc, betaVc, w_c);
fprintf('\n最终时刻:\n');
fprintf('  tau_out(5)=%.4f N·m (总力矩M分量)\n', tau_final(5));
fprintf('  g_vec(5)=%.4f N·m (恢复力矩)\n', g_final(5));
Cnu = C_final * nu_r_final;
fprintf('  C_nu(5)=%.4f N·m\n', Cnu(5));
fprintf('  M(5,5)=%.4f kg·m²\n', M_final(5,5));

% 时间序列图
figure('Position', [100, 100, 1000, 400]);
subplot(1,2,1);
plot(t_hist, rad2deg(q_hist));
xlabel('时间 (s)'); ylabel('q (deg/s)'); title('俯仰角速度 q 时序');
grid on;

subplot(1,2,2);
plot(t_hist, tau_hist(:,5));
xlabel('时间 (s)'); ylabel('M (N·m)'); title('俯仰力矩 M 时序');
grid on;

saveas(gcf, 'pic/diag_pitch.png');
fprintf('诊断图已保存\n');
