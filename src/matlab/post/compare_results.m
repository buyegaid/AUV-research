function compare_results(hist1, hist2,text)

t = hist1.t;

% 指标
rmse_psi_1 = sqrt(mean(hist1.e_psi.^2));
rmse_psi_2   = sqrt(mean(hist2.e_psi.^2));

rmse_y_1 = sqrt(mean(hist1.e_y.^2));
rmse_y_2   = sqrt(mean(hist2.e_y.^2));

rms_delta_1 = sqrt(mean(hist1.ui(:,1).^2));
rms_delta_2   = sqrt(mean(hist2.ui(:,1).^2));

rms_sigma_1 = sqrt(mean(hist1.sigma_heading.^2));
rms_sigma_2   = sqrt(mean(hist2.sigma_heading.^2));
fprintf('===== %s =====', text);
fprintf('\n===== 跟踪效果对比 =====\n');
fprintf('航向误差 RMSE:    1 = %.6f, 2 = %.6f\n', rmse_psi_1, rmse_psi_2);
fprintf('横向误差 RMSE:    1 = %.6f, 2 = %.6f\n', rmse_y_1, rmse_y_2);
fprintf('舵角 RMS:         1 = %.6f, 2 = %.6f\n', rms_delta_1, rms_delta_2);
fprintf('滑模面 RMS:       1 = %.6f, 2 = %.6f\n', rms_sigma_1, rms_sigma_2);

figure('Name','轨迹对比');
plot(hist1.x(:,8), hist1.x(:,7), 'r', 'LineWidth',1.2); hold on;
plot(hist2.x(:,8),   hist2.x(:,7),   'b', 'LineWidth',1.2);
plot(hist1.traj.y, hist1.traj.x, 'k--', 'LineWidth',1);
grid on; axis equal;
legend('1','2','Reference');
xlabel('East (m)'); ylabel('North (m)');
title('轨迹对比');

figure('Name','航向误差对比');
plot(t, rad2deg(hist1.e_psi), 'r'); hold on;
plot(t, rad2deg(hist2.e_psi), 'b');
grid on;
legend('1','2');
xlabel('Time (s)'); ylabel('e_\psi (deg)');
title('航向误差对比');

figure('Name','横向误差对比');
plot(t, hist1.e_y, 'r'); hold on;
plot(t, hist2.e_y, 'b');
grid on;
legend('1','2');
xlabel('Time (s)'); ylabel('Cross-track error (m)');
title('横向误差对比');

figure('Name','舵角对比');
plot(t, rad2deg(hist1.ui(:,1)), 'r'); hold on;
plot(t, rad2deg(hist2.ui(:,1)), 'b');
grid on;
legend('1','2');
xlabel('Time (s)'); ylabel('\delta_r (deg)');
title('舵角输入对比');

figure('Name','滑模面对比');
plot(t, hist1.sigma_heading, 'r'); hold on;
plot(t, hist2.sigma_heading, 'b');
grid on;
legend('1','2');
xlabel('Time (s)'); ylabel('\sigma');
title('航向滑模面对比');

figure('Name','ESO估计扰动');
plot(t, hist2.hat_dr, 'b');
grid on;
xlabel('Time (s)'); ylabel('\hat{d}_r');
title('ESO估计的偏航扰动');

end
