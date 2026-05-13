function plot_xhy_results(hist)
% PLOT_XHY_RESULTS 绘制XHY仿真结果

t = hist.t;
x = hist.x;
ui = hist.ui;
xd = hist.xd;  % [psi_d theta_d psi_ref theta_ref]

figure('Name','XHY仿真结果');

% 3D轨迹（NED坐标，z向下为正，显示时取反使深度向上）
subplot(2,3,1);
plot3(x(:,7), x(:,8), x(:,9)); hold on;
plot3(hist.traj.pos.x, hist.traj.pos.y, hist.traj.pos.z, 'r--');
xlabel('North (m)'); ylabel('East (m)'); zlabel('Down (m)');
title('3D轨迹 (NED)'); legend('实际','期望'); grid on;
set(gca, 'ZDir', 'reverse');  % z轴向下为正，视觉上深度向下

% 航向角
subplot(2,3,2);
plot(t, rad2deg(x(:,12))); hold on;
plot(t, rad2deg(xd(:,1)), 'r--');
xlabel('t (s)'); ylabel('psi (deg)');
title('航向角'); legend('实际','期望'); grid on;

% 俯仰角
subplot(2,3,3);
plot(t, rad2deg(x(:,11))); hold on;
plot(t, rad2deg(xd(:,2)), 'r--');
xlabel('t (s)'); ylabel('theta (deg)');
title('俯仰角'); legend('实际','期望'); grid on;

% 速度
subplot(2,3,4);
plot(t, x(:,1));
xlabel('t (s)'); ylabel('u (m/s)');
title('纵荡速度'); grid on;

% 推进器转速
subplot(2,3,5);
plot(t, ui);
xlabel('t (s)'); ylabel('RPM');
title('推进器转速');
legend('主推','垂直1','垂直2','侧向1','侧向2'); grid on;

% 深度
subplot(2,3,6);
plot(t, x(:,9));
xlabel('t (s)'); ylabel('depth (m)');
title('深度'); grid on;

end
