function plot_compare_single(hist, titleName)

t = hist.t;

figure('Name',['轨迹-' titleName]);
plot(hist.x(:,8), hist.x(:,7), 'b', 'LineWidth',1.2); hold on;
plot(hist.traj.y, hist.traj.x, 'k--', 'LineWidth',1);
grid on; axis equal;
legend(titleName,'Reference');
ylabel('North (m)'); xlabel('East (m)');
title(['轨迹-' titleName]);

figure('Name',['航向误差-' titleName]);
plot(t, rad2deg(hist.e_psi), 'b');
grid on;
xlabel('Time (s)'); ylabel('e_\psi (deg)');
title(['航向误差-' titleName]);

figure('Name',['横向误差-' titleName]);
plot(t, hist.e_y, 'b');
grid on;
xlabel('Time (s)'); ylabel('Cross-track error (m)');
title(['横向误差-' titleName]);

if hist.useESO
    figure('Name',['ESO扰动估计-' titleName]);
    plot(t, hist.hat_dr, 'b');
    grid on;
    xlabel('Time (s)'); ylabel('$\hat{d_r}$');
    title(['ESO扰动估计-' titleName]);
end

end
