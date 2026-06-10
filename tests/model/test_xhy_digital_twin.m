%% test_xhy_digital_twin.m — XHY 数字孪生推进器链路验证
% 验证内容:
%   1. 单轴阶跃: Surge/Sway/Heave/Yaw 各通道独立测试
%   2. 标定增益: 验证水池标定后 model output vs 期望推力
%   3. 全链路: tau_cmd(N) → CAN-g → K矩阵 → PWM → M080/M060 → tau_actual
%   4. 自洽性: 标定后输出反推 DOF级 k 系数，与水池数据对比
%
% 运行方式: 从项目根目录 run
%   addpath(genpath('.'));
%   test_xhy_digital_twin

clear; close all; clc;

%% 初始化
cal = xhy_pool_calibration();
fprintf('===== XHY 数字孪生推进器链路验证 =====\n\n');

% 生成水池标定参数
p_m080 = cal.make_m080_params(24.0);
p_m060_v = cal.make_m060_vert_params(24.0);
p_m060_s = cal.make_m060_side_params(24.0);

fprintf('推进器标定增益:\n');
fprintf('  M080 T5主推:   fwd=%.4f, rev=%.4f\n', p_m080.thrust_gain_forward, p_m080.thrust_gain_reverse);
fprintf('  M060 T1/T2垂推: fwd=%.4f, rev=%.4f\n', p_m060_v.thrust_gain_forward, p_m060_v.thrust_gain_reverse);
fprintf('  M060 T3/T4侧推: fwd=%.4f, rev=%.4f\n', p_m060_s.thrust_gain_forward, p_m060_s.thrust_gain_reverse);

% xhy 数字孪生模式配置
opt.mode = 'pwm';
opt.thruster_params = {p_m060_v, p_m060_v, p_m060_s, p_m060_s, p_m080};
opt.voltage_v = 24.0;

fprintf('\n===== 1. 单轴阶跃测试 =====\n');

%% 1.1 Surge 通道（纯 TX → T5）
fprintf('\n--- Surge (TX) ---\n');
tau_cmd = [10; 0; 0; 0; 0; 0];  % 10N 前进力
fw_cmd = cal.tau_N_to_can_g(tau_cmd);
fprintf('tau_cmd: X=%.1f N → TX=%.0f CAN-g\n', tau_cmd(1), fw_cmd(1));

[pwm_doc, alloc_info] = xhy_force_moment_to_pwm(fw_cmd);
pwm = pwm_doc;
fprintf('PWM: T1=%.0f, T2=%.0f, T3=%.0f, T4=%.0f, T5=%.0f us\n', pwm);

% 直接调用推进器模型
t1_state = m060_thruster_model(pwm(1), 24.0, p_m060_v);
t2_state = m060_thruster_model(pwm(2), 24.0, p_m060_v);
t3_state = m060_thruster_model(pwm(3), 24.0, p_m060_s);
t4_state = m060_thruster_model(pwm(4), 24.0, p_m060_s);
t5_state = m080_thruster_model(pwm(5), 24.0, p_m080);
T_vec = [t1_state.thrust_n; t2_state.thrust_n; t3_state.thrust_n; t4_state.thrust_n; t5_state.thrust_n];

[B_thr, ~] = xhy_thruster_geometry();
tau_actual = B_thr * T_vec;

fprintf('推力: T5=%.2f N, T1=%.2f, T2=%.2f, T3=%.2f, T4=%.2f\n', T_vec);
fprintf('tau_actual: X=%.3f, Y=%.3f, Z=%.3f, M=%.4f, N=%.4f\n', ...
    tau_actual(1), tau_actual(2), tau_actual(3), tau_actual(5), tau_actual(6));
fprintf('Surge 误差: cmd=%.1f N, actual=%.2f N, rel=%.1f%%\n', ...
    tau_cmd(1), tau_actual(1), abs(tau_actual(1)-tau_cmd(1))/abs(tau_cmd(1))*100);

%% 1.2 Heave 通道（纯 TZ → T1+T2）
fprintf('\n--- Heave (TZ) ---\n');
tau_cmd = [0; 0; 5; 0; 0; 0];  % 5N 下沉力
fw_cmd = cal.tau_N_to_can_g(tau_cmd);
fprintf('tau_cmd: Z=%.1f N → TZ=%.0f CAN-g\n', tau_cmd(3), fw_cmd(3));

[pwm_doc, ~] = xhy_force_moment_to_pwm(fw_cmd);
pwm = pwm_doc;

t1_s = m060_thruster_model(pwm(1), 24.0, p_m060_v);
t2_s = m060_thruster_model(pwm(2), 24.0, p_m060_v);
t3_s = m060_thruster_model(pwm(3), 24.0, p_m060_s);
t4_s = m060_thruster_model(pwm(4), 24.0, p_m060_s);
t5_s = m080_thruster_model(pwm(5), 24.0, p_m080);
T_vec = [t1_s.thrust_n; t2_s.thrust_n; t3_s.thrust_n; t4_s.thrust_n; t5_s.thrust_n];
tau_actual = B_thr * T_vec;

fprintf('推力: T1=%.2f, T2=%.2f N (垂推)\n', T_vec(1), T_vec(2));
fprintf('Heave 误差: cmd=%.1f N, actual=%.2f N, rel=%.1f%%\n', ...
    tau_cmd(3), tau_actual(3), abs(tau_actual(3)-tau_cmd(3))/abs(tau_cmd(3))*100);

%% 1.3 Yaw 通道（纯 MZ → T3−T4 差动）
fprintf('\n--- Yaw (MZ) ---\n');
tau_cmd = [0; 0; 0; 0; 0; 0.5];  % 0.5 N·m 偏航力矩
fw_cmd = cal.tau_N_to_can_g(tau_cmd);
fprintf('tau_cmd: N=%.2f N·m → MZ=%.0f CAN-g\n', tau_cmd(6), fw_cmd(6));

[pwm_doc, ~] = xhy_force_moment_to_pwm(fw_cmd);
pwm = pwm_doc;
fprintf('PWM: T3=%.0f, T4=%.0f us (侧推差动)\n', pwm(3), pwm(4));

t3_s = m060_thruster_model(pwm(3), 24.0, p_m060_s);
t4_s = m060_thruster_model(pwm(4), 24.0, p_m060_s);
t5_s = m080_thruster_model(pwm(5), 24.0, p_m080);
T_vec = [0; 0; t3_s.thrust_n; t4_s.thrust_n; t5_s.thrust_n];
tau_actual = B_thr * T_vec;

fprintf('推力: T3=%.3f, T4=%.3f N\n', T_vec(3), T_vec(4));
fprintf('Yaw 误差: cmd=%.2f N·m, actual=%.3f N·m, rel=%.1f%%\n', ...
    tau_cmd(6), tau_actual(6), abs(tau_actual(6)-tau_cmd(6))/abs(tau_cmd(6))*100);

%% 1.4 偏航-侧移解耦检验
fprintf('\n--- 偏航-侧移解耦 ---\n');
sway_residual = abs(tau_actual(2));
fprintf('纯MZ指令下残余侧移力: %.4f N (应接近0)\n', sway_residual);
if sway_residual < 0.5
    fprintf('✅ 解耦良好\n');
else
    fprintf('⚠️ 侧移残余偏大，需检查T3/T4不对称标定\n');
end

fprintf('\n===== 2. 标定前后对比 =====\n');

%% 2.1 M080 满指令对比
fprintf('\n--- M080 满指令 (PWM=1850us, 24V) ---\n');
p_m080_raw = m080_thruster_params();  % 未标定
s_raw = m080_thruster_model(1850, 24.0, p_m080_raw);
s_cal = m080_thruster_model(1850, 24.0, p_m080);
fprintf('未标定: %.1f N (厂商静推力)\n', s_raw.thrust_n);
fprintf('已标定: %.2f N (水池校准)\n', s_cal.thrust_n);
fprintf('缩放因子: %.4f\n', s_cal.thrust_n / s_raw.thrust_n);

%% 2.2 M060 满指令对比
fprintf('\n--- M060 满指令 (PWM=1850us, 24V) ---\n');
p_m060_raw = m060_thruster_params();
s_raw = m060_thruster_model(1850, 24.0, p_m060_raw);
s_cal = m060_thruster_model(1850, 24.0, p_m060_s);
fprintf('未标定: %.1f N (厂商静推力)\n', s_raw.thrust_n);
fprintf('已标定(侧推): %.2f N (水池校准)\n', s_cal.thrust_n);
fprintf('已标定(垂推): %.2f N (水池校准)\n', ...
    m060_thruster_model(1850, 24.0, p_m060_v).thrust_n);

fprintf('\n===== 3. 全链路自洽检验 =====\n');

%% 3.1 反推 DOF 级 k 系数
% 通过单轴阶跃，验证标定后链路的端到端增益是否还原水池 k 值

% Surge: X = k_X * TX
tx_test = 6400;  % 水池参考工作点
fw_test = [tx_test; 0; 0; 0; 0; 0];
[pwm_doc, ~] = xhy_force_moment_to_pwm(fw_test);
t5_s = m080_thruster_model(pwm_doc(5), 24.0, p_m080);
tau_test = B_thr * [0; 0; 0; 0; t5_s.thrust_n];
k_X_actual = tau_test(1) / tx_test;
fprintf('\nSurge: k_X(水池)=%.6f, k_X(标定后)=%.6f N/CAN-g, 误差=%.2f%%\n', ...
    cal.k_X, k_X_actual, abs(k_X_actual-cal.k_X)/abs(cal.k_X)*100);

% Heave: Z = k_Z * TZ
tz_test = 8000;
fw_test = [0; 0; tz_test; 0; 0; 0];
[pwm_doc, ~] = xhy_force_moment_to_pwm(fw_test);
t1_s = m060_thruster_model(pwm_doc(1), 24.0, p_m060_v);
t2_s = m060_thruster_model(pwm_doc(2), 24.0, p_m060_v);
tau_test = B_thr * [t1_s.thrust_n; t2_s.thrust_n; 0; 0; 0];
k_Z_actual = tau_test(3) / tz_test;
fprintf('Heave: k_Z(水池)=%.6f, k_Z(标定后)=%.6f N/CAN-g, 误差=%.2f%%\n', ...
    cal.k_Z, k_Z_actual, abs(k_Z_actual-cal.k_Z)/abs(cal.k_Z)*100);

fprintf('\n===== 4. 推力曲线扫描 =====\n');

%% 4.1 PWM 扫描
pwm_range = [1150:50:1500, 1500:50:1850]';
n_pts = length(pwm_range);

thrust_m080_raw = zeros(n_pts, 1);
thrust_m080_cal = zeros(n_pts, 1);
thrust_m060_raw = zeros(n_pts, 1);
thrust_m060_v_cal = zeros(n_pts, 1);
thrust_m060_s_cal = zeros(n_pts, 1);

for i = 1:n_pts
    pw = pwm_range(i);
    thrust_m080_raw(i) = m080_thruster_model(pw, 24.0, p_m080_raw).thrust_n;
    thrust_m080_cal(i) = m080_thruster_model(pw, 24.0, p_m080).thrust_n;
    thrust_m060_raw(i) = m060_thruster_model(pw, 24.0, p_m060_raw).thrust_n;
    thrust_m060_v_cal(i) = m060_thruster_model(pw, 24.0, p_m060_v).thrust_n;
    thrust_m060_s_cal(i) = m060_thruster_model(pw, 24.0, p_m060_s).thrust_n;
end

figure('Position', [100, 100, 1200, 500]);

subplot(1,2,1);
plot(pwm_range, thrust_m080_raw, 'b--', 'LineWidth', 1.5); hold on;
plot(pwm_range, thrust_m080_cal, 'r-', 'LineWidth', 2);
xline(1500, 'k:'); yline(0, 'k-');
xlabel('PWM (us)'); ylabel('推力 (N)');
title('M080 主推标定前后对比');
legend('厂商原始', '水池标定', 'Location', 'northwest'); grid on;

subplot(1,2,2);
plot(pwm_range, thrust_m060_raw, 'b--', 'LineWidth', 1.5); hold on;
plot(pwm_range, thrust_m060_v_cal, 'r-', 'LineWidth', 2);
plot(pwm_range, thrust_m060_s_cal, 'g-', 'LineWidth', 2);
xline(1500, 'k:'); yline(0, 'k-');
xlabel('PWM (us)'); ylabel('推力 (N)');
title('M060 辅推标定前后对比');
legend('厂商原始', '水池标定(垂推)', '水池标定(侧推)', 'Location', 'northwest'); grid on;

sgtitle('XHY 推进器数字孪生标定验证');

fprintf('\n图表已生成。\n');
fprintf('===== 验证完成 =====\n');
