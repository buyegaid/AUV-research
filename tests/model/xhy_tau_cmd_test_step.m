function [xdot, tau_thr, thr, pwm_us_doc, alloc_info] = xhy_tau_cmd_test_step(x, tau_cmd, voltage_v, Vc, betaVc, w_c, alloc_params, pwm_prev_us_doc)
%XHY_TAU_CMD_TEST_STEP 使用实艇力/力矩指令链路计算 XHY 动力学一步导数。
%
%   tau_cmd 的单位和顺序按 Obsidian《推力分配》:
%   [TX TY TZ MX MY MZ]'，其中 TX/TY/TZ 为 CAN-g，MX/MY/MZ 为 N*m。
%   输出 PWM 先按文档顺序 [T1 T2 T3 T4 T5]' 生成，再重排为
%   xhy_pwm_test_step 需要的 [T5 T1 T2 T3 T4]'。

if nargin < 3 || isempty(voltage_v), voltage_v = 24; end
if nargin < 4 || isempty(Vc), Vc = 0; end
if nargin < 5 || isempty(betaVc), betaVc = 0; end
if nargin < 6 || isempty(w_c), w_c = 0; end
if nargin < 7, alloc_params = []; end
if nargin < 8, pwm_prev_us_doc = []; end

if isempty(pwm_prev_us_doc)
    [pwm_us_doc, alloc_info] = xhy_force_moment_to_pwm(tau_cmd, alloc_params);
else
    [pwm_us_doc, alloc_info] = xhy_force_moment_to_pwm(tau_cmd, alloc_params, pwm_prev_us_doc);
end

% 文档顺序为 [T1 T2 T3 T4 T5]，动力学 PWM 步进顺序为 [T5 T1 T2 T3 T4]。
pwm_us_dyn = [pwm_us_doc(5); pwm_us_doc(1:4)];

if ~isscalar(voltage_v)
    voltage_v = voltage_v(:);
    if numel(voltage_v) ~= 5
        error('xhy_tau_cmd_test_step:InvalidVoltage', 'voltage_v 必须为标量或 5 维向量。');
    end
    voltage_v = [voltage_v(5); voltage_v(1:4)];
end

[xdot, tau_thr, thr] = xhy_pwm_test_step(x, pwm_us_dyn, voltage_v, Vc, betaVc, w_c);
thr.pwm_us_doc = pwm_us_doc;
thr.pwm_us_dyn = pwm_us_dyn;
thr.alloc_info = alloc_info;
end
