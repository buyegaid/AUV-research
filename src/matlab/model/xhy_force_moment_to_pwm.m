function [pwm_us, info] = xhy_force_moment_to_pwm(tau_cmd, params, pwm_prev_us)
%XHY_FORCE_MOMENT_TO_PWM 将 XHY 六自由度力/力矩指令转换为 5 路 PWM 输出。
%
%   [pwm_us, info] = xhy_force_moment_to_pwm(tau_cmd)
%   [pwm_us, info] = xhy_force_moment_to_pwm(tau_cmd, params)
%   [pwm_us, info] = xhy_force_moment_to_pwm(tau_cmd, params, pwm_prev_us)
%
%   输入:
%     tau_cmd     6x1 或 1x6 向量 [TX TY TZ MX MY MZ]。
%                 默认单位按 Obsidian《推力分配》文档:
%                 TX/TY/TZ 为 g, MX/MY/MZ 为 N*m。
%     params      可选参数结构体:
%                 .moment_scale  力矩进入分配矩阵前的倍率, 默认 100
%                                (N*m -> g*cm, 与下位机接收逻辑一致)
%                 .ramp_step     PWM 平滑每周期最大变化量, 默认 50
%                 .enable_ramp   是否启用斜坡平滑, 默认由 pwm_prev_us 是否传入决定
%     pwm_prev_us 可选, 5x1 上一周期 PWM 占空比(us)。传入后默认启用 50 步平滑。
%
%   输出:
%     pwm_us      5x1 PWM 占空比(us), 推进器顺序:
%                 [T1前垂推 T2后垂推 T3前侧推 T4后侧推 T5主推]'
%     info        诊断信息, 包含分配前后推力、PWM 偏移量、限幅比例等。
%
%   说明:
%     本函数实现 Obsidian 笔记《推力分配》中的链路:
%       [TX TY TZ MX MY MZ] -> K 矩阵 -> 推进器推力(g)
%       -> 分段线性插值 PWM(-450~450) -> 1500us 中值占空比。

if nargin < 2 || isempty(params)
    params = struct();
end

if nargin < 3
    pwm_prev_us = [];
end

tau_cmd = tau_cmd(:);
if numel(tau_cmd) ~= 6
    error('xhy_force_moment_to_pwm:InvalidInput', ...
        'tau_cmd 必须为 6 维向量 [TX TY TZ MX MY MZ]。');
end

params = fill_default_params(params, ~isempty(pwm_prev_us));

if ~isempty(pwm_prev_us)
    pwm_prev_us = pwm_prev_us(:);
    if numel(pwm_prev_us) ~= 5
        error('xhy_force_moment_to_pwm:InvalidPrevPwm', ...
            'pwm_prev_us 必须为 5 维向量。');
    end
end

% 力矩帧接收后乘以 100, 再进入推力分配矩阵。
alloc_cmd = tau_cmd;
alloc_cmd(4:6) = alloc_cmd(4:6) * params.moment_scale;

% 文档中的 5x6 推力分配矩阵, 输出单位为 g。
K = [
    0,   0,  -0.5, 0,  0.0083,  0;
    0,   0,  -0.5, 0, -0.0083,  0;
    0,  -0.5, 0,   0,  0,      -0.0083;
    0,  -0.5, 0,   0,  0,       0.0083;
    1.0, 0,   0,   0,  0,       0
    ];

thrust_raw_g = K * alloc_cmd;
[thrust_g, scale_factor] = scale_thrust(thrust_raw_g, params.thrust_limit_pos_g, params.thrust_limit_neg_g);

pwm_offset_raw = thrust_to_pwm_offset(thrust_g, params);
pwm_offset_limited = min(params.pwm_max, max(-params.pwm_max, pwm_offset_raw));

if params.enable_ramp && ~isempty(pwm_prev_us)
    pwm_prev_offset = pwm_prev_us - params.pwm_center_us;
    delta = pwm_offset_limited - pwm_prev_offset;
    delta = min(params.ramp_step, max(-params.ramp_step, delta));
    pwm_offset = pwm_prev_offset + delta;
else
    pwm_offset = pwm_offset_limited;
end

pwm_us = params.pwm_center_us + pwm_offset;

info = struct();
info.allocation_cmd = alloc_cmd;
info.K = K;
info.thrust_raw_g = thrust_raw_g;
info.thrust_g = thrust_g;
info.scale_factor = scale_factor;
info.pwm_offset_raw = pwm_offset_raw;
info.pwm_offset_limited = pwm_offset_limited;
info.pwm_offset = pwm_offset;
info.pwm_us = pwm_us;
info.params = params;

end

function params = fill_default_params(params, has_prev_pwm)
%FILL_DEFAULT_PARAMS 补齐推力分配和 PWM 标定默认参数。

if ~isfield(params, 'moment_scale'), params.moment_scale = 100; end
if ~isfield(params, 'thrust_limit_pos_g'), params.thrust_limit_pos_g = 9500 * ones(5,1); end
if ~isfield(params, 'thrust_limit_neg_g'), params.thrust_limit_neg_g = 7300 * ones(5,1); end
if ~isfield(params, 'pwm_center_us'), params.pwm_center_us = 1500; end
if ~isfield(params, 'pwm_max'), params.pwm_max = 450; end
if ~isfield(params, 'ramp_step'), params.ramp_step = 50; end
if ~isfield(params, 'enable_ramp'), params.enable_ramp = has_prev_pwm; end

% 正向标定表: 推力(g) -> PWM 偏移。
if ~isfield(params, 'pos_thrust_g'), params.pos_thrust_g = [0; 3900; 5900; 7900; 9500]; end
if ~isfield(params, 'pos_pwm'), params.pos_pwm = [0; 230; 310; 370; 450]; end

% 反向标定表: 推力(g) -> PWM 偏移。interp1 要求自变量递增。
if ~isfield(params, 'neg_thrust_g'), params.neg_thrust_g = [-7300; -6100; -4800; -3200; 0]; end
if ~isfield(params, 'neg_pwm'), params.neg_pwm = [-430; -370; -330; -250; 0]; end

% 各推进器死区补偿, 顺序为 [T1 T2 T3 T4 T5]。
if ~isfield(params, 'deadzone_pos'), params.deadzone_pos = [35; 35; 36; 36; 31]; end
if ~isfield(params, 'deadzone_neg'), params.deadzone_neg = [-13; -13; -17; -17; -20]; end

params.thrust_limit_pos_g = params.thrust_limit_pos_g(:);
params.thrust_limit_neg_g = params.thrust_limit_neg_g(:);
params.deadzone_pos = params.deadzone_pos(:);
params.deadzone_neg = params.deadzone_neg(:);
end

function [thrust_scaled_g, scale_factor] = scale_thrust(thrust_g, limit_pos_g, limit_neg_g)
%SCALE_THRUST 按文档要求对所有推进器做等比例推力限幅。

ratio = zeros(5,1);
for i = 1:5
    if thrust_g(i) >= 0
        ratio(i) = abs(thrust_g(i)) / limit_pos_g(i);
    else
        ratio(i) = abs(thrust_g(i)) / limit_neg_g(i);
    end
end

scale_factor = max(1, max(ratio));
thrust_scaled_g = thrust_g / scale_factor;
end

function pwm_offset = thrust_to_pwm_offset(thrust_g, params)
%THRUST_TO_PWM_OFFSET 将推进器推力(g)插值为 PWM 偏移量。

pwm_offset = zeros(5,1);
for i = 1:5
    T = thrust_g(i);
    if abs(T) < eps
        pwm_offset(i) = 0;
    elseif T > 0
        pwm_offset(i) = interp1(params.pos_thrust_g, params.pos_pwm, T, 'linear', 'extrap');
        pwm_offset(i) = pwm_offset(i) + params.deadzone_pos(i);
    else
        pwm_offset(i) = interp1(params.neg_thrust_g, params.neg_pwm, T, 'linear', 'extrap');
        pwm_offset(i) = pwm_offset(i) + params.deadzone_neg(i);
    end
end
end
