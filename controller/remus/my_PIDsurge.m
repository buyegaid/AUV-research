function [n_cmd, term] = my_PIDsurge(u_ref, udot_ref, u, udot, param)
% AUV surge方向 PID 控制器
% 使用 persistent 保存积分项和上一时刻误差
%
% 与 get_params() 匹配的参数字段：
%   param.Kp
%   param.Ki
%   param.Kd
%   param.Ka
%   param.h
%   param.int_sep_th
%   param.use_accel_feedback
%   param.Poutrange   = [min, max]
%   param.Ioutrange   = [min, max]
%   param.Doutrange   = [min, max]
%   param.Aoutrange   = [min, max]
%   param.nrange      = [min, max]
%   param.int_range   = [min, max]
%   param.reset       = 0/1 (可选)
% 输出：
%   n_cmd: 推进器转速命令
%% ============ persistent变量 ============
persistent e_int e_prev is_initialized

%% ============ 默认参数处理 ============
if ~isfield(param, 'use_accel_feedback')
    param.use_accel_feedback = 1;
end

if ~isfield(param, 'reset')
    param.reset = 0;
end

if ~isfield(param, 'Ka')
    param.Ka = 0;
end

%% ============ 初始化 / 复位 ============
if isempty(is_initialized) || isempty(e_int) || isempty(e_prev) || param.reset == 1
    e_int = 0;
    e_prev = 0;
    is_initialized = true;
end

%% ============ 读取范围参数 ============
P_min   = param.Poutrange(1);
P_max   = param.Poutrange(2);

I_min   = param.Ioutrange(1);
I_max   = param.Ioutrange(2);

D_min   = param.Doutrange(1);
D_max   = param.Doutrange(2);

A_min   = param.Aoutrange(1);
A_max   = param.Aoutrange(2);

n_min   = param.nrange(1);
n_max   = param.nrange(2);

int_min = param.int_range(1);
int_max = param.int_range(2);

%% ============ 误差计算 ============
e    = u_ref - u;                 % 速度误差
edot = (e - e_prev) / param.h;    % 误差微分
ea   = udot_ref - udot;           % 加速度误差

%% ============ 积分分离 ============
if abs(e) <= param.int_sep_th
    int_on = 1;
    e_int = e_int + e * param.h;
else
    int_on = 0;
end

%% ============ 积分状态限幅 ============
e_int = min(max(e_int, int_min), int_max);

%% ============ 各环节计算 ============
% 比例项
P_out = param.Kp * e;
P_out = min(max(P_out, P_min), P_max);

% 积分项
I_out = param.Ki * e_int;
I_out = min(max(I_out, I_min), I_max);

% 微分项
D_out = param.Kd * edot;
D_out = min(max(D_out, D_min), D_max);

% 加速度补偿项
if param.use_accel_feedback == 1
    A_out = param.Ka * ea;
else
    A_out = param.Ka * udot_ref;
end
A_out = min(max(A_out, A_min), A_max);

%% ============ 总输出 ============
raw_out = P_out + I_out + D_out + A_out;
n_cmd = min(max(raw_out, n_min), n_max);

%% ============ 更新persistent变量 ============
e_prev = e;

%% ============ 调试输出 ============
term.e = e;
term.edot = edot;
term.ea = ea;
term.P_out = P_out;
term.I_out = I_out;
term.D_out = D_out;
term.A_out = A_out;
term.raw_out = raw_out;
term.n_cmd = n_cmd;
term.int_on = int_on;
term.e_int = e_int;

end
