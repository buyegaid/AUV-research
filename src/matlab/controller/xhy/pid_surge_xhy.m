function [X_cmd, term] = pid_surge_xhy(u, u_d, u_d_dot, h, params)
% PID_SURGE_XHY XHY纵荡速度单环PID控制器，输出期望推力X_cmd (N)
%
%   基于XHY动力学的纵荡通道PID控制器：
%     m_eff * u_dot = X - D(u) + 扰动
%   其中 m_eff = m + Xu_dot, D_linear = m_eff / T1 (线性化阻尼)
%
%   控制律（四项求和）：
%     X_P  = Kp * e                     % 比例：速度误差直接放大
%     X_I  = Ki * ∫e dt                % 积分：消除稳态误差（含分离+限幅）
%     X_D  = Kd * du_filt/dt           % 微分：测量值微分+低通滤波，抑制超调
%     X_FF = m_eff*u_d_dot + D_linear*u_d  % 前馈：模型加速度+阻尼补偿
%     X_cmd = sat(X_P + X_I + X_D + X_FF, [X_min, X_max])
%
%   输入:
%     u       - 当前纵荡速度 (m/s)
%     u_d     - 期望纵荡速度 (m/s)
%     u_d_dot - 期望纵荡加速度 (m/s²)，前馈用，无可传0
%     h       - 采样时间 (s)
%     params  - 参数结构体，字段:
%         .m_eff       - 有效质量 m + Xu_dot (kg)，默认101.642
%         .T1          - 纵荡时间常数 (s)，默认20
%         .Kp          - 比例增益，默认50
%         .Ki          - 积分增益，默认5
%         .Kd          - 微分增益，默认30
%         .X_max       - 推力上限 (N)，默认10
%         .X_min       - 推力下限 (N)，默认-3
%         .int_sep_th  - 积分分离阈值 (m/s)，|e|≤此值时积分使能，默认0.1
%         .int_max     - 积分状态幅值上限 (m·s)，默认3
%         .lpf_alpha   - 微分低通滤波系数 [0,1]，0=无滤波，默认0.7
%         .reset       - 复位标志，1=清零积分和微分状态
%         .use_ff      - 是否启用模型前馈，默认true
%
%   输出:
%     X_cmd - 期望纵向推力 (N)，正值=前进推力
%     term  - 诊断信息结构体，含各环节分量和状态量
%
%   设计说明:
%     - 微分采用测量值(u)而非误差(e)微分，避免给定值突变引起微分冲击
%     - 微分通道含一阶低通滤波 d_filt = α·d_prev + (1-α)·(u-u_prev)/h
%     - 积分分离：大误差时暂停积分，防止剧烈机动时积分windup
%     - 抗饱和：积分状态限幅 + 输出饱和时反向修正积分（back-calculation）
%     - 前馈项基于线性化名义模型，PID负责抑制模型失配和扰动
%
%   See also: smc_surge_xhy, thrust_allocation_xhy

%% ============ persistent 状态变量 ============
persistent e_int     % 积分累加状态 (m·s)
persistent u_prev    % 上一时刻速度，用于微分 (m/s)
persistent d_filt    % 滤波后的微分值 (m/s²)
persistent is_init   % 初始化标志

%% ============ 默认参数填充 ============
if ~isfield(params, 'm_eff'),       params.m_eff = 101.642;  end
if ~isfield(params, 'T1'),          params.T1 = 20;          end
if ~isfield(params, 'Kp'),          params.Kp = 50;          end
if ~isfield(params, 'Ki'),          params.Ki = 5;           end
if ~isfield(params, 'Kd'),          params.Kd = 30;          end
if ~isfield(params, 'X_max'),       params.X_max = 10;       end
if ~isfield(params, 'X_min'),       params.X_min = -3;       end
if ~isfield(params, 'int_sep_th'),  params.int_sep_th = 0.1; end
if ~isfield(params, 'int_max'),     params.int_max = 3;      end
if ~isfield(params, 'lpf_alpha'),   params.lpf_alpha = 0.7;  end
if ~isfield(params, 'reset'),       params.reset = 0;        end
if ~isfield(params, 'use_ff'),      params.use_ff = true;    end

%% ============ 初始化 / 复位 ============
if isempty(is_init) || params.reset
    e_int  = 0;
    u_prev = u;
    d_filt = 0;
    is_init = true;
end

%% ============ 误差计算 ============
% 标准PID误差定义: e = 期望值 - 实测值
% e > 0 → 速度不足，需增大推力加速; e < 0 → 速度过大，需减小推力减速
e = u_d - u;

%% ============ 比例项 ============
P_out = params.Kp * e;

%% ============ 积分项（含分离 + 限幅） ============
if abs(e) <= params.int_sep_th
    e_int = e_int + e * h;
end
e_int = max(-params.int_max, min(params.int_max, e_int));
I_out = params.Ki * e_int;

%% ============ 微分项（测量值微分 + 低通滤波） ============
% 离散一阶低通: d_filt(k) = α·d_filt(k-1) + (1-α)·Δu/h
% 连续等价时间常数 τ_filt = h·α/(1-α)，α=0.7→τ_filt≈0.023s
u_dot_raw = (u - u_prev) / h;
d_filt = params.lpf_alpha * d_filt + (1 - params.lpf_alpha) * u_dot_raw;
D_out = params.Kd * d_filt;

%% ============ 模型前馈 ============
if params.use_ff
    D_linear = params.m_eff / params.T1;
    X_ff = params.m_eff * u_d_dot + D_linear * u_d;
else
    X_ff = 0;
end

%% ============ 合成与输出限幅 ============
X_raw = P_out + I_out + D_out + X_ff;
X_cmd = max(params.X_min, min(params.X_max, X_raw));

%% ============ 抗饱和：输出饱和时反向修正积分 ============
% 使用back-calculation方法，防止积分在输出饱和时继续累加
if params.Ki > 0 && abs(X_cmd - X_raw) > 1e-6
    excess = X_raw - X_cmd;
    % 将超额部分按Ki增益折算为积分修正量
    e_int = e_int - excess / params.Ki * h;
    e_int = max(-params.int_max, min(params.int_max, e_int));
end

%% ============ 更新 persistent 状态 ============
u_prev = u;

%% ============ 诊断输出 ============
term.e      = e;
term.P_out  = P_out;
term.I_out  = I_out;
term.D_out  = D_out;
term.X_ff   = X_ff;
term.X_raw  = X_raw;
term.X_cmd  = X_cmd;
term.e_int  = e_int;
term.d_filt = d_filt;
term.u_dot_raw = u_dot_raw;

end
