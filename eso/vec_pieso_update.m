function [Z_next, aux] = vec_pieso_update(Z, y, a_known, params, dt, resetFlag)
% 物理信息ESO (PI-ESO): 嵌入Gauss-Markov海流模型
% 关键创新: Z3状态方程包含 -Lambda*Z3 项，体现海流时间相关性
%
% Inputs:
%   Z: 6x3 ESO状态 [Z1 Z2 Z3]
%   y: 6x1 测量值
%   a_known: 6x1 已知加速度
%   params: 参数结构体，需包含:
%     - tau_c: Gauss-Markov相关时间常数 (s)
%     - omega0_base: ESO基础带宽
%     - alpha1, alpha2, alpha3: fal函数参数
%     - delta: fal函数线性区宽度
%     - meas_lpf_fc: 测量值滤波截止频率
%     - z3_lpf_fc: Z3滤波截止频率
%     - use_rk4: 是否使用RK4积分
%   dt: 时间步长
%   resetFlag: 可选，重置persistent状态
%
% Outputs:
%   Z_next: 6x3 更新后的状态
%   aux: 诊断信息结构体

persistent y_filt_mem z3_filt_mem is_initialized

if nargin < 6
    resetFlag = false;
end

% 初始化persistent状态
if isempty(is_initialized) || resetFlag
    y_filt_mem = y;
    z3_filt_mem = Z(:,3);
    is_initialized = true;
end

% 1) 测量值低通滤波
y_filt = first_order_lpf(y_filt_mem, y, params.meas_lpf_fc, dt);

% 2) 自适应带宽（可选）
e = y_filt - Z(:,1);
err_norm = norm(e);

if isfield(params,'omega0_max') && params.omega0_max > params.omega0_base
    e_scale = params.adapt_err_scale;
    if isempty(e_scale)
        e_scale = 0.5;
    end
    fac = min(1, max(0, err_norm / e_scale));
    omega0 = params.omega0_base + fac * (params.omega0_max - params.omega0_base);
else
    omega0 = params.omega0_base;
end

% 3) Gauss-Markov衰减矩阵
Lambda = diag(ones(6,1) / params.tau_c);

% 4) ESO积分
if params.use_rk4
    fun = @(t, zvec) pieso_rhs(zvec, e, a_known, omega0, Lambda, params);
    zvec = [Z(:,1); Z(:,2); Z(:,3)];
    zvec_next = rk4_step(fun, 0, zvec, dt);
    
    Z1n = zvec_next(1:6);
    Z2n = zvec_next(7:12);
    Z3n = zvec_next(13:18);
else
    % Euler积分
    beta1 = 3 * omega0;
    beta2 = 3 * (omega0^2);
    beta3 = omega0^3;
    
    f1 = fal(e, params.alpha1, params.delta);
    f2 = fal(e, params.alpha2, params.delta);
    f3 = fal(e, params.alpha3, params.delta);
    
    Z1_dot = Z(:,2) + beta1 * f1;
    Z2_dot = Z(:,3) + a_known + beta2 * f2;
    Z3_dot = -Lambda * Z(:,3) + beta3 * f3;  % 物理信息项
    
    Z1n = Z(:,1) + Z1_dot * dt;
    Z2n = Z(:,2) + Z2_dot * dt;
    Z3n = Z(:,3) + Z3_dot * dt;
end

% 5) Z3低通滤波
z3_filt = first_order_lpf(z3_filt_mem, Z3n, params.z3_lpf_fc, dt);

% 6) 可选饱和限制
if isfield(params,'z3_sat') && params.z3_sat > 0
    z3_filt = max(-params.z3_sat, min(params.z3_sat, z3_filt));
end

% 输出
Z_next = [Z1n, Z2n, z3_filt];

% 更新persistent状态
y_filt_mem = y_filt;
z3_filt_mem = z3_filt;

% 诊断信息
aux.e = e;
aux.omega0 = omega0;
aux.a_known = a_known;
aux.y_filt = y_filt;
aux.z3_raw = Z3n;
aux.z3_filt = z3_filt;
aux.Lambda = Lambda;
end

% RK4右端函数
function dz = pieso_rhs(zvec, e, a_known, omega0, Lambda, params)
Z1 = zvec(1:6);
Z2 = zvec(7:12);
Z3 = zvec(13:18);

beta1 = 3 * omega0;
beta2 = 3 * (omega0^2);
beta3 = omega0^3;

f1 = fal(e, params.alpha1, params.delta);
f2 = fal(e, params.alpha2, params.delta);
f3 = fal(e, params.alpha3, params.delta);

Z1_dot = Z2 + beta1 * f1;
Z2_dot = Z3 + a_known + beta2 * f2;
Z3_dot = -Lambda * Z3 + beta3 * f3;  % 物理信息项

dz = [Z1_dot; Z2_dot; Z3_dot];
end

% RK4积分器
function y_next = rk4_step(fun, t, y, dt)
k1 = fun(t, y);
k2 = fun(t + 0.5*dt, y + 0.5*dt*k1);
k3 = fun(t + 0.5*dt, y + 0.5*dt*k2);
k4 = fun(t + dt, y + dt*k3);
y_next = y + (dt/6)*(k1 + 2*k2 + 2*k3 + k4);
end
