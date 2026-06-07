function [Z_next, aux] = vec_leso_update_adv(Z, y, a_known, params, dt, resetFlag)
% Vectorized advanced LESO update with:
% - measurement LPF
% - RK4 integration option
% - Z3 lowpass filter
% - optional adaptive bandwidth
% - internal persistent filter states
%
% Inputs:
%   Z: 6x3 current LESO state columns [Z1 Z2 Z3] ESO状态变量
%   y: 6x1 raw measurement vector 传感器测量值
%   a_known: 6x1 已知加速度
%   params: parameter struct 包含以下字段:
%   dt: timestep 时间步长
%   resetFlag: optional, true to reset persistent states
%
% Outputs:
%   Z_next: 6x3 updated states
%   aux: diagnostic struct

persistent y_filt_mem z3_filt_mem is_initialized

if nargin < 7
    resetFlag = false;
end

% 若首次调用或要求重置，则初始化 persistent 状态
if isempty(is_initialized) || resetFlag
    y_filt_mem = y;
    z3_filt_mem = Z(:,3);
    is_initialized = true;
end

% 1) 测量值低通滤波
y_filt = first_order_lpf(y_filt_mem, y, params.meas_lpf_fc, dt);

% 2) 自适应带宽
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

% observer integration
if params.use_rk4
    fun = @(t, zvec) leso_rhs(zvec,e, a_known, omega0, params);
    zvec = [Z(:,1); Z(:,2); Z(:,3)];
    zvec_next = rk4_step(fun, 0, zvec, dt);
    
    Z1n = zvec_next(1:6);
    Z2n = zvec_next(7:12);
    Z3n = zvec_next(13:18);
else
    Z1_dot = Z(:,2) + B1 * f1;
    Z2_dot = Z(:,3) + a_known + B2 * f2;
    Z3_dot = B3 * f3;
    
    Z1n = Z(:,1) + Z1_dot * dt;
    Z2n = Z(:,2) + Z2_dot * dt;
    Z3n = Z(:,3) + Z3_dot * dt;
end

% 3) Z3 lowpass filtering
z3_filt = first_order_lpf(z3_filt_mem, Z3n, params.z3_lpf_fc, dt);

% 4) optional saturation
if isfield(params,'z3_sat') && params.z3_sat > 0
    z3_filt = max(-params.z3_sat, min(params.z3_sat, z3_filt));
end

% output  Z3输出滤波后的数据
Z_next = [Z1n, Z2n, z3_filt];

% update persistent memory
y_filt_mem = y_filt;
z3_filt_mem = z3_filt;

% aux
aux.e = e;
aux.omega0 = omega0;
aux.a_known = a_known;
aux.y_filt = y_filt;
aux.z3_raw = Z3n;
aux.z3_filt = z3_filt;
end

% helper for RK4 right-hand side
function dz = leso_rhs(zvec,e, a_known, omega0,params)
Z1 = zvec(1:6);
Z2 = zvec(7:12);
Z3 = zvec(13:18);
% 增益
beta1 = 3 * omega0;
beta2 = 3 * (omega0^2);
beta3 = omega0^3;

f1 = fal(e, params.alpha1, params.delta);
f2 = fal(e, params.alpha2, params.delta);
f3 = fal(e, params.alpha3, params.delta);

Z1_dot = Z2 + beta1 * f1;
Z2_dot = Z3 + a_known + beta2 * f2;
Z3_dot = beta3 * f3;

dz = [Z1_dot; Z2_dot; Z3_dot];
end

function y_next = rk4_step(fun, t, y, dt)
k1 = fun(t, y);
k2 = fun(t + 0.5*dt, y + 0.5*dt*k1);
k3 = fun(t + 0.5*dt, y + 0.5*dt*k2);
k4 = fun(t + dt,       y + dt*k3);
y_next = y + (dt/6)*(k1 + 2*k2 + 2*k3 + k4);
end
