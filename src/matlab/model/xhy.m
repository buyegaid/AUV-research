function [xdot,U,M,C,D,g,tau]=xhy(x,ui,Vc,betaVc,w_c,opt)
% XHY 5-thruster AUV dynamics
%   x   = [u v w p q r x y z phi theta psi]'  (12 states)
%   ui  = 控制输入，含义取决于 opt.mode:
%         'rpm' (默认): [T1 T2 T3 T4 T5]' (RPM)
%         'pwm':        [T1 T2 T3 T4 T5]' (PWM us)
%   Vc, betaVc, w_c are optional ocean current inputs
%   opt - 可选参数结构体:
%         .mode:            'rpm' (默认, 向后兼容) | 'pwm' (数字孪生)
%         .thruster_params: 1×5 cell, PWM模式下各推进器的标定参数
%                           顺序: {M060_T1, M060_T2, M060_T3, M060_T4, M080_T5}
%         .voltage_v:       标量或 1×5 向量, 供电电压 (默认 24V)
%   Outputs are the state derivative xdot, speed U, and model matrices/forces.

% 只有状态变量输入
if nargin < 2
    x = zeros(12,1);
    ui = zeros(5,1);
end

% 只有状态变量和控制输入
if nargin < 3
    Vc = 0;
end

% 海流输入不完整时默认值
if nargin < 4
    betaVc = 0;
end

% 海流垂向分量默认值
if nargin < 5
    w_c = 0;
end

% 可选参数结构体默认值
if nargin < 6 || isempty(opt)
    opt = struct('mode', 'rpm'); % 默认使用 RPM 模式
end


if ~isfield(opt, 'mode'), opt.mode = 'rpm'; end

if ~isfield(opt, 'thruster_params'), opt.thruster_params = {}; end

if ~isfield(opt, 'voltage_v'), opt.voltage_v = 24.0; end

pwm_mode = strcmp(opt.mode, 'pwm');


if (length(ui) ~= 5)
    error('ui-vector must have dimension 5!');
end

if (length(x) ~= 12)
    error('x-vector must have dimension 12');
end

% Constants
mu = 63.446827;         % 纬度
g_mu = gravity(mu);     % 重力加速度 (m/s^2)
rho = 1000;             % 水的密度 (kg/m^3) % 默认为淡水

nu = x(1:6);
nu_2 = nu(4:6);
psi = x(12); % 航向角

% 海流速度在船体坐标系中的分量
u_c_x = Vc * cos(betaVc - psi);
u_c_y = Vc * sin(betaVc - psi);
nu_c = [u_c_x; u_c_y; w_c; 0; 0; 0];
Dnu_c = [nu(6) * u_c_y; -nu(6) * u_c_x; 0; 0; 0; 0];

% 计算相对速度
nu_r = nu - nu_c;
U_r = norm(nu_r(1:3)); % 总相对速度
U   = norm(nu(1:3));   % 总绝对速度

r_bG = [0 0 0]';             % 重心位置(m)
% 等效稳心/浮心恢复力臂 12 cm；Body/NED 均采用 z 向下约定，负 z 表示浮心在重心上方。
r_bB = [0 0 -0.12]';         % 浮心位置(m)

% 刚体惯性矩阵
m=85.832;                      % 质量(kg, 2026-06-21: 恢复原始值)
Ix=1.419139;                   % 绕x轴转动惯量(kg·m²)
Iy=7.173529;                   % 绕y轴转动惯量(kg·m²)
Iz=6.390605;                   % 绕z轴转动惯量(kg·m²)

Ig = diag([Ix Iy Iz]);
MRB=diag([ m m m Ix Iy Iz ]);
MA=diag([ 15.81 124.73 42.87 0.014 0.041 0.123 ]);

O3 = zeros(3,3);
CRB = [ m * Smtrx(nu_2)    O3
    O3               -Smtrx(Ig*nu_2) ];
CA=m2c(MA, nu_r);
% 如果缺少二次旋转阻尼，横滚、俯仰和偏航中的 CA 项会使模型不稳定。这些项被假定为零。
CA(5,3) = 0; CA(3,5) = 0;
CA(5,1) = 0; CA(1,5) = 0;
CA(6,1) = 0; CA(1,6) = 0;
C = CRB + CA;
M = MRB + MA;

W = m * g_mu;
B = W; % 浮力等于重力

% 计算阻力（基于CFD仿真数据）
[tau_drag, D] = xhy_drag_cfd(nu_r);

% 运动学和重力/浮力矩阵
[J,R] = eulerang(x(10), x(11), x(12));
g = gRvect(W, B, R, r_bG, r_bB);

% ===== 推进器推力计算 =====
[B_thr, ~] = xhy_thruster_geometry();

if pwm_mode
    % 数字孪生模式: ui = PWM us, 顺序 [T1 T2 T3 T4 T5]
    voltage_v = opt.voltage_v;
    if isscalar(voltage_v)
        voltage_v = voltage_v * ones(5,1);
    end
    
    % 各推进器标定参数
    if ~isempty(opt.thruster_params)
        thr_p = opt.thruster_params;  % 1×5 cell
    else
        % 默认使用未标定的厂商参数
        thr_p = {m060_thruster_params(), m060_thruster_params(), ...
            m060_thruster_params(), m060_thruster_params(), ...
            m080_thruster_params()};
    end
    
    % 逐推进器调用 M080/M060 模型
    T_vert1 = m060_thruster_model(ui(1), voltage_v(1), thr_p{1});
    T_vert2 = m060_thruster_model(ui(2), voltage_v(2), thr_p{2});
    T_side1 = m060_thruster_model(ui(3), voltage_v(3), thr_p{3});
    T_side2 = m060_thruster_model(ui(4), voltage_v(4), thr_p{4});
    T_main  = m080_thruster_model(ui(5), voltage_v(5), thr_p{5});
    
    T_vec = [
        T_vert1.thrust_n;
        T_vert2.thrust_n;
        T_side1.thrust_n;
        T_side2.thrust_n;
        T_main.thrust_n
        ];
else
    % 传统 RPM 模式: ui = RPM, 顺序 [T1 T2 T3 T4 T5]
    n_max = 2500;            % 额定最大转速 (RPM)
    n_rpm = sat(ui, n_max);  % 限幅
    
    % 推力计算：主推直径10cm，辅助推直径6cm
    T_vert1 = thrust_aux(n_rpm(1), rho);
    T_vert2 = thrust_aux(n_rpm(2), rho);
    T_side1 = thrust_aux(n_rpm(3), rho);
    T_side2 = thrust_aux(n_rpm(4), rho);
    T_main  = thrust_main(n_rpm(5), rho);
    
    T_vec = [T_vert1; T_vert2; T_side1; T_side2; T_main];
end

tau = B_thr * T_vec;



% 总输入力矩 = 推进器推力 + CFD阻力
tau_out = tau + tau_drag;

% 状态方程（tau_out 已包含推力和阻力）
xdot = [ Dnu_c + M \ (tau_out - C * nu_r - g);
    J * nu ];
end
