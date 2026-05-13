function [xdot,U,M,C,D,g,tau]=xhy(x,ui,Vc,betaVc,w_c)
% XHY 5-thruster AUV dynamics
%   x   = [u v w p q r x y z phi theta psi]'  (12 states)
%   ui  = [n_main n_vert1 n_vert2 n_side1 n_side2]'  (RPM)
%   Vc, betaVc, w_c are optional ocean current inputs
%   Outputs are the state derivative xdot, speed U, and model matrices/forces.

if nargin < 2
    x = zeros(12,1);
    ui = zeros(5,1);
end

if nargin < 3
    Vc = 0;
end

if nargin < 4
    betaVc = 0;
end

if nargin < 5
    w_c = 0;
end


if (length(ui) ~= 5)
    error('ui-vector must have dimension 5!');
end

if (length(x) ~= 12)
    error('x-vector must have dimension 12');
end

% Constants
mu = 63.446827;         % 纬度
g_mu = gravity(mu);     % 重力加速度 (m/s^2)
rho = 1026;             % 水的密度 (kg/m^3)

nu = x(1:6);
psi = x(12); % 航向角

% 海流速度在船体坐标系中的分量
u_c_x = Vc * cos(betaVc - psi);
u_c_y = Vc * sin(betaVc - psi);
nu_c = [u_c_x; u_c_y; w_c; 0; 0; 0];
Dnu_c = [nu(6) * u_c_y; -nu(6) * u_c_x; 0; 0; 0; 0];

% 相对速度
nu_r = nu - nu_c;
U_r = norm(nu_r(1:3));
U   = norm(nu(1:3));

% AUV parameters (spheroid approximation)
L_auv = 1.242;               % AUV length (m)
D_auv = 0.24;                % AUV diameter (m)
H_auv = 0.42;                % AUV height (m)
S = 93223.78e-6;                % Planform area(mm^2)

r_bG = [0 0 0]';             % 重心位置(m)
r_bB = [0 0 -0.03]';         % 浮心位置(m)

% 刚体惯性矩阵
m=33;                      % 质量(kg)
Ix=0.540804;               % 绕x轴转动惯量(kg*m^2)
Iy=2.107488;               % 绕y轴转动惯量(kg*m^2)
Iz=1.849137;               % 绕z轴转动惯量(kg*m^2)
Ig = diag([Ix Iy Iz]);
MRB=diag([ m m m Ix Iy Iz ]);
MA=diag([ 11.2773 132.1086 47.104 0.006 0.043 0.138 ]);
nu_2 = nu(4:6);
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
B = W*1.01; % 轻微正浮力

% 阻力矩阵
% Low-speed linear damping matrix parameters 低速线性阻尼矩阵参数
T1 = 20;                 % Time constant in surge (s)
T2 = 20;                 % Time constant in sway (s)
zeta4 = 0.3;             % Relative damping ratio in roll
zeta5 = 0.8;             % Relative damping ratio in pitch
T6 = 1;                  % Time constant in yaw (s)
D = Dmtrx([T1 T2 T6],[zeta4 zeta5],MRB,MA,[W r_bG' r_bB']);
D(1,1) = D(1,1) * exp(-3 * U_r);
D(2,2) = D(2,2) * exp(-3 * U_r);

% 计算阻力
Cd = 0.42;
CD_0 = Cd * pi * D_auv^2 / S;
alpha = atan2(nu_r(3), nu_r(1));
tau_liftdrag = forceLiftDrag(D_auv, S, CD_0, alpha, U_r);
tau_crossflow = crossFlowDrag(L_auv, D_auv, D_auv, nu_r, 'cylinder');

% 运动学和重力/浮力矩阵
[J,R] = eulerang(x(10), x(11), x(12));
g = gRvect(W, B, R, r_bG, r_bB);

% Thruster mapping (5 thrusters)
% ui = [n_main, n_vert1, n_vert2, n_side1, n_side2]' in RPM
n_max = 2500;            % Nominal maximum thruster speed (RPM)
n_rpm = sat(ui, n_max);  % RPM with saturation

% 推力计算：主推直径10cm，辅助推直径7cm
T_main  = thrust_main(n_rpm(1), rho);
T_vert1 = thrust_aux(n_rpm(2), rho);
T_vert2 = thrust_aux(n_rpm(3), rho);
T_side1 = thrust_aux(n_rpm(4), rho);
T_side2 = thrust_aux(n_rpm(5), rho);

% Thruster geometry (modifiable)
x_vert_f = +0.344;  % fore vertical thruster x-position (m)
x_vert_r = -0.293;  % aft vertical thruster x-position (m)
x_side_f   = +0.424;  % longitudinal offset for side thrusters (m)
x_side_r   = -0.376;  % longitudinal offset for side thrusters (m)

% Force and moment allocation
% Positive Z is downwards, positive Y is starboard/right, positive N is nose-right
B_thr = [
    1,  0,           0,           0,           0;
    0,  0,           0,           1,           1;
    0,  1,           1,           0,           0;
    0,  0,           0,           0,           0;
    0, -x_vert_f,   -x_vert_r,    0,           0;
    0,  0,           0,           x_side_f,    x_side_r
    ];

T_vec = [T_main; T_vert1; T_vert2; T_side1; T_side2];

tau = B_thr * T_vec;



% 总输入力矩 = 主推进力矩 + 升阻力矩 + 横流阻力矩
tau_out = tau + tau_liftdrag + tau_crossflow;

% 状态方程
xdot = [ Dnu_c + M \ (tau_out - C * nu_r - D * nu_r - g);
    J * nu ];
end