function [n_rpm, info] = thrust_allocation_xhy(tau_cmd, params)
% THRUST_ALLOCATION_XHY 将6-DOF力/力矩指令分配到5个推进器
%
%   n_rpm = thrust_allocation_xhy(tau_cmd, params)
%
%   输入:
%     tau_cmd: 6x1 期望力/力矩向量 [X Y Z K M N]' (N, N·m)
%     params:  参数结构体，包含:
%       .rho         水密度 (kg/m^3)
%       .D_prop      推进器直径 (m)
%       .KT          推力系数
%       .n_max       最大转速 (RPM)
%       .x_vert_f    前垂直推进器x位置 (m)
%       .x_vert_r    后垂直推进器x位置 (m)
%       .x_side_f    前侧向推进器x位置 (m)
%       .x_side_r    后侧向推进器x位置 (m)
%
%   输出:
%     n_rpm: 5x1 推进器转速 [n_main n_vert1 n_vert2 n_side1 n_side2]' (RPM)
%     info:  诊断信息结构体
%
%   推力分配矩阵 (来自xhy.m):
%     B_thr = [1   0           0           0         0      ;  % X (surge)
%              0   0           0           1         1      ;  % Y (sway)
%              0   1           1           0         0      ;  % Z (heave)
%              0   0           0           0         0      ;  % K (roll, 不可控)
%              0  -x_vert_f   -x_vert_r    0         0      ;  % M (pitch)
%              0   0           0           x_side_f  x_side_r]; % N (yaw)
%
%   推力模型: T = rho * D_prop^4 * KT * |n| * n  (n单位: rps)

% 默认参数
if nargin < 2
    params.rho = 1026;
    params.D_prop = 0.10;
    params.KT = 0.22;
    params.n_max = 2500;
    params.x_vert_f = +0.344;
    params.x_vert_r = -0.293;
    params.x_side_f = +0.424;
    params.x_side_r = -0.376;
end

rho = params.rho;
D_prop = params.D_prop;
KT = params.KT;
n_max = params.n_max;

% 推力分配矩阵
B_thr = [
    1,  0,                  0,                  0,                0;
    0,  0,                  0,                  1,                1;
    0,  1,                  1,                  0,                0;
    0,  0,                  0,                  0,                0;
    0, -params.x_vert_f,   -params.x_vert_r,    0,                0;
    0,  0,                  0,                  params.x_side_f,  params.x_side_r
    ];

% 横滚通道不可控，移除第4行
B_actuated = B_thr([1 2 3 5 6], :);  % 5x5矩阵
tau_actuated = tau_cmd([1 2 3 5 6]); % 5x1向量

% 伪逆求解推力向量
T_vec = pinv(B_actuated) * tau_actuated;

% 推力转RPM: T = rho * D_prop^4 * KT * |n| * n
% 求解: n = sign(T) * sqrt(|T| / (rho * D_prop^4 * KT))
coeff = rho * D_prop^4 * KT;
n_rps = zeros(5,1);
for i = 1:5
    if abs(T_vec(i)) < 1e-6
        n_rps(i) = 0;
    else
        n_rps(i) = sign(T_vec(i)) * sqrt(abs(T_vec(i)) / coeff);
    end
end

% 转换为RPM并限幅
n_rpm = n_rps * 60;
n_rpm = max(-n_max, min(n_max, n_rpm));

% 诊断信息
info.T_vec = T_vec;
info.tau_actuated = tau_actuated;
info.tau_achieved = B_actuated * T_vec;
info.tau_error = tau_actuated - info.tau_achieved;
info.n_rps = n_rps;

end
