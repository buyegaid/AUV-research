function [n_rpm, info] = thrust_allocation_xhy(tau_cmd, params)
% THRUST_ALLOCATION_XHY 将6-DOF力/力矩指令分配到5个推进器
%
%   n_rpm = thrust_allocation_xhy(tau_cmd, params)
%
%   输入:
%     tau_cmd: 6x1 期望力/力矩向量 [X Y Z K M N]' (N, N·m)
%     params:  参数结构体，包含:
%       .rho            水密度 (kg/m^3)
%       .D_prop_main    主推直径 (m)
%       .D_prop_aux     辅推直径 (m)
%       .KT_main_fwd    主推正向推力系数（正转）
%       .KT_main_rev    主推反向推力系数（反转）
%       .KT_aux_fwd     辅推正向推力系数（正转）
%       .KT_aux_rev     辅推反向推力系数（反转）
%       .n_max          最大转速 (RPM)
%       .x_vert_f       前垂直推进器x位置 (m)
%       .x_vert_r       后垂直推进器x位置 (m)
%       .x_side_f       前侧向推进器x位置 (m)
%       .x_side_r       后侧向推进器x位置 (m)
%
%   输出:
%     n_rpm: 5x1 推进器转速 [T1 T2 T3 T4 T5]' (RPM)
%     info:  诊断信息结构体
%
%   推力分配矩阵 (来自xhy.m):
%     B_thr = [0   0           0           0         1      ;  % X (surge)
%              0   0           1           1         0      ;  % Y (sway)
%              1   1           0           0         0      ;  % Z (heave)
%              0   0           0           0         0      ;  % K (roll, 不可控)
%             -x_vert_f -x_vert_r 0        0         0      ;  % M (pitch)
%              0   0           x_side_f    x_side_r  0];       % N (yaw)
%
%   推力模型: T = rho * D^4 * KT * |n| * n  (n单位: rps)
%   主推和辅推使用不同的D和KT，正反转KT独立

% 默认参数（传统 RPM/KT 兼容路径）
if nargin < 2
    params.rho = 1026;
    params.D_prop_main = 0.10;
    params.D_prop_aux  = 0.06;
    params.KT_main_fwd = 0.0293;   % CFD阻力校准 (2026-06-03)
    params.KT_main_rev = 0.0201;   % 同比缩放
    params.KT_aux_fwd  = 0.327;    % CFD阻力校准 (2026-06-03)
    params.KT_aux_rev  = 0.327;    % 无反向数据, 暂取与正向相同
    params.n_max = 2500;
    params.x_vert_f = +0.344;
    params.x_vert_r = -0.293;
    params.x_side_f = +0.424;
    params.x_side_r = -0.376;
end

% 向后兼容旧版thr_params（仅有KT/D_prop单一值）
if ~isfield(params, 'KT_main_fwd') && isfield(params, 'KT')
    params.KT_main_fwd = params.KT;
    params.KT_main_rev = params.KT;
    params.KT_aux_fwd  = params.KT;
    params.KT_aux_rev  = params.KT;
end
if ~isfield(params, 'D_prop_main') && isfield(params, 'D_prop')
    params.D_prop_main = params.D_prop;
    params.D_prop_aux  = params.D_prop;
end

rho = params.rho;
n_max = params.n_max;

% 每个推进器的直径和推力系数（区分主推/辅推，正转/反转）
D_arr  = [params.D_prop_aux; params.D_prop_aux; params.D_prop_aux; params.D_prop_aux; params.D_prop_main];
KT_fwd_arr = [params.KT_aux_fwd; params.KT_aux_fwd; params.KT_aux_fwd; params.KT_aux_fwd; params.KT_main_fwd];
KT_rev_arr = [params.KT_aux_rev; params.KT_aux_rev; params.KT_aux_rev; params.KT_aux_rev; params.KT_main_rev];

% 推力分配矩阵（统一来源，消除重复定义）
[B_thr, pos] = xhy_thruster_geometry();

% 横滚通道不可控，移除第4行
B_actuated = B_thr([1 2 3 5 6], :);  % 5x5矩阵
tau_actuated = tau_cmd([1 2 3 5 6]); % 5x1向量

% 伪逆求解推力向量
T_vec = pinv(B_actuated) * tau_actuated;

% 推力转RPM: T = rho * D^4 * KT * |n| * n
% 求解: n = sign(T) * sqrt(|T| / (rho * D^4 * KT))
% 每个推进器使用独立的D和KT（区分主推/辅推，正转/反转）
n_rps = zeros(5,1);
for i = 1:5
    if abs(T_vec(i)) < 1e-6
        n_rps(i) = 0;
    else
        % 根据推力方向选择正/反转KT
        if T_vec(i) >= 0
            KT = KT_fwd_arr(i);
        else
            KT = KT_rev_arr(i);
        end
        coeff = rho * D_arr(i)^4 * KT;
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
