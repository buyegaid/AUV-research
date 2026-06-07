function [B_thr, pos] = xhy_thruster_geometry()
%XHY_THRUSTER_GEOMETRY XHY 推进器几何配置（统一来源，消除多处重复定义）
%
% 输出:
%   B_thr  - 6×5 推力分配矩阵，将推进器推力(N)映射为船体坐标系力/力矩(N, N·m)
%   pos    - 推进器位置参数结构体
%
% 推进器排列顺序（动力学顺序 dyn order）:
%   [T5主推, T1前垂推, T2后垂推, T3前侧推, T4后侧推]'
%
% 推进器排列顺序（固件文档顺序 doc order）:
%   [T1前垂推, T2后垂推, T3前侧推, T4后侧推, T5主推]'
%
% 参考:
%   [[CAN协议说明]] — 推进器控制 CAN 协议
%   [[推力分配]] — K 矩阵与推力分配矩阵

pos = struct();
pos.x_vert_f = +0.344;   % 前垂推 X 坐标 (m)
pos.x_vert_r = -0.293;   % 后垂推 X 坐标 (m)
pos.x_side_f = +0.424;   % 前侧推 X 坐标 (m)
pos.x_side_r = -0.376;   % 后侧推 X 坐标 (m)

% 推力分配矩阵 B_thr: tau = B_thr * T_vec
% T_vec = [T5_main, T1_vert_f, T2_vert_r, T3_side_f, T4_side_r]' (dyn order)
% tau   = [X, Y, Z, K, M, N]' (N, N·m)
%
% X: T5 主推贡献前进力
% Y: T3+T4 侧推贡献侧移力
% Z: T1+T2 垂推贡献垂荡力
% K: 横滚通道不可控（全零行，5推进器无法产生独立横滚力矩）
% M: T1(前)+T2(后) 垂推通过 x 力臂产生俯仰力矩
% N: T3(前)+T4(后) 侧推通过 x 力臂产生偏航力矩

B_thr = [
    1,  0,              0,              0,        0;
    0,  0,              0,              1,        1;
    0,  1,              1,              0,        0;
    0,  0,              0,              0,        0;
    0, -pos.x_vert_f,  -pos.x_vert_r,   0,        0;
    0,  0,              0,              pos.x_side_f, pos.x_side_r
    ];

end

function doc_order = dyn_to_doc_order()
%DYN_TO_DOC_ORDER 动力学顺序 → 固件文档顺序的索引映射
% doc_order(i) = dyn_order 中的索引，使得 pwm_doc = pwm_dyn(dyn_to_doc_order)
% 即 pwm_doc = [T1 T2 T3 T4 T5] = pwm_dyn([2 3 4 5 1])
    doc_order = [2; 3; 4; 5; 1];
end

function dyn_order = doc_to_dyn_order()
%DOC_TO_DYN_ORDER 固件文档顺序 → 动力学顺序的索引映射
% dyn_order(i) = doc_order 中的索引，使得 pwm_dyn = pwm_doc(doc_to_dyn_order)
% 即 pwm_dyn = [T5 T1 T2 T3 T4] = pwm_doc([5 1 2 3 4])
    dyn_order = [5; 1; 2; 3; 4];
end
