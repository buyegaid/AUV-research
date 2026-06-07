function [x_filt, x_dot_filt] = LOSobserver(x_filt, x_dot_filt, x_ref, h, K_f)
% LOSOBSERVER 一阶低通滤波器用于LOS制导信号
%   [x_filt, x_dot_filt] = LOSobserver(x_filt, x_dot_filt, x_ref, h, K_f)
%
%   输入:
%     x_filt:     当前滤波后的状态
%     x_dot_filt: 当前滤波后的导数
%     x_ref:      参考输入
%     h:          时间步长
%     K_f:        滤波器增益 (越大响应越快)
%
%   输出:
%     x_filt:     更新后的滤波状态
%     x_dot_filt: 更新后的滤波导数
%
%   动态方程:
%     x_dot_filt = -K_f * (x_filt - x_ref)
%     x_filt = x_filt + h * x_dot_filt

% 计算导数
x_dot_filt = -K_f * (x_filt - x_ref);

% 欧拉积分
x_filt = x_filt + h * x_dot_filt;

end
