function pts = traj(ds, N)
% 生成圆形航迹点（以原点为圆心，半径 R）
% 输入:
%   ds - 相邻航迹点沿弧长间距 (m)
%   N  - 生成点的数量
% 输出:
%   pts - N×3 矩阵，每行为 [x, y, z]
%
% 轨迹为逆时针圆周，z 恒为 0

if nargin < 2
    error('需要两个输入：ds 和 N');
end
if ds <= 0 || N < 1
    error('ds 必须 > 0，N 必须 >= 1');
end

R = 300;           % 半径（m）
delta_theta = ds / R;        % 每个点之间的角距（rad）

theta = (0:(N-1))' * delta_theta;   % 列向量
x = R * cos(theta);
y = R * sin(theta);
z = zeros(N,1);

% pts = [x, y, z];
pts.pos.x = x;
pts.pos.y = y;
pts.pos.z = z;
end
