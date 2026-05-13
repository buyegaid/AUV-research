function pts = line_traj(start_xyz, yaw, ds, N)
% 生成直线轨迹点
%
% 输入:
% start_xyz = [x0; y0; z0]    起点
% yaw       = 航向角 (rad)   绕Z轴
% ds        = 点间距 (m)
% N         = 点的数量
%
% 输出:
% pts.pos.x, pts.pos.y, pts.pos.z

x0 = start_xyz(1);
y0 = start_xyz(2);
z0 = start_xyz(3);

% 方向向量（航向角）
dx = cos(yaw);
dy = sin(yaw);
dz = 0;         % 默认在水平面内走直线

% 距离序列
s = (0:N-1) * ds;

% 轨迹
x = x0 + dx * s;
y = y0 + dy * s;
z = z0 + dz * s;

% 输出结构体
pts.pos.x = x';
pts.pos.y = y';
pts.pos.z = z';
end
