function x_next = rk4(f, h, x, varargin)
% RK4 四阶龙格-库塔积分器
%   x_next = rk4(f, h, x, u1, u2, ...)
%
%   输入:
%     f:       函数句柄 @(x, u1, u2, ...) 返回 xdot
%     h:       时间步长
%     x:       当前状态
%     varargin: 额外参数传递给f
%
%   输出:
%     x_next:  下一时刻状态

k1 = f(x, varargin{:});
k2 = f(x + 0.5*h*k1, varargin{:});
k3 = f(x + 0.5*h*k2, varargin{:});
k4 = f(x + h*k3, varargin{:});

x_next = x + (h/6) * (k1 + 2*k2 + 2*k3 + k4);

end
