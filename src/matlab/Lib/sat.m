function y = sat(x, limit)
% SAT 饱和函数
%   y = sat(x, limit)
%
%   将x限制在 [-limit, limit] 范围内
%   如果limit是2元素向量 [min, max]，则限制在 [min, max] 范围内

if nargin < 2
    error('sat requires 2 arguments: sat(x, limit)');
end

if length(limit) == 1
    % 对称限幅
    y = max(-limit, min(limit, x));
elseif length(limit) == 2
    % 非对称限幅 [min, max]
    y = max(limit(1), min(limit(2), x));
else
    error('limit must be scalar or 2-element vector');
end

end
