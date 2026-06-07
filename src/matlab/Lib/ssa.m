function angle = ssa(angle)
% SSA 将角度归一化到 [-pi, pi]
%   angle = ssa(angle)
%
%   Smallest Signed Angle - 将角度映射到 [-pi, pi] 区间

angle = mod(angle + pi, 2*pi) - pi;

end
