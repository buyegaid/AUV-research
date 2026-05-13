function p = param2ismc(a, pmin, pmax)
% 将动作 a ∈ [-1,1] 映射到参数区间 [pmin, pmax]

a = max(min(a, 1), -1);   % 限幅
p = pmin + (a + 1) / 2 * (pmax - pmin);

end
