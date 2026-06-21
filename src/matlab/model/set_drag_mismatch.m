function set_drag_mismatch(pct, seed)
% SET_DRAG_MISMATCH 设置CFD阻力系数失配百分比
% pct: 0-100, 每个系数独立在±pct范围内随机扰动
% seed: 随机种子(可选)

persistent mismatch_val
if nargin == 0
    mismatch_val = 0;
else
    if nargin < 2, seed = randi(1e6); end
    rng(seed);
    % 存储 [pct, seed] 供 xhy_drag_cfd 读取
    mismatch_val = [pct; seed];
end
assignin('base', '_ucco_mismatch', mismatch_val);
end
