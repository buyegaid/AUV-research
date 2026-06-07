function env = createEnv(obsDim, actDim,Name,stepFcn,resetFcn)
% 创建强化学习环境

% ===== 状态维度 =====
obsInfo = rlNumericSpec([obsDim 1], ...
    'LowerLimit', -inf*ones(obsDim,1), ...
    'UpperLimit',  inf*ones(obsDim,1));
%  观测空间命名
obsInfo.Name = Name + " Observations";

% ===== 动作维度 =====
actInfo = rlNumericSpec([actDim 1], ...
    'LowerLimit', -1*ones(actDim,1), ...
    'UpperLimit',  1*ones(actDim,1));
actInfo.Name = Name + " Control Parameters";

% ===== 创建函数式环境 =====
env = rlFunctionEnv(obsInfo, actInfo, stepFcn, resetFcn);
end