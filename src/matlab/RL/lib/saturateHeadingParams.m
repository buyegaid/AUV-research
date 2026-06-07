function params = saturateHeadingParams(params)
% 参数约束函数
params.K_d(1)     = min(max(params.K_d(1), 0), 0.5); % PID 增益
params.K_sigma(1) = min(max(params.K_sigma(1), 0.001), 0.5); % SMC 增益
params.lambda(1)  = min(max(params.lambda(1), 0.01), 0.5); % 滑动面动力学常数
params.phi_b(1)   = min(max(params.phi_b(1), 0.01), 0.5); % 边界层厚度

end