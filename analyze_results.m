% 分析对比实验结果
clear; close all; clc;
addpath('.', './Lib', './guidance', './controller/xhy', './controller/remus', './model', './eso', './post', './traj');

load('results/comparison_smc_pieso.mat');

fprintf('=== 结果诊断 ===\n\n');

% 检查轨迹
fprintf('轨迹信息:\n');
fprintf('  轨迹点数: %d\n', length(hist_smc.traj.pos.x));
fprintf('  轨迹半径: %.1f m\n', mean(sqrt(hist_smc.traj.pos.x.^2 + hist_smc.traj.pos.y.^2)));

% 检查最终位置
fprintf('\n最终位置 (300s):\n');
fprintf('  SMC:    (%.1f, %.1f, %.1f)\n', hist_smc.x(end,7), hist_smc.x(end,8), hist_smc.x(end,9));
fprintf('  LESO:   (%.1f, %.1f, %.1f)\n', hist_leso.x(end,7), hist_leso.x(end,8), hist_leso.x(end,9));
fprintf('  PI-ESO: (%.1f, %.1f, %.1f)\n', hist_pieso.x(end,7), hist_pieso.x(end,8), hist_pieso.x(end,9));

% 检查速度
fprintf('\n平均速度:\n');
fprintf('  SMC:    %.2f m/s\n', mean(hist_smc.x(:,1)));
fprintf('  LESO:   %.2f m/s\n', mean(hist_leso.x(:,1)));
fprintf('  PI-ESO: %.2f m/s\n', mean(hist_pieso.x(:,1)));

% 检查控制输入
fprintf('\n平均推力指令:\n');
fprintf('  SMC X:    %.1f N\n', mean(hist_smc.tau(:,1)));
fprintf('  LESO X:   %.1f N\n', mean(hist_leso.tau(:,1)));
fprintf('  PI-ESO X: %.1f N\n', mean(hist_pieso.tau(:,1)));

fprintf('\n平均偏航力矩指令:\n');
fprintf('  SMC N:    %.2f N·m\n', mean(hist_smc.tau(:,6)));
fprintf('  LESO N:   %.2f N·m\n', mean(hist_leso.tau(:,6)));
fprintf('  PI-ESO N: %.2f N·m\n', mean(hist_pieso.tau(:,6)));
