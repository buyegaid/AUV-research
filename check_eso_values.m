% 检查ESO估计值
clear; close all; clc;
addpath('.', './Lib', './guidance', './controller/xhy', './controller/remus', './model', './eso', './post', './traj');

load('results/comparison_smc_pieso.mat');

fprintf('=== ESO估计值分析 ===\n\n');

% SMC (无ESO)
fprintf('SMC (useESO=0):\n');
fprintf('  hat_d范围: [%.3f, %.3f] N\n', min(hist_smc.hat_d(:,1)), max(hist_smc.hat_d(:,1)));
fprintf('  hat_d均值: %.3f N\n', mean(hist_smc.hat_d(:,1)));

% LESO
fprintf('\nLESO (useESO=1, usePIESO=0):\n');
fprintf('  hat_d范围: [%.3f, %.3f] N\n', min(hist_leso.hat_d(:,1)), max(hist_leso.hat_d(:,1)));
fprintf('  hat_d均值: %.3f N\n', mean(hist_leso.hat_d(:,1)));
fprintf('  Z3范围: [%.3f, %.3f]\n', min(hist_leso.Z(:,13)), max(hist_leso.Z(:,13)));

% PI-ESO
fprintf('\nPI-ESO (useESO=1, usePIESO=1):\n');
fprintf('  hat_d范围: [%.3f, %.3f] N\n', min(hist_pieso.hat_d(:,1)), max(hist_pieso.hat_d(:,1)));
fprintf('  hat_d均值: %.3f N\n', mean(hist_pieso.hat_d(:,1)));
fprintf('  Z3范围: [%.3f, %.3f]\n', min(hist_pieso.Z(:,13)), max(hist_pieso.Z(:,13)));

fprintf('\n结论:\n');
if max(abs(hist_leso.hat_d(:))) < 0.01 && max(abs(hist_pieso.hat_d(:))) < 0.01
    fprintf('⚠️ ESO估计值接近零，说明ESO未正常工作\n');
else
    fprintf('✅ ESO产生了非零估计值\n');
end
