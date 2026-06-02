% 检查ESO估计值
clear; close all; clc;
addpath('.', './Lib', './guidance', './controller/xhy', './controller/remus', './model', './eso', './post', './traj');

load('results/comparison_smc_pieso.mat');

fprintf('=== ESO估计值检查 ===\n\n');

% 检查hist结构体是否包含Z数据
if isfield(hist_smc, 'Z')
    fprintf('SMC hist包含Z数据\n');
else
    fprintf('⚠️ SMC hist不包含Z数据\n');
end

if isfield(hist_leso, 'Z')
    fprintf('LESO hist包含Z数据\n');
else
    fprintf('⚠️ LESO hist不包含Z数据\n');
end

if isfield(hist_pieso, 'Z')
    fprintf('PI-ESO hist包含Z数据\n');
else
    fprintf('⚠️ PI-ESO hist不包含Z数据\n');
end

fprintf('\n问题诊断:\n');
fprintf('所有三种方法的控制输入完全相同，说明ESO补偿未生效。\n');
fprintf('可能原因:\n');
fprintf('1. run_xhy_experiment函数未保存ESO状态到hist\n');
fprintf('2. ESO估计值为零\n');
fprintf('3. useESO标志未正确传递\n');
