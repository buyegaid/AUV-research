%% omega0灵敏度分析：验证PI-ESO在适当带宽下的优势
% 当omega0与1/tau_c同量级时，Gauss-Markov先验才能发挥作用
clear; close all; clc;
addpath('.', './Lib', './guidance', './controller/xhy', './model', './eso');

fprintf('=== omega0灵敏度分析 (GM tau_c=50s, 定点控制) ===\n');
fprintf('%-12s | %-10s | %-10s | %-12s\n', 'omega0', 'LESO(m)', 'PIESO(m)', 'PIESO改善%');
fprintf('%s\n', repmat('-',1,55));

omega0_arr = [0.3, 0.5, 1.0, 2.0, 3.0];
leso_errs  = zeros(size(omega0_arr));
pieso_errs = zeros(size(omega0_arr));

for oi = 1:length(omega0_arr)
    leso_errs(oi)  = run_sk(omega0_arr(oi), 50, false, 50);
    pieso_errs(oi) = run_sk(omega0_arr(oi), 50, true,  50);
    imp = (leso_errs(oi) - pieso_errs(oi)) / leso_errs(oi) * 100;
    fprintf('%-12.2f | %-10.3f | %-10.3f | %-12.1f\n', ...
        omega0_arr(oi), leso_errs(oi), pieso_errs(oi), imp);
end
