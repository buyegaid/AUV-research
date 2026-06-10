%% 单配置测试: 从results/tmp_p.mat读取参数
% 2026-06-10
project_root = setup_paths();
load('results/tmp_p.mat', 'params');

cfg0 = struct('scenario', 'circle', 'mismatch_pct', 0, ...
             'observers', {{'UCCO'}}, ...
             'noise_level', 'none', 'seed', 1, 'T_end', 100, 'verbose', false);
cfg30 = struct('scenario', 'circle', 'mismatch_pct', 30, ...
              'observers', {{'UCCO'}}, ...
              'noise_level', 'none', 'seed', 1, 'T_end', 100, 'verbose', false);

% 用修改后的 params.ucco
r0 = run_observer_comparison(cfg0);
r30 = run_observer_comparison(cfg30);

rmse0 = r0.UCCO.RMSE;
rmse30 = r30.UCCO.RMSE;
degrade = (rmse30 - rmse0) / rmse0 * 100;
fprintf('K_obs=%.0f: 0%%=%.4f  30%%=%.4f  退化=%+.1f%%\n', ...
    params.ucco.K_obs, rmse0, rmse30, degrade);
