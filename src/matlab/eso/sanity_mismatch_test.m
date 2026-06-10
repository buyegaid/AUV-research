%% Mismatch Sanity: 0% vs 30% 对比
% 验证模型失配机制和Børhaug增益
% 2026-06-10

project_root = setup_paths();

fprintf('=== 0%% 失配 ===\n');
cfg0 = struct('scenario', 'circle', 'mismatch_pct', 0, ...
             'observers', {{'KIN','BORHAUG','EKF','UCCO'}}, ...
             'noise_level', 'none', 'seed', 1, 'T_end', 100, 'verbose', false);
r0 = run_observer_comparison(cfg0);

fprintf('\n=== 30%% 失配 ===\n');
cfg30 = struct('scenario', 'circle', 'mismatch_pct', 30, ...
              'observers', {{'KIN','BORHAUG','EKF','UCCO'}}, ...
              'noise_level', 'none', 'seed', 1, 'T_end', 100, 'verbose', false);
r30 = run_observer_comparison(cfg30);

fprintf('\n===== 失配对比 =====\n');
fprintf('%-10s %-10s %-10s %-10s\n', '方法', '0% RMSE', '30% RMSE', '退化');
for m = {'KIN','BORHAUG','EKF','UCCO'}
    name = m{1};
    v0 = r0.(name).RMSE;
    v30 = r30.(name).RMSE;
    degrade = (v30 - v0) / v0 * 100;
    fprintf('%-10s %-10.4f %-10.4f %+.1f%%\n', name, v0, v30, degrade);
end
fprintf('\nDone.\n');
