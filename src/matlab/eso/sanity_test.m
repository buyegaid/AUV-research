%% Sanity Check: 四观测器最小验证（20s）
% 2026-06-10

project_root = setup_paths();
cfg = struct('scenario', 'circle', 'mismatch_pct', 0, ...
             'observers', {{'KIN','BORHAUG','EKF','UCCO'}}, ...
             'noise_level', 'none', 'seed', 1, 'T_end', 20, 'verbose', true);
result = run_observer_comparison(cfg);
fprintf('\nSanity test PASSED.\n');
