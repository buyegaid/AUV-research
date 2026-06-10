%% R010: Monte Carlo 噪声鲁棒性验证
% 2场景(圆形/直线) × 2失配(0%/20%) × 4观测器 × 10 seeds
% DVL σ=0.02m/s, heading σ=0.5°
% 2026-06-10

project_root = setup_paths();
params = get_params();

scenarios = {'circle', 'straight'};
mismatches = [0, 20];
n_seeds = 10;

fprintf('===== R010: Monte Carlo (10 seeds, low noise) =====\n');
fprintf('场景 × 失配 × 观测器 × %d seeds = %d runs\n', n_seeds, ...
    2*2*4*n_seeds);

results = struct();

for si = 1:2
    sc = scenarios{si};
    for mi = 1:2
        mm = mismatches(mi);
        key = sprintf('%s_%d', sc, mm);
        results.(key) = struct();
        for obs = {'KIN','BORHAUG','EKF','UCCO'}
            obs_name = obs{1};
            rmse_arr = zeros(1, n_seeds);

            for s = 1:n_seeds
                cfg = struct('scenario', sc, 'mismatch_pct', mm, ...
                             'observers', {{obs_name}}, ...
                             'noise_level', 'low', ...
                             'seed', s, 'T_end', 100, 'verbose', false);
                r = run_observer_comparison(cfg);
                rmse_arr(s) = r.(obs_name).RMSE;
            end

            results.(key).(obs_name) = rmse_arr;
            fprintf('%-8s 失配=%d%% %-8s: %.4f ± %.4f\n', ...
                sc, mm, obs_name, mean(rmse_arr), std(rmse_arr));
        end
    end
end

save('results/monte_carlo.mat', 'results');

fprintf('\n===== MC汇总 =====\n');
fprintf('%-10s %-6s %-8s %-8s %-8s %-8s\n', '场景', '失配', 'KIN', 'BORHAUG', 'EKF', 'UCCO');
for si = 1:2
    for mi = 1:2
        key = sprintf('%s_%d', scenarios{si}, mismatches(mi));
        fprintf('%-10s %-6d', scenarios{si}, mismatches(mi));
        for obs = {'KIN','BORHAUG','EKF','UCCO'}
            v = results.(key).(obs{1});
            fprintf(' %-8.3f', mean(v));
        end
        fprintf('\n');
    end
end
fprintf('\nMonte Carlo完成。\n');
