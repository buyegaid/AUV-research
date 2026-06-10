%% 逐机制消融: 关预测校正 / 关GM衰减 / 关限幅
% 对比 PC-RCO(full) vs 三个消融变体 vs KIN vs CFD-Luenberger vs EKF
% 场景: 圆形+直线, 失配: 0%/30%, 5 seeds, 低噪声
% 2026-06-11

project_root = setup_paths();
scenarios = {'circle', 'straight'};
mismatches = [0, 30];
n_seeds = 5;

fprintf('===== 逐机制消融实验 =====\n');
fprintf('场景: %s | 失配: 0/30%% | seeds: %d\n\n', strjoin(scenarios, ', '), n_seeds);

% 6个变体: KIN, CFD-Luenberger, EKF, PC-RCO, NoPredCorr, NoGM, NoClamp
variants = {'KIN','CFDLuenberger','EKF','PCRCO','NoPredCorr','NoGM','NoClamp'};
results = struct();

for si = 1:length(scenarios)
    sc = scenarios{si};
    for mi = 1:length(mismatches)
        mm = mismatches(mi);
        key = sprintf('%s_%d', sc, mm);
        results.(key) = struct();

        for vi = 1:length(variants)
            vn = variants{vi};
            rmse_arr = zeros(1, n_seeds);

            for s = 1:n_seeds
                cfg = struct('scenario', sc, 'mismatch_pct', mm, ...
                             'observers', {{vn}}, ...
                             'noise_level', 'low', ...
                             'seed', s, 'T_end', 100, 'verbose', false);
                r = run_ablation_variant(cfg);
                rmse_arr(s) = r.UCCO.RMSE;
            end

            rmse_mean = mean(rmse_arr);
            rmse_std  = std(rmse_arr);
            results.(key).(vn) = struct('mean', rmse_mean, 'std', rmse_std);

            fprintf('%-8s 失配=%d%%  %-12s  %.4f ± %.4f\n', sc, mm, vn, rmse_mean, rmse_std);
        end
        fprintf('\n');
    end
end

% 汇总表
fprintf('\n===== 消融汇总 =====\n');
fprintf('%-10s %-6s %-12s %-12s %-12s %-12s\n', '场景', '失配', 'PCRCO', 'NoPredCorr', 'NoGM', 'NoClamp');
for si = 1:2
    for mi = 1:2
        key = sprintf('%s_%d', scenarios{si}, mismatches(mi));
        fprintf('%-10s %-6d %-12.4f %-12.4f %-12.4f %-12.4f\n', ...
            scenarios{si}, mismatches(mi), ...
            results.(key).('PCRCO').mean, ...
            results.(key).NoPredCorr.mean, ...
            results.(key).NoGM.mean, ...
            results.(key).NoClamp.mean);
    end
end

save('results/per_mechanism_ablation.mat', 'results');
fprintf('\n数据已保存到 results/per_mechanism_ablation.mat\n');
