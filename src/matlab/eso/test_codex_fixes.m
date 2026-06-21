%% Codex推荐三方案验证: 修复GM+Clamp
% A: tau_c=Inf + max_dc=0.1 (无GM衰减)
% B: tau_c=Inf + max_dc=0.3 (无GM+放宽Clamp)
% C: 自适应GM(lambda<1e-5则alpha=1) + max_dc=0.3
% vs PC-RCO(full), NoGM, NoClamp baselines
% 场景: circle+straight, 0%/30%失配, 5 seeds, 低噪声
% 2026-06-11

project_root = setup_paths();
scenarios = {'circle','straight'};
mismatches = [0, 30];
n_seeds = 5;

fprintf('===== Codex修复方案验证 =====\n\n');

all_results = struct();

for si = 1:2
    sc = scenarios{si};
    for mi = 1:2
        mm = mismatches(mi);

        for fix_name = {'PCRCO','NoGM','NoClamp','FixA_NoGM','FixB_NoGM_LooseClamp','FixC_AdaptGM_LooseClamp'}
            fn = fix_name{1};

            % 选择配置
            switch fn
                case 'PCRCO'
                    tau_c = 100; max_dc = 0.1; adapt_gm = false;
                case 'NoGM'
                    tau_c = 1e9; max_dc = 0.1; adapt_gm = false;
                case 'NoClamp'
                    tau_c = 100; max_dc = 10; adapt_gm = false;
                case 'FixA_NoGM'
                    tau_c = 1e9; max_dc = 0.1; adapt_gm = false;
                case 'FixB_NoGM_LooseClamp'
                    tau_c = 1e9; max_dc = 0.3; adapt_gm = false;
                case 'FixC_AdaptGM_LooseClamp'
                    tau_c = 3000; max_dc = 0.3; adapt_gm = true;
            end

            rmse_arr = zeros(1, n_seeds);
            for s = 1:n_seeds
                cfg = struct('scenario', sc, 'mismatch_pct', mm, ...
                             'noise_level', 'low', 'seed', s, 'T_end', 100, 'verbose', false);
                r = run_codex_fix(cfg, tau_c, max_dc, adapt_gm);
                rmse_arr(s) = r.rmse;
            end
            rmse_mean = mean(rmse_arr);
            rmse_std = std(rmse_arr);
            all_results.(sprintf('%s_%d',sc,mm)).(fn) = struct('mean',rmse_mean,'std',rmse_std);

            fprintf('%-8s 失配=%d%% %-22s %.4f ± %.4f\n', sc, mm, fn, rmse_mean, rmse_std);
        end
        fprintf('\n');
    end
end

% 汇总
fprintf('\n===== 汇总表 =====\n');
fprintf('%-10s %-6s %-8s %-8s %-8s %-8s %-8s %-8s\n', '场景','失配','PCRCO','NoGM','NoClamp','FixA','FixB','FixC');
for si=1:2
    for mi=1:2
        key = sprintf('%s_%d',scenarios{si},mismatches(mi));
        fprintf('%-10s %-6d', scenarios{si}, mismatches(mi));
        for fn = {'PCRCO','NoGM','NoClamp','FixA_NoGM','FixB_NoGM_LooseClamp','FixC_AdaptGM_LooseClamp'}
            fprintf(' %-8.3f', all_results.(key).(fn{1}).mean);
        end
        fprintf('\n');
    end
end

save('results/codex_fixes.mat','all_results');
fprintf('\n数据已保存\n');
