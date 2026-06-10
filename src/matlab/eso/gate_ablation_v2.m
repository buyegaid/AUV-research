%% Gate Ablation V2: UCCO-Gate vs UCCO-NoGate 公平消融
% 场景: straight, step (非circular — 那里gate无差异)
% 同一CFD prior, 同一K_obs, 同一GM propagation, 只关gate
% 2026-06-10

project_root = setup_paths();

scenarios = {'straight', 'step'};
mismatches = [0, 20, 30];
n_seeds = 5;  % 快速5seed验证，主表用10 seeds

fprintf('===== Gate Ablation: UCCO-Gate vs UCCO-NoGate =====\n');
fprintf('场景: straight, step | 失配: 0/20/30%% | seeds: %d\n\n', n_seeds);

results = struct();
params_file = fullfile(project_root, 'src', 'matlab', 'Lib', 'get_params.m');

for si = 1:2
    sc = scenarios{si};
    for mi = 1:3
        mm = mismatches(mi);
        key = sprintf('%s_%d', sc, mm);
        results.(key) = struct();

        for gate_mode = {'gate', 'nogate'}
            gm = gate_mode{1};
            is_gate = strcmp(gm, 'gate');
            gate_mu_val = 1e-8 * is_gate;  % gate: 1e-8, nogate: 0

            rmse_arr = zeros(1, n_seeds);

            for s = 1:n_seeds
                % 修改 gate_mu
                content = fileread(params_file);
                lines = splitlines(content);
                for li = 1:length(lines)
                    if contains(lines{li}, 'params.ucco.gate_mu')
                        idx = strfind(lines{li}, '%');
                        comment_part = '';
                        if ~isempty(idx), comment_part = lines{li}(idx(1):end); end
                        lines{li} = sprintf('params.ucco.gate_mu = %g;%s', gate_mu_val, comment_part);
                        break;
                    end
                end
                content = strjoin(lines, newline);
                fid = fopen(params_file, 'w'); fprintf(fid, '%s', content); fclose(fid);
                clear get_params;

                cfg = struct('scenario', sc, 'mismatch_pct', mm, ...
                             'observers', {{'UCCO'}}, ...
                             'noise_level', 'low', ...
                             'seed', s, 'T_end', 100, 'verbose', false);
                r = run_observer_comparison(cfg);
                rmse_arr(s) = r.UCCO.RMSE;
            end

            rmse_mean = mean(rmse_arr);
            rmse_std  = std(rmse_arr);
            results.(key).(gm) = struct('mean', rmse_mean, 'std', rmse_std, 'vals', rmse_arr);

            fprintf('%-8s 失配=%d%%  %-6s: %.4f ± %.4f\n', sc, mm, gm, rmse_mean, rmse_std);
        end

        % 计算门控收益
        g_mean = results.(key).gate.mean;
        ng_mean = results.(key).nogate.mean;
        benefit = (ng_mean - g_mean) / ng_mean * 100;
        fprintf('  → 门控收益: %+.1f%%\n\n', benefit);
    end
end

% 恢复 gate_mu
content = fileread(params_file);
lines = splitlines(content);
for li = 1:length(lines)
    if contains(lines{li}, 'params.ucco.gate_mu')
        idx = strfind(lines{li}, '%');
        comment_part = '';
        if ~isempty(idx), comment_part = lines{li}(idx(1):end); end
        lines{li} = sprintf('params.ucco.gate_mu = 1e-8;%s', comment_part);
        break;
    end
end
content = strjoin(lines, newline);
fid = fopen(params_file, 'w'); fprintf(fid, '%s', content); fclose(fid);

fprintf('\n===== Gate Ablation 汇总 =====\n');
fprintf('%-10s %-6s %-10s %-10s %-10s\n', '场景', '失配', 'Gate', 'NoGate', '收益');
for si = 1:2
    for mi = 1:3
        key = sprintf('%s_%d', scenarios{si}, mismatches(mi));
        fprintf('%-10s %-6d %-10.4f %-10.4f %+.1f%%\n', ...
            scenarios{si}, mismatches(mi), ...
            results.(key).gate.mean, results.(key).nogate.mean, ...
            (results.(key).nogate.mean - results.(key).gate.mean) / results.(key).nogate.mean * 100);
    end
end

save('results/gate_ablation_v2.mat', 'results');
fprintf('\n数据已保存。\n');
