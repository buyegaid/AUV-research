%% R009: 门控消融 — 简化版
% 直接测3个gate_mu值 × 3噪声 × 2seeds = 18次
% 2026-06-10

project_root = setup_paths();

gate_mu_vals = [1e-8, 0, 0.01];
gate_names   = {'Gramian', 'NoGate', 'HighThr'};
noise_names  = {'none', 'low', 'high'};

fprintf('===== R009: 门控消融 =====\n');
fprintf('%-12s %-8s %-10s\n', 'Gate', 'Noise', 'RMSE');

params_file = fullfile(project_root, 'src', 'matlab', 'Lib', 'get_params.m');

for gi = 1:3
    for ni = 1:3
        rmse_sum = 0;
        for s = 1:2
            % 修改 gate_mu — 使用strrep直接替换数字
            content = fileread(params_file);

            % 找到 gate_mu 行并替换
            lines = splitlines(content);
            for li = 1:length(lines)
                if contains(lines{li}, 'params.ucco.gate_mu')
                    % 提取注释部分
                    comment_part = '';
                    if contains(lines{li}, '%')
                        idx = strfind(lines{li}, '%');
                        comment_part = lines{li}(idx(1):end);
                    end
                    lines{li} = sprintf('params.ucco.gate_mu = %g;%s', gate_mu_vals(gi), comment_part);
                    break;
                end
            end
            content = strjoin(lines, newline);
            fid = fopen(params_file, 'w'); fprintf(fid, '%s', content); fclose(fid);
            clear get_params;

            cfg = struct('scenario', 'circle', 'mismatch_pct', 0, ...
                         'observers', {{'UCCO'}}, ...
                         'noise_level', noise_names{ni}, ...
                         'seed', s, 'T_end', 100, 'verbose', false);
            r = run_observer_comparison(cfg);
            rmse_sum = rmse_sum + r.UCCO.RMSE;
        end
        rmse_mean = rmse_sum / 2;
        fprintf('%-12s %-8s %-10.4f\n', gate_names{gi}, noise_names{ni}, rmse_mean);
    end
end

% 恢复
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

fprintf('\n门控消融完成。\n');
