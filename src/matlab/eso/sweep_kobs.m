%% UCCO K_obs 参数扫描
% 直接修改 get_params.m 文件并运行测试
% 2026-06-10

project_root = setup_paths();
K_vals = [4, 6, 8, 10, 12, 16];
fprintf('===== UCCO K_obs 扫描 (圆形, 100s, 1 seed) =====\n');
fprintf('%-8s %-10s %-10s %-10s\n', 'K_obs', 'RMSE 0%', 'RMSE 30%', '退化');

params_file = fullfile(project_root, 'src', 'matlab', 'Lib', 'get_params.m');

for ki = 1:length(K_vals)
    kv = K_vals(ki);

    % 读取并修改 get_params.m
    content = fileread(params_file);
    % 替换 K_obs
    content = regexprep(content, 'params\.ucco\.K_obs\s*=\s*[\d.]+', ...
                        sprintf('params.ucco.K_obs = %.1f', kv));
    % 替换 max_dc
    content = regexprep(content, 'params\.ucco\.max_dc\s*=\s*[\d.]+', ...
                        sprintf('params.ucco.max_dc = %.2f', kv/100));

    fid = fopen(params_file, 'w');
    fprintf(fid, '%s', content);
    fclose(fid);

    % 清除缓存
    clear get_params;

    % 重新获取参数
    params = get_params();

    % 运行测试
    cfg0 = struct('scenario', 'circle', 'mismatch_pct', 0, ...
                 'observers', {{'UCCO'}}, ...
                 'noise_level', 'none', 'seed', 1, 'T_end', 100, 'verbose', false);
    cfg30 = cfg0; cfg30.mismatch_pct = 30;

    r0 = run_observer_comparison(cfg0);
    r30 = run_observer_comparison(cfg30);

    rmse0 = r0.UCCO.RMSE;
    rmse30 = r30.UCCO.RMSE;
    degrade = (rmse30 - rmse0) / rmse0 * 100;
    fprintf('%-8.0f %-10.4f %-10.4f %+9.1f%%\n', kv, rmse0, rmse30, degrade);
end

% 恢复到默认 K_obs=8
content = fileread(params_file);
content = regexprep(content, 'params\.ucco\.K_obs\s*=\s*[\d.]+', 'params.ucco.K_obs = 8.0');
content = regexprep(content, 'params\.ucco\.max_dc\s*=\s*[\d.]+', 'params.ucco.max_dc = 0.08');
fid = fopen(params_file, 'w'); fprintf(fid, '%s', content); fclose(fid);

fprintf('\n扫描完成，已恢复到 K_obs=8\n');
