% 只跑 PC-RCO (K_obs=10, max_dc=0.06)
try
    setup_paths();
    params = get_params();
    fprintf('PC-RCO: K_obs=%.0f, max_dc=%.2f\n', params.ucco.K_obs, params.ucco.max_dc);

    % 运行圆形和直线
    for traj = {'circle','straight'}
        tname = traj{1};
        fprintf('--- %s ---\n', tname);
        results = run_trajectory_comparison(tname);

        % 提取 PC-RCO 值
        mismatches = [0 10 20 30];
        fprintf('  PC-RCO RMSE: ');
        for m = mismatches
            val = results.(tname).UCCO.(['m' num2str(m)]).rmse_mean;
            fprintf('%.4f ', val);
        end
        fprintf('\n');
    end
    fprintf('完成\n');
catch ME
    fprintf(2, 'ERROR: %s\n', ME.message);
    exit(1);
end
exit(0);
