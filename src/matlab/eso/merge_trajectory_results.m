function merge_trajectory_results()
% 合并分开保存的轨迹对比结果到统一文件
% 输入: results/trajectory_comparison_{circle,straight}.mat
% 输出: results/trajectory_comparison.mat (合并后的完整结果)

project_root = setup_paths();
res_dir = fullfile(project_root, 'results');

fprintf('===== 合并轨迹对比结果 =====\n');

merged = struct();
trajs_loaded = {};

% 尝试加载各轨迹结果
for traj = {'circle', 'straight'}
    fname = fullfile(res_dir, sprintf('trajectory_comparison_%s.mat', traj{1}));
    if exist(fname, 'file')
        data = load(fname);
        fn = fieldnames(data.results);
        for fi = 1:length(fn)
            if ~strcmp(fn{fi}, 'cfg')
                merged.(fn{fi}) = data.results.(fn{fi});
                trajs_loaded{end+1} = fn{fi};
                fprintf('  已加载: %s (%d 观测器)\n', fn{fi}, ...
                    length(fieldnames(data.results.(fn{fi}))));
            end
        end
    else
        fprintf('  [跳过] 文件不存在: %s\n', fname);
    end
end

if isempty(trajs_loaded)
    fprintf('错误: 没有可合并的结果文件\n');
    return;
end

% 合并cfg
merged.cfg = struct('trajectories', {trajs_loaded}, ...
    'date', datestr(now, 'yyyy-mm-dd HH:MM:SS'));

% 保存
save_path = fullfile(res_dir, 'trajectory_comparison.mat');
save(save_path, 'merged');
fprintf('\n已合并 %d 个轨迹 → %s\n', length(trajs_loaded), save_path);

% 打印汇总表
obs_int = {'KIN','BORHAUG','EKF','UCCO'};
obs_lbl = {'KIN','CFD-Luenberger','EKF','PC-RCO'};
mismatches = [0, 10, 20, 30];

fprintf('\n');
fprintf('╔══════════════════════════════════════════════════════════════════════╗\n');
fprintf('║         轨迹对比实验: RMSE_Vc (m/s)  汇总 (5 seeds, low noise)     ║\n');
fprintf('╠══════════════════════════════════════════════════════════════════════╣\n');

for ti = 1:length(trajs_loaded)
    traj = trajs_loaded{ti};
    fprintf('║  ▸ %s 轨迹                                                     ║\n', traj);
    fprintf('║  %-5s', '失配');
    for oi = 1:4, fprintf(' %-12s', obs_lbl{oi}); end
    fprintf('   ║\n');
    fprintf('║  %-5s %-12s %-12s %-12s %-12s   ║\n', '-----', '------------', '------------', '------------', '------------');

    for mi = 1:length(mismatches)
        mm = mismatches(mi);
        m_field = ['m' num2str(mm)];
        fprintf('║  %-4d%%', mm);
        for oi = 1:4
            if isfield(merged.(traj), obs_int{oi}) && isfield(merged.(traj).(obs_int{oi}), m_field)
                val = merged.(traj).(obs_int{oi}).(m_field).rmse_mean;
                fprintf(' %-12.4f', val);
            else
                fprintf(' %-12s', 'N/A');
            end
        end
        fprintf('   ║\n');
    end

    % 退化率
    if isfield(merged.(traj).KIN, 'm0') && isfield(merged.(traj).KIN, 'm30')
        fprintf('║  %-5s', 'Δ0→30');
        for oi = 1:4
            base = merged.(traj).(obs_int{oi}).m0.rmse_mean;
            deg = merged.(traj).(obs_int{oi}).m30.rmse_mean;
            pct = (deg - base) / base * 100;
            fprintf(' %+11.1f%%', pct);
        end
        fprintf('   ║\n');
    end
end

fprintf('╚══════════════════════════════════════════════════════════════════════╝\n');

% 关键对比
fprintf('\n关键发现:\n');
for t = trajs_loaded
    tname = t{1};
    pc_base = merged.(tname).UCCO.m0.rmse_mean;
    ekf_base = merged.(tname).EKF.m0.rmse_mean;
    kin_base = merged.(tname).KIN.m0.rmse_mean;
    fprintf('  %s: PC-RCO=%.4f  EKF=%.4f  KIN=%.4f  (PC-RCO/EKF=%.1fx KIN, PC-RCO vs EKF: PC-RCO > EKF by %.0f%%)\n', ...
        tname, pc_base, ekf_base, kin_base, pc_base/kin_base, (pc_base-ekf_base)/ekf_base*100);
end

end
