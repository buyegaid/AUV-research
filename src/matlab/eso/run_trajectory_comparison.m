function results = run_trajectory_comparison(mode)
% RUN_TRAJECTORY_COMPARISON  轨迹对比实验: 圆形 vs 直线 × 4基线 × 4失配水平
%
% 目的: 系统对比不同轨迹激励条件下各基线方法的RMSE表现
%   圆形 = 持续激励（论文 Tab 1 主表）
%   直线 = 低激励（论文 proposal 提及但系统数据缺失）
%
% 用法:
%   run_trajectory_comparison('sanity')   % 快速自检: 1 seed × 圆形+直线
%   run_trajectory_comparison('straight') % 直线批量: 4失配 × 4基线 × 5 seeds
%   run_trajectory_comparison('circle')   % 圆形复核: 4失配 × 4基线 × 5 seeds
%   run_trajectory_comparison('all')      % 全部运行
%
% 配置:
%   基线:  KIN, CFD-Luenberger, EKF, PC-RCO
%   轨迹:  circle(圆形/持续激励), straight(直线/低激励)
%   失配:  0%, 10%, 20%, 30%
%   噪声:  low (σ=0.02 m/s DVL)
%   海流:  Vc=0.3 m/s, β=45° 恒流
%   Seeds: 5
%
% 输出:  results/trajectory_comparison.mat
%
% 2026-06-23

if nargin < 1, mode = 'sanity'; end

project_root = setup_paths();

%% ===== 实验参数 =====
trajectories = {'circle', 'straight'};
mismatches   = [0, 10, 20, 30];
observers    = {'KIN','BORHAUG','EKF','UCCO'};  % 内部名称
obs_labels   = {'KIN','CFD-Luenberger','EKF','PC-RCO'};  % 论文名称
n_seeds      = 5;
noise_level  = 'low';   % DVL σ=0.02 m/s

switch mode
    case 'sanity'
        traj_subset = trajectories;  % 圆形+直线都做
        mis_subset  = 0;
        seed_subset = 1;
        fprintf('===== 轨迹对比实验: SANITY 自检 =====\n');
    case 'straight'
        traj_subset = {'straight'};
        mis_subset  = mismatches;
        seed_subset = 1:n_seeds;
        fprintf('===== 轨迹对比实验: 直线批量 =====\n');
    case 'circle'
        traj_subset = {'circle'};
        mis_subset  = mismatches;
        seed_subset = 1:n_seeds;
        fprintf('===== 轨迹对比实验: 圆形复核 =====\n');
    case 'all'
        traj_subset = trajectories;
        mis_subset  = mismatches;
        seed_subset = 1:n_seeds;
        fprintf('===== 轨迹对比实验: 全部运行 =====\n');
    otherwise
        error('未知模式: %s。可用: sanity, straight, circle, all', mode);
end

n_total = length(traj_subset) * length(observers) * length(mis_subset) * length(seed_subset);
fprintf('轨迹: %s\n', strjoin(traj_subset, ', '));
fprintf('失配: %s %%\n', strjoin(cellstr(num2str(mis_subset')), ', '));
fprintf('基线: %s\n', strjoin(obs_labels, ', '));
fprintf('噪声: %s,  种子数: %d,  总计: %d runs\n\n', noise_level, length(seed_subset), n_total);

%% ===== 运行 =====
n_done = 0;
t_start = tic;

% 结果存储: results.(traj).(observer).(mismatch).rmse_mean, rmse_std
results = struct();
results.cfg = struct('trajectories', {traj_subset}, 'mismatches', mis_subset, ...
    'observers', {observers}, 'obs_labels', {obs_labels}, ...
    'n_seeds', length(seed_subset), 'noise_level', noise_level, ...
    'date', datestr(now, 'yyyy-mm-dd HH:MM:SS'));

for ti = 1:length(traj_subset)
    traj = traj_subset{ti};
    if ~isfield(results, traj), results.(traj) = struct(); end

    for oi = 1:length(observers)
        obs = observers{oi};
        if ~isfield(results.(traj), obs), results.(traj).(obs) = struct(); end

        for mi = 1:length(mis_subset)
            mm = mis_subset(mi);
            mm_str = sprintf('%d%%', mm);
            if ~isfield(results.(traj).(obs), ['m', strrep(mm_str,'%','')])
                results.(traj).(obs).(['m', strrep(mm_str,'%','')]) = struct();
            end

            rmse_vals = zeros(1, length(seed_subset));

            for s_idx = 1:length(seed_subset)
                s = seed_subset(s_idx);
                n_done = n_done + 1;

                cfg.scenario = traj;
                cfg.mismatch_pct = mm;
                cfg.observers = {obs};  % 每次只跑一个观测器
                cfg.noise_level = noise_level;
                cfg.seed = s;
                cfg.T_end = 100;
                cfg.verbose = false;

                t_elapsed = toc(t_start);
                eta_str = '';
                if n_done > 1
                    eta_s = t_elapsed / (n_done - 1) * (n_total - n_done + 1);
                    if eta_s > 60
                        eta_str = sprintf(' | ETA: %.0f min', eta_s/60);
                    else
                        eta_str = sprintf(' | ETA: %.0f s', eta_s);
                    end
                end

                fprintf('[%2d/%2d] %-8s  %-5s  失配=%2d%%  seed=%d (%.1fs)%s\n', ...
                    n_done, n_total, traj, obs, mm, s, t_elapsed, eta_str);

                r = run_observer_comparison(cfg);
                rmse_vals(s_idx) = r.(obs).RMSE;
            end

            % 存均值和标准差
            m_field = ['m', strrep(mm_str,'%','')];
            results.(traj).(obs).(m_field).rmse_mean = mean(rmse_vals);
            results.(traj).(obs).(m_field).rmse_std  = std(rmse_vals);
            results.(traj).(obs).(m_field).rmse_vals = rmse_vals;  % 保留原始值
        end
    end
end

%% ===== 保存结果 =====
out_dir = fullfile(project_root, 'results');
if ~exist(out_dir, 'dir'), mkdir(out_dir); end

% 按轨迹分开保存，避免覆盖
traj_name = strjoin(traj_subset, '_');
save_path = fullfile(out_dir, sprintf('trajectory_comparison_%s.mat', traj_name));
save(save_path, 'results');
fprintf('\n结果已保存到: %s\n', save_path);

%% ===== 打印汇总表 =====
print_summary_table(results, obs_labels);

end

%% ==================== 汇总表打印 ====================
function print_summary_table(results, obs_labels)
fprintf('\n');
fprintf('╔══════════════════════════════════════════════════════════════════════════════╗\n');
fprintf('║              轨迹对比实验: RMSE_Vc (m/s)  汇总表                           ║\n');
fprintf('╠══════════════════════════════════════════════════════════════════════════════╣\n');
fprintf('║  噪声: low (σ=0.02 m/s)  海流: Vc=0.3 m/s, β=45°  种子: 5               ║\n');
fprintf('╠══════════════════════════════════════════════════════════════════════════════╣\n');

traj_list = fieldnames(results);
traj_list = traj_list(~strcmp(traj_list, 'cfg'));  % 排除cfg字段

for ti = 1:length(traj_list)
    traj = traj_list{ti};

    % 表头
    fprintf('║                                                                              ║\n');
    fprintf('║  ▸ %s 轨迹 (持续激励)                                                    ║\n', traj);
    fprintf('║  %-6s', '失配');
    for oi = 1:length(obs_labels)
        fprintf(' %-12s', obs_labels{oi});
    end
    fprintf('  ║\n');
    fprintf('║  %-6s %-12s %-12s %-12s %-12s  ║\n', '------', '------------', '------------', '------------', '------------');

    % 动态获取已运行的失配水平
    first_obs = map_label_to_internal(obs_labels{1});
    m_fields = fieldnames(results.(traj).(first_obs));
    m_fields = m_fields(startsWith(m_fields, 'm'));
    mismatch_values = zeros(1, length(m_fields));
    for fi = 1:length(m_fields)
        mismatch_values(fi) = str2double(erase(m_fields{fi}, 'm'));
    end
    mismatch_values = sort(mismatch_values);

    for mi = 1:length(mismatch_values)
        mm = mismatch_values(mi);
        m_field = ['m', num2str(mm)];
        fprintf('║  %-5d%%', mm);
        for oi = 1:length(obs_labels)
            obs_key = obs_labels{oi};
            obs_internal = map_label_to_internal(obs_key);
            if isfield(results.(traj).(obs_internal), m_field)
                val = results.(traj).(obs_internal).(m_field).rmse_mean;
                fprintf(' %-12.4f', val);
            else
                fprintf(' %-12s', 'N/A');
            end
        end
        fprintf('  ║\n');
    end
end

fprintf('╚══════════════════════════════════════════════════════════════════════════════╝\n');
fprintf('\n对比关键发现:\n');
fprintf('  1. 圆形 vs 直线的RMSE差异 → 激励水平对方法精度的影响\n');
fprintf('  2. 直线下 CFD-Luenberger 是否崩塌 (>1.0 表示崩塌)\n');
fprintf('  3. PC-RCO 在直线下的退化幅度 vs EKF\n');
fprintf('  4. KIN 的两轨迹一致性（模型无关特性）\n');
end

%% ==================== 名称映射 ====================
function obs_internal = map_label_to_internal(paper_label)
% 论文名称 → run_observer_comparison内部存储名称
switch paper_label
    case 'KIN',              obs_internal = 'KIN';
    case 'CFD-Luenberger',   obs_internal = 'BORHAUG';
    case 'EKF',              obs_internal = 'EKF';
    case 'PC-RCO',           obs_internal = 'UCCO';
    otherwise,               obs_internal = paper_label;
end
end
