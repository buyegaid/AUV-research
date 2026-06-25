function results = run_ablation_v2(scenario_id)
% RUN_ABLATION_V2  新消融实验: 4场景 × 4变体 × 多指标
%   针对6个已识别问题进行修复后的消融实验。
%
%   场景:
%     'B1' — 阶跃海流+大失配 (测试CLAMP)
%     'B2' — 间歇机动 (测试GATE+GM)
%     'B3' — 高噪声圆形 (测试PREDCORR)
%     'B4' — 时变正弦海流 (综合压力)
%     'all' — 运行全部4个场景
%
%   变体: PCRCO(Full), NoGM, NoClamp, NoPredCorr
%
%   2026-06-21

if nargin < 1, scenario_id = 'B1'; end

project_root = setup_paths();
params = get_params();

if strcmp(scenario_id, 'all')
    all_results = struct();
    for sc = {'B1','B2','B3','B4'}
        all_results.(sc{1}) = run_ablation_v2(sc{1});
    end
    print_summary_table(all_results);
    results = all_results;
    return;
end

%% ===== 场景配置 =====
cfg = get_scenario_config(scenario_id);
variants = {'PCRCO','NoGM','NoClamp','NoPredCorr'};
n_seeds = cfg.n_seeds;
N = round(cfg.T_end / cfg.dt) + 1;

fprintf('===== 消融实验 V2: 场景 %s =====\n', scenario_id);
fprintf('描述: %s\n', cfg.desc);
fprintf('Vc=%.2f m/s, 失配=%d%%, 噪声=%s, seeds=%d\n', ...
    cfg.Vc_base, cfg.mismatch_pct, cfg.noise_level, n_seeds);
fprintf('变体: %s\n\n', strjoin(variants, ', '));

%% ===== 运行所有变体 =====
% 存储结构: results.(variant).(metric)
results = struct();
results.cfg = cfg;
results.scenario_id = scenario_id;

for vi = 1:length(variants)
    vn = variants{vi};
    fprintf('--- %s ---\n', vn);

    % 预分配
    c_rmse_transient = zeros(1, n_seeds);
    c_rmse_steady    = zeros(1, n_seeds);
    c_tau90          = zeros(1, n_seeds);
    c_overshoot      = zeros(1, n_seeds);
    c_bias_final     = zeros(1, n_seeds);
    c_gate_open      = zeros(1, n_seeds);
    c_clamp_pct      = zeros(1, n_seeds);
    c_dc_max         = zeros(1, n_seeds);

    for s = 1:n_seeds
        rng(s);
        result_seed = run_single_seed(cfg, vn, params, s);
        c_rmse_transient(s) = result_seed.rmse_transient;
        c_rmse_steady(s)    = result_seed.rmse_steady;
        c_tau90(s)          = result_seed.tau_90;
        c_overshoot(s)      = result_seed.overshoot_max;
        c_bias_final(s)     = result_seed.bias_final;
        c_gate_open(s)      = result_seed.gate_open_pct;
        c_clamp_pct(s)      = result_seed.clamp_pct;
        c_dc_max(s)         = result_seed.dc_max;
    end

    % 存储均值和标准差
    results.(vn).rmse_transient_mean = mean(c_rmse_transient);
    results.(vn).rmse_transient_std  = std(c_rmse_transient);
    results.(vn).rmse_steady_mean    = mean(c_rmse_steady);
    results.(vn).rmse_steady_std     = std(c_rmse_steady);
    results.(vn).tau_90_mean         = mean(c_tau90);
    results.(vn).overshoot_mean      = mean(c_overshoot);
    results.(vn).bias_final_mean     = mean(c_bias_final);
    results.(vn).gate_open_pct_mean  = mean(c_gate_open);
    results.(vn).clamp_pct_mean      = mean(c_clamp_pct);
    results.(vn).dc_max_mean         = mean(c_dc_max);

    fprintf('  RMSE_transient=%.4f±%.4f  RMSE_steady=%.4f±%.4f  τ_90=%.1fs  gate=%.0f%%  clamp=%.1f%%  dc_max=%.4f\n', ...
        results.(vn).rmse_transient_mean, results.(vn).rmse_transient_std, ...
        results.(vn).rmse_steady_mean, results.(vn).rmse_steady_std, ...
        results.(vn).tau_90_mean, results.(vn).gate_open_pct_mean*100, ...
        results.(vn).clamp_pct_mean*100, results.(vn).dc_max_mean);
end

%% ===== 对比表 =====
fprintf('\n===== 场景 %s 消融对比 =====\n', scenario_id);
fprintf('%-12s %-16s %-16s %-10s %-8s\n', '变体', 'RMSE_transient', 'RMSE_steady', 'τ_90(s)', 'vs Full');
fprintf('%s\n', repmat('-', 1, 65));

base_rmse_t = results.PCRCO.rmse_transient_mean;
base_rmse_s = results.PCRCO.rmse_steady_mean;

for vi = 1:4
    vn = variants{vi};
    rt = results.(vn).rmse_transient_mean;
    rs = results.(vn).rmse_steady_mean;
    tt = results.(vn).tau_90_mean;
    if vi == 1
        fprintf('%-12s %-16.4f %-16.4f %-10.1f  --\n', vn, rt, rs, tt);
    else
        diff_t = (rt - base_rmse_t) / base_rmse_t * 100;
        fprintf('%-12s %-16.4f %-16.4f %-10.1f  %+.1f%%\n', vn, rt, rs, tt, diff_t);
    end
end

%% ===== 诊断指标对比 =====
fprintf('\n%-12s %-12s %-12s %-12s %-12s\n', '变体', 'gate_open%', 'clamp%', 'dc_max', 'overshoot');
for vi = 1:4
    vn = variants{vi};
    fprintf('%-12s %-11.1f%% %-11.2f%% %-12.4f %-12.4f\n', ...
        vn, results.(vn).gate_open_pct_mean*100, results.(vn).clamp_pct_mean*100, ...
        results.(vn).dc_max_mean, results.(vn).overshoot_mean);
end

%% ===== 有效性判断 =====
fprintf('\n--- 消融有效性判断 ---\n');
check_ablation_validity(results, variants);

% 保存结果
save(fullfile(project_root, 'results', sprintf('ablation_v2_%s.mat', scenario_id)), 'results');
fprintf('\n结果已保存到 results/ablation_v2_%s.mat\n', scenario_id);

end

%% ==================== 单种子运行 ====================
function result = run_single_seed(cfg, variant_name, params, seed)
rng(seed);

dt = cfg.dt;
T_end = cfg.T_end;
N = round(T_end / dt) + 1;

% 轨迹
if strcmp(cfg.traj_type, 'circle')
    pts = traj(20, 50);
    x_state = [cfg.u0; 0; 0; 0; 0; 0; cfg.xn0; cfg.yn0; 10; 0; 0; cfg.psi0];
elseif strcmp(cfg.traj_type, 'intermittent')
    % 间歇机动: 构建航点序列 [直线30s → 转弯 → 直线30s → ...]
    pts = build_intermittent_waypoints(cfg.T_end, cfg.u0);
    x_state = [cfg.u0; 0; 0; 0; 0; 0; 0; 0; 10; 0; 0; 0];
end

M_const = compute_M_constant();
thr_params = get_thr_params();
opt_xhy = struct('mode', 'rpm', 'mismatch_pct', cfg.mismatch_pct, 'mismatch_seed', seed);

% 初始化
psi_d = cfg.psi0; r_d = 0; u_d = cfg.u0; ui = zeros(5, 1);

% 观测器初始化
c_hat = [0; 0];
reset_observer(variant_name);

% 记录
c_hist = zeros(N, 2);
gate_hist = zeros(N, 1);
dc_hist = zeros(N, 1);

for i = 1:N
    nu = x_state(1:6); xn = x_state(7); yn = x_state(8); zn = x_state(9);
    psi = x_state(12);

    % 时变海流
    [Vc_now, betaVc_now] = get_current(cfg, (i-1)*dt);

    % 传感器噪声
    nu_meas = nu;
    switch cfg.noise_level
        case 'low',  nu_meas(1:2) = nu(1:2) + 0.02 * randn(2,1);
        case 'high', nu_meas(1:2) = nu(1:2) + 0.10 * randn(2,1);
    end

    % 动力学
    [~, ~, M, ~, ~, ~, tau_thr] = xhy(x_state, ui, Vc_now, betaVc_now, 0, opt_xhy);

    % 制导 + 控制
    [psi_ref, ~, ~, ~, ~, ~, ~] = my_ALOS3D(xn, yn, zn, dt, pts, params.alos);
    [psi_d, r_d] = LOSobserver(psi_d, r_d, psi_ref, dt, params.alos.K_f);
    r_d = sat(r_d, 0.5);
    X_cmd = smc_surge_xhy(nu(1), u_d, 0, dt, params.xhy.surge);
    N_cmd = smc_yaw_xhy(psi, nu(6), psi_d, r_d, 0, dt, params.xhy.yaw);
    Z_cmd = smc_heave_xhy(zn, nu(3), 10, 0, 0, dt, params.xhy.heave);
    tau_cmd = [X_cmd; 0; Z_cmd; 0; 0; N_cmd];
    [ui, ~] = thrust_allocation_xhy(tau_cmd, thr_params);

    % 观测器
    ch_prev = c_hat;
    [c_hat, aux] = call_observer(variant_name, c_hat, nu_meas, tau_thr, psi, M_const, params);

    c_hist(i, :) = c_hat';
    gate_hist(i) = aux.excited;
    dc_hist(i) = norm(c_hat - ch_prev);

    % 状态更新
    x_state = rk4(@xhy, dt, x_state, ui, Vc_now, betaVc_now, 0, opt_xhy);
    x_state(12) = ssa(x_state(12));
end

% 真实海流时程
c_true_hist = zeros(N, 2);
for i = 1:N
    [~, ~, c_t] = get_current_trace(cfg, (i-1)*dt);
    c_true_hist(i, :) = c_t';
end

% ==== 指标计算 ====
% RMSE_transient: 前20s, 无burn-in
transient_end = round(20 / dt);
c_err_transient = c_hist(1:transient_end, :) - c_true_hist(1:transient_end, :);
result.rmse_transient = sqrt(mean(sum(c_err_transient.^2, 2)));

% RMSE_steady: 20s-结束
steady_start = transient_end + 1;
c_err_steady = c_hist(steady_start:end, :) - c_true_hist(steady_start:end, :);
result.rmse_steady = sqrt(mean(sum(c_err_steady.^2, 2)));

% τ_90: 达到90%最终值的时间
c_norm = sqrt(sum(c_hist.^2, 2));
c_final_median = median(c_norm(round(N*0.8):end));
threshold_90 = 0.9 * c_final_median;
idx_90 = find(c_norm >= threshold_90, 1, 'first');
if isempty(idx_90), idx_90 = N; end
result.tau_90 = idx_90 * dt;

% overshoot
c_true_norm = sqrt(sum(c_true_hist.^2, 2));
overshoot_vec = c_norm - c_true_norm;
result.overshoot_max = max(overshoot_vec);

% bias_final
result.bias_final = norm(c_hist(end, :) - c_true_hist(end, :));

% gate_open_pct
result.gate_open_pct = mean(gate_hist);

% clamp_pct (仅限Full和NoGM)
if ismember(variant_name, {'PCRCO','NoGM'})
    % 限幅触发: |dc| == max_dc (被夹住了)
    result.clamp_pct = mean(abs(dc_hist - params.ucco.max_dc) < 1e-10 & dc_hist > 0);
else
    result.clamp_pct = 0;
end

% dc_max
result.dc_max = max(dc_hist);
end

%% ==================== 场景配置 ====================
function cfg = get_scenario_config(scenario_id)
cfg.dt = 0.01;
cfg.u0 = 1.0;

switch scenario_id
    case 'B1'  % 阶跃海流+大失配 → 测试CLAMP
        cfg.desc = '阶跃海流(0.15→0.45)+30%失配';
        cfg.T_end = 100;
        cfg.traj_type = 'circle';
        cfg.Vc_base = 0.15;  cfg.Vc_step = 0.45;
        cfg.betaVc = deg2rad(45);
        cfg.Vc_mode = 'step';  cfg.step_time = 50;
        cfg.mismatch_pct = 30;
        cfg.noise_level = 'low';
        cfg.psi0 = pi/2;  cfg.xn0 = 300;  cfg.yn0 = 0;
        cfg.n_seeds = 5;

    case 'B2'  % 间歇机动 → 测试GATE+GM
        cfg.desc = '间歇机动(直线30s↔转弯)';
        cfg.T_end = 150;
        cfg.traj_type = 'intermittent';
        cfg.Vc_base = 0.3;
        cfg.betaVc = deg2rad(45);
        cfg.Vc_mode = 'constant';
        cfg.mismatch_pct = 0;
        cfg.noise_level = 'low';
        cfg.psi0 = 0;  cfg.xn0 = 0;  cfg.yn0 = 0;
        cfg.n_seeds = 5;

    case 'B3'  % 高噪声圆形 → 测试PREDCORR
        cfg.desc = '高噪声(σ=0.10)圆形';
        cfg.T_end = 100;
        cfg.traj_type = 'circle';
        cfg.Vc_base = 0.3;
        cfg.betaVc = deg2rad(45);
        cfg.Vc_mode = 'constant';
        cfg.mismatch_pct = 0;
        cfg.noise_level = 'high';
        cfg.psi0 = pi/2;  cfg.xn0 = 300;  cfg.yn0 = 0;
        cfg.n_seeds = 5;

    case 'B4'  % 时变正弦海流 → 综合压力
        cfg.desc = '时变海流 Vc(t)=0.3+0.15sin(2πt/200)';
        cfg.T_end = 200;
        cfg.traj_type = 'circle';
        cfg.Vc_base = 0.3;  cfg.Vc_amp = 0.15;  cfg.Vc_period = 200;
        cfg.betaVc = deg2rad(45);
        cfg.Vc_mode = 'sinusoidal';
        cfg.mismatch_pct = 20;
        cfg.noise_level = 'low';
        cfg.psi0 = pi/2;  cfg.xn0 = 300;  cfg.yn0 = 0;
        cfg.n_seeds = 5;
end
end

%% ==================== 海流生成 ====================
function [Vc, betaVc] = get_current(cfg, t)
switch cfg.Vc_mode
    case 'constant'
        Vc = cfg.Vc_base; betaVc = cfg.betaVc;
    case 'step'
        if t >= cfg.step_time
            Vc = cfg.Vc_step; betaVc = deg2rad(60);
        else
            Vc = cfg.Vc_base; betaVc = cfg.betaVc;
        end
    case 'sinusoidal'
        Vc = cfg.Vc_base + cfg.Vc_amp * sin(2*pi*t / cfg.Vc_period);
        betaVc = cfg.betaVc;
    otherwise
        Vc = cfg.Vc_base; betaVc = cfg.betaVc;
end
end

function [Vc, betaVc, c_true] = get_current_trace(cfg, t)
[Vc, betaVc] = get_current(cfg, t);
c_true = [Vc*cos(betaVc); Vc*sin(betaVc)];
end

%% ==================== 观测器调用 ====================
function reset_observer(variant_name)
switch variant_name
    case 'PCRCO',      clear eg_ucco_simple;
    case 'NoGM',       pcrco_no_gm([],[],[],[],[],[],[],true);
    case 'NoClamp',    pcrco_no_clamp([],[],[],[],[],[],[],true);
    case 'NoPredCorr', pcrco_no_predcorr([],[],[],[],[],[],[],true);
end
end

function [c_hat, aux] = call_observer(variant_name, c_hat, nu_meas, tau, psi, M, params)
switch variant_name
    case 'PCRCO'
        [c_hat, aux] = eg_ucco_simple(c_hat, nu_meas, tau, psi, M, params.ucco, 0.01);
    case 'NoGM'
        [c_hat, aux] = pcrco_no_gm(c_hat, nu_meas, tau, psi, M, params.ucco, 0.01);
    case 'NoClamp'
        [c_hat, aux] = pcrco_no_clamp(c_hat, nu_meas, tau, psi, M, params.ucco, 0.01);
    case 'NoPredCorr'
        [c_hat, aux] = pcrco_no_predcorr(c_hat, nu_meas, tau, psi, M, params.ucco, 0.01);
end
end

%% ==================== 间歇机动航点 ====================
function pts = build_intermittent_waypoints(T_end, u0)
% 构建"直线→90°转弯→直线→..."的航点序列
% 每段直线30s, 转弯速度~5°/s → 转弯~18s
straight_len = 30;  % 直线段时间(s)
turn_time = 18;     % 转弯时间(s)
segment_time = straight_len + turn_time;

n_segments = ceil(T_end / segment_time);
x = zeros(n_segments * 2 + 1, 1);
y = zeros(n_segments * 2 + 1, 1);
z = 10 * ones(n_segments * 2 + 1, 1);

% 航向: 0°(北) → 90°(东) → 180°(南) → 270°(西) → 0°...
directions = [0, pi/2, pi, 3*pi/2];
cur_x = 0; cur_y = 0;
wp_idx = 1;
x(wp_idx) = cur_x; y(wp_idx) = cur_y;

for seg = 1:n_segments
    heading = directions(mod(seg-1, 4) + 1);
    % 直线终点
    cur_x = cur_x + u0 * straight_len * cos(heading);
    cur_y = cur_y + u0 * straight_len * sin(heading);
    wp_idx = wp_idx + 1;
    x(wp_idx) = cur_x; y(wp_idx) = cur_y;
end

% 移除多余预分配
x = x(1:wp_idx); y = y(1:wp_idx); z = z(1:wp_idx);
pts.pos.x = x; pts.pos.y = y; pts.pos.z = z;
end

%% ==================== 汇总表 ====================
function print_summary_table(all_results)
fprintf('\n');
fprintf('╔══════════════════════════════════════════════════════════════════╗\n');
fprintf('║              消融实验 V2 汇总 (所有场景)                        ║\n');
fprintf('╠══════════════════════════════════════════════════════════════════╣\n');

for sc = {'B1','B2','B3','B4'}
    sid = sc{1};
    if ~isfield(all_results, sid), continue; end
    r = all_results.(sid);

    fprintf('║ 场景 %s: %-52s ║\n', sid, r.cfg.desc);
    fprintf('║ %-10s %-12s %-12s %-10s %-8s ║\n', '变体', 'RMSE_trans', 'RMSE_steady', 'τ_90', 'ΔvsFull');
    fprintf('║ %-10s %-12s %-12s %-10s %-8s ║\n', '----------', '------------', '------------', '----------', '--------');

    base_t = r.PCRCO.rmse_transient_mean;
    for vi = {'PCRCO','NoGM','NoClamp','NoPredCorr'}
        vn = vi{1};
        if ~isfield(r, vn), continue; end
        rt = r.(vn).rmse_transient_mean;
        rs = r.(vn).rmse_steady_mean;
        tt = r.(vn).tau_90_mean;
        if strcmp(vn, 'PCRCO')
            fprintf('║ %-10s %-12.4f %-12.4f %-10.1f %-8s ║\n', vn, rt, rs, tt, '--');
        else
            diff = (rt - base_t) / base_t * 100;
            fprintf('║ %-10s %-12.4f %-12.4f %-10.1f %+7.1f%% ║\n', vn, rt, rs, tt, diff);
        end
    end
    fprintf('║ %-10s %-12s %-12s %-10s %-8s ║\n', '', '', '', '', '');
end
fprintf('╚══════════════════════════════════════════════════════════════════╝\n');
end

%% ==================== 有效性判断 ====================
function check_ablation_validity(results, variants)
% 检查消融是否有效（各变体应有显著差异）
base_rt = results.PCRCO.rmse_transient_mean;
base_gt = results.PCRCO.gate_open_pct_mean;
base_cp = results.PCRCO.clamp_pct_mean;

fprintf('有效性标准:\n');
fprintf('  NoClamp vs Full: RMSE_transient应不同 (当前ratio=%.2fx)\n', ...
    results.NoClamp.rmse_transient_mean / max(1e-6, base_rt));
fprintf('  NoGM vs Full: gate_open%%应类似, 但RMSE_transient应不同\n');
fprintf('  NoPredCorr vs Full: 高噪声场景下应有≥5%%退化\n');

% 判断
if results.NoClamp.rmse_transient_mean / max(1e-6, base_rt) > 1.05
    fprintf('  ✅ CLAMP消融有效: NoClamp RMSE比Full高%.1f%%\n', ...
        (results.NoClamp.rmse_transient_mean/base_rt - 1)*100);
elseif abs(results.NoClamp.rmse_transient_mean - base_rt) < 1e-6
    fprintf('  ❌ CLAMP消融无效: NoClamp≈Full (差异<1e-6)\n');
else
    fprintf('  ⚠️  CLAMP消融边缘: 差异=%.4f (%.1f%%)\n', ...
        results.NoClamp.rmse_transient_mean - base_rt, ...
        (results.NoClamp.rmse_transient_mean/base_rt - 1)*100);
end
end

%% ==================== 辅助 ====================
function M = compute_M_constant()
m = 85.832; Ix = 0.553864787+0.865274; Iy = 2.162341935+5.011187; Iz = 1.849137+4.541468;
MRB = diag([m,m,m,Ix,Iy,Iz]); MA = diag([15.81,124.73,42.87,0.014,0.041,0.123]); M = MRB+MA;
end

function p = get_thr_params()
% 推进器参数 (2026-06-22 分段PWM反推, 与 thrust_main/thrust_aux 保持一致)
p.rho = 1026; p.D_prop_main = 0.08; p.D_prop_aux = 0.06;
p.KT_main_fwd = 0.1489; p.KT_main_rev = 0.0506; p.KT_aux_fwd = 0.53; p.KT_aux_rev = 0.71;
p.n_max = 2500; p.x_vert_f = +0.344; p.x_vert_r = -0.293; p.x_side_f = +0.424; p.x_side_r = -0.376;
end
