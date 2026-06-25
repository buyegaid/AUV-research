%% 消融变体快速验证: 确认4变体输出不同
% 用B1场景(阶跃海流+30%失配)运行50步，逐变体对比
% 2026-06-21

project_root = setup_paths();
params = get_params();

%% ===== 固定条件（阶跃后，大失配）=====
rng(42);
dt = 0.01;
N = 50;

% 状态: 假设阶跃后 (Vc从0.15跳到0.45，失配30%)
nu = [0.8; 0.15; 0; 0; 0; 0.005];  % 阶跃后的典型状态
psi = pi/2 + 0.1;  % 轻微转向中
Vc_now = 0.45; betaVc_now = deg2rad(60);
c_true = [Vc_now*cos(betaVc_now); Vc_now*sin(betaVc_now)];

% 推力
[~, ~, ~, ~, ~, ~, tau_thr] = xhy(...
    [nu; 300; 0; 10; 0; 0; psi], zeros(5,1), Vc_now, betaVc_now, 0, ...
    struct('mode','rpm','mismatch_pct',30,'mismatch_seed',42));

% 惯性矩阵
m = 85.832; Ix = 1.419; Iy = 7.174; Iz = 6.391;
MRB = diag([m,m,m,Ix,Iy,Iz]);
MA = diag([15.81, 124.73, 42.87, 0.014, 0.041, 0.123]);
M_const = MRB + MA;

%% ===== 单步对比 =====
fprintf('===== 消融变体单步验证 =====\n');
fprintf('c_true=[%.4f,%.4f]  nu=[%.2f,%.2f,%.2f]  psi=%.1f°\n\n', ...
    c_true, nu(1:3), rad2deg(psi));

variants = {'PCRCO','NoGM','NoClamp','NoPredCorr'};
c_results = zeros(2, 4);
diag_info = cell(4, 1);

for vi = 1:4
    vn = variants{vi};

    % 重置
    switch vn
        case 'PCRCO',      clear eg_ucco_simple;
        case 'NoGM',       pcrco_no_gm([],[],[],[],[],[],[],true);
        case 'NoClamp',    pcrco_no_clamp([],[],[],[],[],[],[],true);
        case 'NoPredCorr', pcrco_no_predcorr([],[],[],[],[],[],[],true);
    end

    ch = [0; 0];  % 从零开始

    % 运行5步（跳过首步初始化）
    for k = 1:5
        nu_meas = nu + 0.02 * randn(6,1);  % low noise
        switch vn
            case 'PCRCO',      [ch, aux] = eg_ucco_simple(ch, nu_meas, tau_thr, psi, M_const, params.ucco, dt);
            case 'NoGM',       [ch, aux] = pcrco_no_gm(ch, nu_meas, tau_thr, psi, M_const, params.ucco, dt);
            case 'NoClamp',    [ch, aux] = pcrco_no_clamp(ch, nu_meas, tau_thr, psi, M_const, params.ucco, dt);
            case 'NoPredCorr', [ch, aux] = pcrco_no_predcorr(ch, nu_meas, tau_thr, psi, M_const, params.ucco, dt);
        end
    end

    c_results(:, vi) = ch;
    diag_info{vi} = aux;

    fprintf('%-12s: c_hat_5=[%+.8f,%+.8f]  |dc|=%+.8f  λ_min=%.2e  excited=%d\n', ...
        vn, ch(1), ch(2), norm(ch), aux.lambda_min, aux.excited);
end

%% ===== 差异检查 =====
fprintf('\n===== 差异矩阵 =====\n');
fprintf('%-12s %-12s %-12s %-12s\n', '', variants{1}, variants{2}, variants{3});
for vi = 2:4
    fprintf('%-12s', variants{vi});
    for vj = 1:vi-1
        diff = norm(c_results(:,vi) - c_results(:,vj));
        if diff > 1e-6
            fprintf(' %-12.2e✅', diff);
        else
            fprintf(' %-12s❌', 'ZERO!');
        end
    end
    fprintf('\n');
end

%% ===== 机制触发检查 =====
fprintf('\n===== 机制触发状态 =====\n');

% 门控检查
for vi = 1:4
    vn = variants{vi};
    excited = diag_info{vi}.excited;
    fprintf('%-12s: gate=%s', vn, condstr(excited));

    if isfield(diag_info{vi}, 'gm_applied')
        fprintf('  GM=%s', condstr(diag_info{vi}.gm_applied));
    end
    if isfield(diag_info{vi}, 'dc_unclamped')
        fprintf('  dc_unclamped=[%.6f,%.6f]', diag_info{vi}.dc_unclamped);
    end
    if isfield(diag_info{vi}, 'e_acc')
        fprintf('  e_acc=[%.4f,%.4f]', diag_info{vi}.e_acc);
    end
    if isfield(diag_info{vi}, 'e_vel')
        fprintf('  e_vel=[%.6f,%.6f]', diag_info{vi}.e_vel);
    end
    fprintf('\n');
end

%% ===== 50步累积对比 =====
fprintf('\n===== 50步累积对比 =====\n');
c_50 = zeros(2, 4);

for vi = 1:4
    vn = variants{vi};
    % 重置
    switch vn
        case 'PCRCO',      clear eg_ucco_simple;
        case 'NoGM',       pcrco_no_gm([],[],[],[],[],[],[],true);
        case 'NoClamp',    pcrco_no_clamp([],[],[],[],[],[],[],true);
        case 'NoPredCorr', pcrco_no_predcorr([],[],[],[],[],[],[],true);
    end
    ch = [0; 0];

    for k = 1:N
        nu_meas_k = nu + 0.02 * randn(6,1);
        switch vn
            case 'PCRCO',      [ch, ~] = eg_ucco_simple(ch, nu_meas_k, tau_thr, psi, M_const, params.ucco, dt);
            case 'NoGM',       [ch, ~] = pcrco_no_gm(ch, nu_meas_k, tau_thr, psi, M_const, params.ucco, dt);
            case 'NoClamp',    [ch, ~] = pcrco_no_clamp(ch, nu_meas_k, tau_thr, psi, M_const, params.ucco, dt);
            case 'NoPredCorr', [ch, ~] = pcrco_no_predcorr(ch, nu_meas_k, tau_thr, psi, M_const, params.ucco, dt);
        end
    end
    c_50(:, vi) = ch;
    fprintf('%-12s: c_hat_50=[%+.8f,%+.8f]  误差=%.6f\n', vn, ch(1), ch(2), norm(ch - c_true));
end

fprintf('\n===== 50步差异 vs PCRCO =====\n');
for vi = 2:4
    diff_50 = norm(c_50(:,vi) - c_50(:,1));
    if diff_50 > 1e-6
        fprintf('%-12s: Δ=%.6f (%.1f%%) ✅ 消融有效\n', variants{vi}, diff_50, diff_50/norm(c_50(:,1))*100);
    else
        fprintf('%-12s: Δ=0 ❌ 消融仍无效，需进一步调查\n', variants{vi});
    end
end

fprintf('\n验证完成。\n');

function s = condstr(cond)
if cond, s = 'OPEN'; else, s = 'CLOSED'; end
end
