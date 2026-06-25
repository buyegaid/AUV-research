function results = run_reviewer_experiments()
% RUN_REVIEWER_EXPERIMENTS  Round 2审查者要求的补强实验
%   E1: Clamp阈值敏感性 (max_dc sweep)
%   E2: 压力矩阵 (mismatch × noise) Full vs NoClamp
%   E3: Baseline Pareto对比
%   2026-06-22

project_root = setup_paths();
params = get_params();

dt = 0.01; T_end = 100; N = round(T_end/dt) + 1;
Vc = 0.3; betaVc = deg2rad(45);
c_true = [Vc*cos(betaVc); Vc*sin(betaVc)];
n_seeds = 3;  % 快速模式用3seeds

% 惯性矩阵
m=85.832; Ix=1.419; Iy=7.174; Iz=6.391;
MRB=diag([m,m,m,Ix,Iy,Iz]); MA=diag([15.81,124.73,42.87,0.014,0.041,0.123]);
M_const=MRB+MA;

thr = get_thr_params();

%% ===== E1: Clamp阈值敏感性 =====
fprintf('===== E1: Clamp阈值敏感性 =====\n');
max_dc_vals = [0.01, 0.03, 0.06, 0.10, 0.20, 1.0];  % 1.0 ≈ Inf for this scale
e1_results = struct();

for di = 1:length(max_dc_vals)
    dc_val = max_dc_vals(di);
    rmse_arr = zeros(1, n_seeds); dc_max_arr = zeros(1, n_seeds);
    overshoot_arr = zeros(1, n_seeds); bias_arr = zeros(1, n_seeds);

    for s = 1:n_seeds
        rng(s);
        % 使用阶跃海流+30%失配场景 (B1-like)
        [rmse, dc_m, ov, bias] = run_clamp_test(dc_val, params, M_const, thr, ...
            Vc, betaVc, c_true, T_end, dt, s, 30);
        rmse_arr(s) = rmse; dc_max_arr(s) = dc_m;
        overshoot_arr(s) = ov; bias_arr(s) = bias;
    end

    key = sprintf('dc_%02d', round(dc_val*100));
    e1_results.(key).rmse_mean = mean(rmse_arr);
    e1_results.(key).rmse_std = std(rmse_arr);
    e1_results.(key).dc_max_mean = mean(dc_max_arr);
    e1_results.(key).overshoot_mean = mean(overshoot_arr);
    e1_results.(key).bias_mean = mean(bias_arr);

    fprintf('max_dc=%.2f: RMSE=%.4f±%.4f  dc_max=%.4f  overshoot=%.4f  bias=%.4f\n', ...
        dc_val, mean(rmse_arr), std(rmse_arr), mean(dc_max_arr), mean(overshoot_arr), mean(bias_arr));
end

%% ===== E2: 压力矩阵 (快速版: 3×3) =====
fprintf('\n===== E2: 压力矩阵 (mismatch × noise) =====\n');
mismatches = [0, 20, 30];
noises = {'low', 'high'};
e2_results = struct();

for mi = 1:length(mismatches)
    for ni = 1:length(noises)
        mm = mismatches(mi); nl = noises{ni};
        key = sprintf('mm%02d_%s', mm, nl);

        for vn = {'Full', 'NoClamp'}
            vk = vn{1};
            rmse_arr = zeros(1, n_seeds); dc_max_arr = zeros(1, n_seeds);
            overshoot_arr = zeros(1, n_seeds);

            for s = 1:n_seeds
                rng(s);
                [rmse, dc_m, ov, ~] = run_stress_test(vk, params, M_const, thr, ...
                    Vc, betaVc, c_true, T_end, dt, s, mm, nl);
                rmse_arr(s) = rmse; dc_max_arr(s) = dc_m; overshoot_arr(s) = ov;
            end

            e2_results.(key).(vk).rmse = mean(rmse_arr);
            e2_results.(key).(vk).dc_max = mean(dc_max_arr);
            e2_results.(key).(vk).overshoot = mean(overshoot_arr);
        end

        fprintf('mm=%d%% %-5s: Full RMSE=%.4f dc=%.3f | NoClamp RMSE=%.4f dc=%.3f | dc_ratio=%.1fx\n', ...
            mm, nl, e2_results.(key).Full.rmse, e2_results.(key).Full.dc_max, ...
            e2_results.(key).NoClamp.rmse, e2_results.(key).NoClamp.dc_max, ...
            e2_results.(key).NoClamp.dc_max / max(1e-6, e2_results.(key).Full.dc_max));
    end
end

%% ===== E3: Baseline Pareto对比 =====
fprintf('\n===== E3: Baseline Pareto (B3高噪声场景) =====\n');
baselines = {'KIN', 'CFDLuenberger', 'EKF', 'PCRCO', 'NoClamp', 'NoPredCorr'};
e3_results = struct();

for bi = 1:length(baselines)
    bn = baselines{bi};
    rmse_arr = zeros(1, n_seeds); dc_max_arr = zeros(1, n_seeds);
    overshoot_arr = zeros(1, n_seeds); bias_arr = zeros(1, n_seeds);

    for s = 1:n_seeds
        rng(s);
        [rmse, dc_m, ov, bias] = run_baseline_test(bn, params, M_const, thr, ...
            Vc, betaVc, c_true, T_end, dt, s);
        rmse_arr(s) = rmse; dc_max_arr(s) = dc_m;
        overshoot_arr(s) = ov; bias_arr(s) = bias;
    end

    e3_results.(bn).rmse = mean(rmse_arr);
    e3_results.(bn).dc_max = mean(dc_max_arr);
    e3_results.(bn).overshoot = mean(overshoot_arr);
    e3_results.(bn).bias = mean(bias_arr);

    fprintf('%-14s: RMSE=%.4f  dc_max=%.4f  overshoot=%.4f  bias=%.4f\n', ...
        bn, mean(rmse_arr), mean(dc_max_arr), mean(overshoot_arr), mean(bias_arr));
end

%% ===== 保存 =====
results.e1 = e1_results;
results.e2 = e2_results;
results.e3 = e3_results;
results.cfg.dc_vals = max_dc_vals;
results.cfg.mismatches = mismatches;
results.cfg.noises = noises;
results.cfg.n_seeds = n_seeds;

save(fullfile(project_root, 'results', 'reviewer_experiments.mat'), 'results');
fprintf('\n全部实验完成，结果已保存\n');
end

%% ==================== E1: Clamp阈值测试 ====================
function [rmse, dc_max, overshoot, bias_final] = run_clamp_test(max_dc, params, M_const, thr, ...
    Vc, betaVc, c_true, T_end, dt, seed, mismatch_pct)
N = round(T_end/dt) + 1; rng(seed);

% 圆形轨迹
pts = traj(20, 50);
x_state = [1.0;0;0;0;0;0;300;0;10;0;0;pi/2];
opt_xhy = struct('mode','rpm','mismatch_pct',mismatch_pct,'mismatch_seed',seed);

psi_d=pi/2; r_d=0; u_d=1.0; ui=zeros(5,1);
clear eg_ucco_simple; c_hat=[0;0];

% 临时修改max_dc
orig_max_dc = params.ucco.max_dc;
params.ucco.max_dc = max_dc;

c_hist=zeros(N,2); dc_hist=zeros(N,1);

for i=1:N
    nu=x_state(1:6); xn=x_state(7); yn=x_state(8); zn=x_state(9); psi=x_state(12);

    % 阶跃在t=50s
    if (i-1)*dt >= 50
        Vc_now=0.45; betaVc_now=deg2rad(60);
    else
        Vc_now=Vc; betaVc_now=betaVc;
    end
    c_true_now=[Vc_now*cos(betaVc_now); Vc_now*sin(betaVc_now)];

    nu_meas=nu+0.02*randn(6,1);
    [~,~,M,~,~,~,tau_thr]=xhy(x_state,ui,Vc_now,betaVc_now,0,opt_xhy);
    [psi_ref,~,~,~,~,~,~]=my_ALOS3D(xn,yn,zn,dt,pts,params.alos);
    [psi_d,r_d]=LOSobserver(psi_d,r_d,psi_ref,dt,params.alos.K_f);
    r_d=sat(r_d,0.5);
    X=smc_surge_xhy(nu(1),u_d,0,dt,params.xhy.surge);
    Nc=smc_yaw_xhy(psi,nu(6),psi_d,r_d,0,dt,params.xhy.yaw);
    Z=smc_heave_xhy(zn,nu(3),10,0,0,dt,params.xhy.heave);
    tau_cmd=[X;0;Z;0;0;Nc];
    [ui,~]=thrust_allocation_xhy(tau_cmd,thr);

    ch_prev=c_hat;
    [c_hat,~]=eg_ucco_simple(c_hat,nu_meas,tau_thr,psi,M_const,params.ucco,dt);

    c_hist(i,:)=c_hat'; dc_hist(i)=norm(c_hat-ch_prev);
    x_state=rk4(@xhy,dt,x_state,ui,Vc_now,betaVc_now,0,opt_xhy);
    x_state(12)=ssa(x_state(12));
end

% 恢复
params.ucco.max_dc = orig_max_dc;

c_err=c_hist-repmat(c_true',N,1);
rmse=sqrt(mean(sum(c_err.^2,2)));
dc_max=max(dc_hist);
c_norm=sqrt(sum(c_hist.^2,2));
c_true_norm=sqrt(sum(c_true_now'.^2));
overshoot=max(c_norm-c_true_norm);
bias_final=norm(c_hist(end,:)-c_true_now');
end

%% ==================== E2: 压力矩阵测试 ====================
function [rmse, dc_max, overshoot, bias_final] = run_stress_test(variant, params, M_const, thr, ...
    Vc, betaVc, c_true, T_end, dt, seed, mismatch_pct, noise_level)
N = round(T_end/dt) + 1; rng(seed);

pts = traj(20, 50);
x_state = [1.0;0;0;0;0;0;300;0;10;0;0;pi/2];
opt_xhy = struct('mode','rpm','mismatch_pct',mismatch_pct,'mismatch_seed',seed);

psi_d=pi/2; r_d=0; u_d=1.0; ui=zeros(5,1);

% 重置
switch variant
    case 'Full', clear eg_ucco_simple;
    case 'NoClamp', pcrco_no_clamp([],[],[],[],[],[],[],true);
end
c_hat=[0;0];

c_hist=zeros(N,2); dc_hist=zeros(N,1);

for i=1:N
    nu=x_state(1:6); xn=x_state(7); yn=x_state(8); zn=x_state(9); psi=x_state(12);
    nu_meas=nu;
    switch noise_level
        case 'low', nu_meas(1:2)=nu(1:2)+0.02*randn(2,1);
        case 'high', nu_meas(1:2)=nu(1:2)+0.10*randn(2,1);
    end

    [~,~,M,~,~,~,tau_thr]=xhy(x_state,ui,Vc,betaVc,0,opt_xhy);
    [psi_ref,~,~,~,~,~,~]=my_ALOS3D(xn,yn,zn,dt,pts,params.alos);
    [psi_d,r_d]=LOSobserver(psi_d,r_d,psi_ref,dt,params.alos.K_f);
    r_d=sat(r_d,0.5);
    X=smc_surge_xhy(nu(1),u_d,0,dt,params.xhy.surge);
    Nc=smc_yaw_xhy(psi,nu(6),psi_d,r_d,0,dt,params.xhy.yaw);
    Z=smc_heave_xhy(zn,nu(3),10,0,0,dt,params.xhy.heave);
    tau_cmd=[X;0;Z;0;0;Nc];
    [ui,~]=thrust_allocation_xhy(tau_cmd,thr);

    ch_prev=c_hat;
    switch variant
        case 'Full'
            [c_hat,~]=eg_ucco_simple(c_hat,nu_meas,tau_thr,psi,M_const,params.ucco,dt);
        case 'NoClamp'
            [c_hat,~]=pcrco_no_clamp(c_hat,nu_meas,tau_thr,psi,M_const,params.ucco,dt);
    end

    c_hist(i,:)=c_hat'; dc_hist(i)=norm(c_hat-ch_prev);
    x_state=rk4(@xhy,dt,x_state,ui,Vc,betaVc,0,opt_xhy);
    x_state(12)=ssa(x_state(12));
end

c_err=c_hist-repmat(c_true',N,1);
rmse=sqrt(mean(sum(c_err.^2,2)));
dc_max=max(dc_hist);
c_norm=sqrt(sum(c_hist.^2,2)); c_tn=sqrt(sum(c_true.^2));
overshoot=max(c_norm-c_tn);
bias_final=norm(c_hist(end,:)-c_true');
end

%% ==================== E3: Baseline对比 ====================
function [rmse, dc_max, overshoot, bias_final] = run_baseline_test(baseline, params, M_const, thr, ...
    Vc, betaVc, c_true, T_end, dt, seed)
N = round(T_end/dt) + 1; rng(seed);

pts = traj(20, 50);
x_state = [1.0;0;0;0;0;0;300;0;10;0;0;pi/2];
opt_xhy = struct('mode','rpm','mismatch_pct',20,'mismatch_seed',seed);

psi_d=pi/2; r_d=0; u_d=1.0; ui=zeros(5,1);

% 初始化
clear eg_ucco_simple; clear borhaug_current_observer;
switch baseline
    case 'KIN', c_hat=[0;0];
    case 'CFDLuenberger', c_hat=[0;0]; clear borhaug_current_observer;
    case 'EKF', x_hat=[]; P=[];
    case 'PCRCO', c_hat=[0;0]; clear eg_ucco_simple;
    case 'NoClamp', c_hat=[0;0]; pcrco_no_clamp([],[],[],[],[],[],[],true);
    case 'NoPredCorr', c_hat=[0;0]; pcrco_no_predcorr([],[],[],[],[],[],[],true);
end

c_hist=zeros(N,2); dc_hist=zeros(N,1);

for i=1:N
    nu=x_state(1:6); xn=x_state(7); yn=x_state(8); zn=x_state(9); psi=x_state(12);
    nu_meas=nu+0.10*randn(6,1);  % high noise

    [~,~,M,~,~,~,tau_thr]=xhy(x_state,ui,Vc,betaVc,0,opt_xhy);
    [psi_ref,~,~,~,~,~,~]=my_ALOS3D(xn,yn,zn,dt,pts,params.alos);
    [psi_d,r_d]=LOSobserver(psi_d,r_d,psi_ref,dt,params.alos.K_f);
    r_d=sat(r_d,0.5);
    X=smc_surge_xhy(nu(1),u_d,0,dt,params.xhy.surge);
    Nc=smc_yaw_xhy(psi,nu(6),psi_d,r_d,0,dt,params.xhy.yaw);
    Z=smc_heave_xhy(zn,nu(3),10,0,0,dt,params.xhy.heave);
    tau_cmd=[X;0;Z;0;0;Nc];
    [ui,~]=thrust_allocation_xhy(tau_cmd,thr);

    ch_prev=c_hat;
    switch baseline
        case 'KIN'
            [c_hat,~]=kin_current_observer(c_hat,nu_meas,psi,u_d,params.kin,dt);
        case 'CFDLuenberger'
            [c_hat,~]=borhaug_current_observer(c_hat,nu_meas,tau_thr,psi,M_const,params.borhaug,dt);
        case 'EKF'
            [x_hat,P,ekf_aux]=ekf_current_estimator(x_hat,P,nu_meas,tau_thr,psi,M_const,params.ekf,dt);
            c_hat=ekf_aux.c_hat;
        case 'PCRCO'
            [c_hat,~]=eg_ucco_simple(c_hat,nu_meas,tau_thr,psi,M_const,params.ucco,dt);
        case 'NoClamp'
            [c_hat,~]=pcrco_no_clamp(c_hat,nu_meas,tau_thr,psi,M_const,params.ucco,dt);
        case 'NoPredCorr'
            [c_hat,~]=pcrco_no_predcorr(c_hat,nu_meas,tau_thr,psi,M_const,params.ucco,dt);
    end

    c_hist(i,:)=c_hat'; dc_hist(i)=norm(c_hat-ch_prev);
    x_state=rk4(@xhy,dt,x_state,ui,Vc,betaVc,0,opt_xhy);
    x_state(12)=ssa(x_state(12));
end

c_err=c_hist-repmat(c_true',N,1);
rmse=sqrt(mean(sum(c_err.^2,2)));
dc_max=max(dc_hist);
c_norm=sqrt(sum(c_hist.^2,2)); c_tn=sqrt(sum(c_true.^2));
overshoot=max(c_norm-c_tn);
bias_final=norm(c_hist(end,:)-c_true');
end

%% ==================== 辅助 ====================
function p=get_thr_params()
p.rho=1026; p.D_prop_main=0.08; p.D_prop_aux=0.06;
p.KT_main_fwd=0.1489; p.KT_main_rev=0.0506; p.KT_aux_fwd=0.53; p.KT_aux_rev=0.71;  % 0616非饱和
p.n_max=2500; p.x_vert_f=+0.344; p.x_vert_r=-0.293; p.x_side_f=+0.424; p.x_side_r=-0.376;
end
