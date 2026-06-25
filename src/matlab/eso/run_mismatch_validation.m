function results = run_mismatch_validation()
% 失配鲁棒性验证：多失配水平 × 低噪声 × 多观测器
% 验证原始Claim: PC-RCO在CFD模型失配下保持精度
% 2026-06-22

project_root = setup_paths();
params = get_params();

dt = 0.01; T_end = 100; N = round(T_end/dt) + 1;
Vc = 0.3; betaVc = deg2rad(45);
c_true = [Vc*cos(betaVc); Vc*sin(betaVc)];

% 测试参数
mismatches = [0, 10, 20, 30, 40, 50];
observers = {'KIN', 'CFDLuenberger', 'EKF', 'PCRCO'};
n_seeds = 5;

% 惯性矩阵
m=85.832; Ix=1.419; Iy=7.174; Iz=6.391;
MRB = diag([m,m,m,Ix,Iy,Iz]);
MA = diag([15.81, 124.73, 42.87, 0.014, 0.041, 0.123]);
M_const = MRB + MA;

% 推进器
thr.rho=1026; thr.D_prop_main=0.08; thr.D_prop_aux=0.06;
thr.KT_main_fwd=0.1489; thr.KT_main_rev=0.0506;
thr.KT_aux_fwd=0.53; thr.KT_aux_rev=0.71;  % 0616非饱和
thr.n_max=2500;
thr.x_vert_f=+0.344; thr.x_vert_r=-0.293;
thr.x_side_f=+0.424; thr.x_side_r=-0.376;

fprintf('===== 失配鲁棒性验证 =====\n');
fprintf('海流: Vc=%.1f m/s, βc=%.0f°\n', Vc, rad2deg(betaVc));
fprintf('噪声: low (σ=0.02 m/s DVL)\n');
fprintf('失配: %s %%\n', strjoin(cellstr(num2str(mismatches')), ', '));
fprintf('观测器: %s\n', strjoin(observers, ', '));
fprintf('Seeds: %d\n\n', n_seeds);

results = struct();
results.cfg.mismatches = mismatches;
results.cfg.observers = observers;
results.cfg.n_seeds = n_seeds;

for mi = 1:length(mismatches)
    mm = mismatches(mi);
    fprintf('--- 失配=%d%% ---\n', mm);

    for oi = 1:length(observers)
        obs_name = observers{oi};
        rmse_arr = zeros(1, n_seeds);
        dc_max_arr = zeros(1, n_seeds);
        tau90_arr = zeros(1, n_seeds);

        for s = 1:n_seeds
            rng(s);
            [rmse, dc_m, tau90] = run_one_mismatch_test(...
                obs_name, mm, s, params, M_const, thr, ...
                Vc, betaVc, c_true, T_end, dt);
            rmse_arr(s) = rmse;
            dc_max_arr(s) = dc_m;
            tau90_arr(s) = tau90;
        end

        key = sprintf('mm%d_%s', mm, obs_name);
        results.(key).rmse_mean = mean(rmse_arr);
        results.(key).rmse_std = std(rmse_arr);
        results.(key).dc_max_mean = mean(dc_max_arr);
        results.(key).tau90_mean = mean(tau90_arr);

        fprintf('  %-14s: RMSE=%.4f±%.4f  dc_max=%.4f  τ90=%.1fs\n', ...
            obs_name, mean(rmse_arr), std(rmse_arr), mean(dc_max_arr), mean(tau90_arr));
    end

    % 计算退化率 (vs 0% mismatch)
    if mi > 1
        fprintf('  退化率 (vs 0%%): ');
        for oi = 1:length(observers)
            obs_name = observers{oi};
            key0 = sprintf('mm%d_%s', 0, obs_name);
            key_m = sprintf('mm%d_%s', mm, obs_name);
            degradation = (results.(key_m).rmse_mean - results.(key0).rmse_mean) / results.(key0).rmse_mean * 100;
            fprintf('%s=%+.0f%%  ', obs_name, degradation);
        end
        fprintf('\n');
    end
end

%% ===== 汇总表 =====
fprintf('\n===== 失配鲁棒性汇总 =====\n');
fprintf('%-8s', '失配');
for oi = 1:length(observers)
    fprintf(' %-12s', observers{oi});
end
fprintf(' %-16s\n', 'EKF退化 vs PC-RCO');
fprintf('%s\n', repmat('-', 1, 80));

for mi = 1:length(mismatches)
    mm = mismatches(mi);
    fprintf('%-8s', sprintf('%d%%', mm));
    for oi = 1:length(observers)
        key = sprintf('mm%d_%s', mm, observers{oi});
        fprintf(' %-12.4f', results.(key).rmse_mean);
    end
    if mi > 1
        % EKF vs PC-RCO degradation
        key_ekf0 = sprintf('mm%d_%s', 0, 'EKF');
        key_pcrco0 = sprintf('mm%d_%s', 0, 'PCRCO');
        key_ekf = sprintf('mm%d_%s', mm, 'EKF');
        key_pcrco = sprintf('mm%d_%s', mm, 'PCRCO');
        d_ekf = (results.(key_ekf).rmse_mean - results.(key_ekf0).rmse_mean) / results.(key_ekf0).rmse_mean * 100;
        d_pcrco = (results.(key_pcrco).rmse_mean - results.(key_pcrco0).rmse_mean) / results.(key_pcrco0).rmse_mean * 100;
        fprintf(' EKF%+.0f%% vs PC%+.0f%% (差%.0f%%)', d_ekf, d_pcrco, d_ekf-d_pcrco);
    end
    fprintf('\n');
end

save(fullfile(project_root, 'results', 'mismatch_validation.mat'), 'results');
fprintf('\n结果已保存到 results/mismatch_validation.mat\n');
end

%% ==================== 单次测试 ====================
function [rmse, dc_max, tau90] = run_one_mismatch_test(...
    obs_name, mismatch_pct, seed, params, M_const, thr, ...
    Vc, betaVc, c_true, T_end, dt)

N = round(T_end/dt) + 1; rng(seed);

% 圆形轨迹
pts = traj(20, 50);
x_state = [1.0;0;0;0;0;0;300;0;10;0;0;pi/2];
opt_xhy = struct('mode','rpm','mismatch_pct',mismatch_pct,'mismatch_seed',seed);

psi_d=pi/2; r_d=0; u_d=1.0; ui=zeros(5,1);

% 初始化
c_hat=[0;0];  % 所有观测器共用的c_hat
switch obs_name
    case 'KIN', ;
    case 'CFDLuenberger', clear borhaug_current_observer;
    case 'EKF', x_hat=[]; P=[];
    case 'PCRCO', clear eg_ucco_simple;
end

c_hist=zeros(N,2); dc_hist=zeros(N,1);

for i=1:N
    nu=x_state(1:6); xn=x_state(7); yn=x_state(8); zn=x_state(9); psi=x_state(12);

    % 低噪声 (真实DVL水平)
    nu_meas=nu+0.02*randn(6,1);

    % Plant: 使用扰动CFD → opt_xhy含mismatch
    [~,~,M,~,~,~,tau_thr]=xhy(x_state,ui,Vc,betaVc,0,opt_xhy);

    % 制导+控制
    [psi_ref,~,~,~,~,~,~]=my_ALOS3D(xn,yn,zn,dt,pts,params.alos);
    [psi_d,r_d]=LOSobserver(psi_d,r_d,psi_ref,dt,params.alos.K_f);
    r_d=sat(r_d,0.5);
    X=smc_surge_xhy(nu(1),u_d,0,dt,params.xhy.surge);
    Nc=smc_yaw_xhy(psi,nu(6),psi_d,r_d,0,dt,params.xhy.yaw);
    Z=smc_heave_xhy(zn,nu(3),10,0,0,dt,params.xhy.heave);
    tau_cmd=[X;0;Z;0;0;Nc];
    [ui,~]=thrust_allocation_xhy(tau_cmd,thr);

    % Observer: 使用名义CFD (无mismatch)
    ch_prev=c_hat;
    switch obs_name
        case 'KIN'
            [c_hat,~]=kin_current_observer(c_hat,nu_meas,psi,u_d,params.kin,dt);
        case 'CFDLuenberger'
            [c_hat,~]=borhaug_current_observer(c_hat,nu_meas,tau_thr,psi,M_const,params.borhaug,dt);
        case 'EKF'
            [x_hat,P,ekf_aux]=ekf_current_estimator(x_hat,P,nu_meas,tau_thr,psi,M_const,params.ekf,dt);
            c_hat=ekf_aux.c_hat;
        case 'PCRCO'
            [c_hat,~]=eg_ucco_simple(c_hat,nu_meas,tau_thr,psi,M_const,params.ucco,dt);
    end

    c_hist(i,:)=c_hat';
    dc_hist(i)=norm(c_hat-ch_prev);

    % Plant更新 (扰动CFD)
    x_state=rk4(@xhy,dt,x_state,ui,Vc,betaVc,0,opt_xhy);
    x_state(12)=ssa(x_state(12));
end

c_err=c_hist-repmat(c_true',N,1);
rmse=sqrt(mean(sum(c_err.^2,2)));
dc_max=max(dc_hist);

% τ90
c_norm=sqrt(sum(c_hist.^2,2));
c_final=median(c_norm(round(N*0.8):end));
idx90=find(c_norm>=0.9*c_final,1,'first');
if isempty(idx90), idx90=N; end
tau90=idx90*dt;
end
