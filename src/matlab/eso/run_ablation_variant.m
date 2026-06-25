function result = run_ablation_variant(cfg)
% 消融变体实验 — 支持7个变体
% variants: KIN, CFDLuenberger, EKF, PC-RCO, NoPredCorr, NoGM, NoClamp
% 2026-06-11

variant_name = cfg.observers{1};
project_root = setup_paths();
params = get_params();

dt = 0.01; T_end = cfg.T_end; N = T_end/dt + 1;
rng(cfg.seed);

%% 海流+轨迹（同run_observer_comparison）
Vc = 0.3; betaVc = deg2rad(45); wc = 0;
c_true = [Vc*cos(betaVc); Vc*sin(betaVc)];
if strcmp(cfg.scenario, 'circle')
    pts = traj(20, 50); u0=1; psi0=pi/2; xn0=300; yn0=0;
elseif strcmp(cfg.scenario, 'straight')
    pts.pos.x=(0:20:1000)'; pts.pos.y=zeros(51,1); pts.pos.z=10*ones(51,1);
    u0=1; psi0=0; xn0=0; yn0=0;
else
    pts = traj(20, 50); u0=1; psi0=pi/2; xn0=300; yn0=0;
end
x_state = [u0;0;0;0;0;0;xn0;yn0;10;0;0;psi0];
M_const = compute_M_constant();
thr_params = get_thr_params();
opt_xhy = struct('mode','rpm','mismatch_pct',cfg.mismatch_pct,'mismatch_seed',cfg.seed);

%% 初始化
psi_d=psi0; r_d=0; u_d=1.0; ui=zeros(5,1);
switch variant_name
    case 'KIN',          obs.c_hat=[0;0];
    case 'CFDLuenberger', obs.c_hat=[0;0]; clear borhaug_current_observer;
    case 'EKF',          obs.x_hat=[]; obs.P=[];
    case 'PCRCO',       obs.c_hat=[0;0]; clear eg_ucco_simple;
	    case 'NoPredCorr',   obs.c_hat=[0;0]; pcrco_no_predcorr([],[],[],[],[],[],[],true);
	    case 'NoGM',         obs.c_hat=[0;0]; pcrco_no_gm([],[],[],[],[],[],[],true);
	    case 'NoClamp',      obs.c_hat=[0;0]; pcrco_no_clamp([],[],[],[],[],[],[],true);
end

c_hist = zeros(N,2);

%% 主循环
for i = 1:N
    nu = x_state(1:6); xn = x_state(7); yn = x_state(8); zn = x_state(9);
    psi = x_state(12);
    if strcmp(cfg.scenario,'step') && (i-1)*dt >= 50
        Vc_now=0.45; betaVc_now=deg2rad(60);
    else, Vc_now=Vc; betaVc_now=betaVc;
    end

    nu_meas = nu;
    if strcmp(cfg.noise_level,'low'), nu_meas(1:2)=nu(1:2)+0.02*randn(2,1); end
    if strcmp(cfg.noise_level,'high'), nu_meas(1:2)=nu(1:2)+0.10*randn(2,1); end

    [~,~,M,~,~,~,tau_thr] = xhy(x_state,ui,Vc_now,betaVc_now,wc,opt_xhy);
    [psi_ref,~,~,~,~,~,~] = my_ALOS3D(xn,yn,zn,dt,pts,params.alos);
    [psi_d,r_d] = LOSobserver(psi_d,r_d,psi_ref,dt,params.alos.K_f);
    r_d = sat(r_d,0.5);
    X_cmd=smc_surge_xhy(nu(1),u_d,0,dt,params.xhy.surge);
    N_cmd=smc_yaw_xhy(psi,nu(6),psi_d,r_d,0,dt,params.xhy.yaw);
    Z_cmd=smc_heave_xhy(zn,nu(3),10,0,0,dt,params.xhy.heave);
    tau_cmd=[X_cmd;0;Z_cmd;0;0;N_cmd];
    [ui,~]=thrust_allocation_xhy(tau_cmd,thr_params);

    % 根据变体调用不同观测器
    switch variant_name
        case 'KIN'
            [obs.c_hat,~]=kin_current_observer(obs.c_hat,nu_meas,psi,u_d,params.kin,dt);
        case 'CFDLuenberger'
            [obs.c_hat,~]=borhaug_current_observer(obs.c_hat,nu_meas,tau_thr,psi,M_const,params.borhaug,dt);
        case 'EKF'
            [obs.x_hat,obs.P,ekf_aux]=ekf_current_estimator(obs.x_hat,obs.P,nu_meas,tau_thr,psi,M_const,params.ekf,dt);
            obs.c_hat=ekf_aux.c_hat;
        case 'PCRCO'
            [obs.c_hat,~]=eg_ucco_simple(obs.c_hat,nu_meas,tau_thr,psi,M_const,params.ucco,dt);
	        case 'NoPredCorr'
	            [obs.c_hat,~]=pcrco_no_predcorr(obs.c_hat,nu_meas,tau_thr,psi,M_const,params.ucco,dt);
	        case 'NoGM'
	            [obs.c_hat,~]=pcrco_no_gm(obs.c_hat,nu_meas,tau_thr,psi,M_const,params.ucco,dt);
	        case 'NoClamp'
	            [obs.c_hat,~]=pcrco_no_clamp(obs.c_hat,nu_meas,tau_thr,psi,M_const,params.ucco,dt);
    end
    c_hist(i,:) = obs.c_hat';

    x_state = rk4(@xhy,dt,x_state,ui,Vc_now,betaVc_now,wc,opt_xhy);
    x_state(12)=ssa(x_state(12));
end

burn = round(N*0.2); valid = burn:N;
c_err = c_hist(valid,:) - repmat(c_true',length(valid),1);
rmse = sqrt(mean(sum(c_err.^2,2)));
result.UCCO.RMSE = rmse;
end

%% ===== 辅助函数 =====
function M=compute_M_constant()
m=85.832; Ix=0.553864787+0.865274; Iy=2.162341935+5.011187; Iz=1.849137+4.541468;
MRB=diag([m,m,m,Ix,Iy,Iz]); MA=diag([15.81,124.73,42.87,0.014,0.041,0.123]); M=MRB+MA;
end

function p=get_thr_params()
p.rho=1026; p.D_prop_main=0.08; p.D_prop_aux=0.06;
p.KT_main_fwd=0.1489; p.KT_main_rev=0.0506; p.KT_aux_fwd=0.53; p.KT_aux_rev=0.71;  % 0616非饱和
p.n_max=2500; p.x_vert_f=+0.344; p.x_vert_r=-0.293; p.x_side_f=+0.424; p.x_side_r=-0.376;
end
