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
    case 'NoPredCorr',   obs.c_hat=[0;0];
    case 'NoGM',         obs.c_hat=[0;0]; clear eg_ucco_simple;
    case 'NoClamp',      obs.c_hat=[0;0]; clear eg_ucco_simple;
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
            [obs.c_hat,~]=ucco_no_predcorr(obs.c_hat,nu_meas,tau_thr,psi,M_const,params.ucco,dt);
        case 'NoGM'
            [obs.c_hat,~]=ucco_no_gm(obs.c_hat,nu_meas,tau_thr,psi,M_const,params.ucco,dt);
        case 'NoClamp'
            [obs.c_hat,~]=ucco_no_clamp(obs.c_hat,nu_meas,tau_thr,psi,M_const,params.ucco,dt);
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

%% ===== 消融变体实现 =====

% NoPredCorr: 用加速度残差替代速度预测-校正
function [c_hat, aux] = ucco_no_predcorr(c_hat, nu_meas, tau, psi, M, params, dt)
persistent nu_prev is_init a_filt
if isempty(is_init), is_init=true; a_filt=zeros(6,1); end
if isempty(nu_prev), nu_prev=nu_meas; end
if isempty(c_hat), c_hat=[0;0]; end

u_c=c_hat(1)*cos(psi)+c_hat(2)*sin(psi); v_c=-c_hat(1)*sin(psi)+c_hat(2)*cos(psi);
nu_c=[u_c;v_c;0;0;0;0]; nu_r=nu_prev-nu_c;
[tau_drag,~]=xhy_drag_cfd(nu_r);
[C_nu,g_nu]=compute_cg_standalone(nu_r,psi,M);
r=nu_prev(6); Dnu_c=[r*v_c;-r*u_c;0;0;0;0];
a_model=Dnu_c+M\(tau+tau_drag-C_nu*nu_r-g_nu);

% 关键改变: 用加速度残差 e_acc = a_meas_filt - a_model
a_raw=(nu_meas-nu_prev)/dt;
alpha=params.accel_lpf_alpha;
a_filt=alpha*a_raw+(1-alpha)*a_filt;
e_acc=a_filt(1:2)-a_model(1:2);

% 数值灵敏度（加速度层面）
delta=params.sens_pert; Phi=zeros(2,2);
for j=1:2
    cp=c_hat; cp(j)=cp(j)+delta;
    u_c_p=cp(1)*cos(psi)+cp(2)*sin(psi); v_c_p=-cp(1)*sin(psi)+cp(2)*cos(psi);
    nu_r_p=nu_prev-[u_c_p;v_c_p;0;0;0;0];
    [td_p,~]=xhy_drag_cfd(nu_r_p);
    [Cp,gp]=compute_cg_standalone(nu_r_p,psi,M);
    Dnc_p=[nu_prev(6)*v_c_p;-nu_prev(6)*u_c_p;0;0;0;0];
    a_p=Dnc_p+M\(tau+td_p-Cp*nu_r_p-gp);
    Phi(:,j)=(a_p(1:2)-a_model(1:2))/delta;
end

Wc=Phi'*Phi; lambda_min=min(eig(Wc));
gamma_eff=params.K_obs*100;
gain=gamma_eff/(1+lambda_min*1e6);
dc=gain*Phi'*e_acc;
dc=max(-params.max_dc,min(params.max_dc,dc));
c_hat=c_hat+dc;
alpha_gm=exp(-dt/params.tau_c);
c_hat=alpha_gm*c_hat+(1-alpha_gm)*params.c_mean;
c_hat=max(-params.c_max,min(params.c_max,c_hat));
nu_prev=nu_meas;
aux.c_hat=c_hat;
end

% NoGM: tau_c→∞, 无GM衰减
function [c_hat, aux] = ucco_no_gm(c_hat, nu_meas, tau, psi, M, params, dt)
persistent nu_prev is_init
if isempty(is_init), is_init=true; end
if isempty(nu_prev), nu_prev=nu_meas; end
if isempty(c_hat), c_hat=[0;0]; end

u_c=c_hat(1)*cos(psi)+c_hat(2)*sin(psi); v_c=-c_hat(1)*sin(psi)+c_hat(2)*cos(psi);
nu_c=[u_c;v_c;0;0;0;0]; nu_r=nu_prev-nu_c;
[tau_drag,~]=xhy_drag_cfd(nu_r);
[C_nu,g_nu]=compute_cg_standalone(nu_r,psi,M);
r=nu_prev(6); Dnu_c=[r*v_c;-r*u_c;0;0;0;0];
a_model=Dnu_c+M\(tau+tau_drag-C_nu*nu_r-g_nu);
nu_pred=nu_prev+a_model*dt;
e_vel=nu_meas(1:2)-nu_pred(1:2);

delta=params.sens_pert; Phi=zeros(2,2);
for j=1:2
    cp=c_hat; cp(j)=cp(j)+delta;
    u_c_p=cp(1)*cos(psi)+cp(2)*sin(psi); v_c_p=-cp(1)*sin(psi)+cp(2)*cos(psi);
    nu_r_p=nu_prev-[u_c_p;v_c_p;0;0;0;0];
    [td_p,~]=xhy_drag_cfd(nu_r_p);
    [Cp,gp]=compute_cg_standalone(nu_r_p,psi,M);
    Dnc_p=[nu_prev(6)*v_c_p;-nu_prev(6)*u_c_p;0;0;0;0];
    a_p=Dnc_p+M\(tau+td_p-Cp*nu_r_p-gp);
    nu_p=nu_prev+a_p*dt;
    Phi(:,j)=(nu_p(1:2)-nu_pred(1:2))/delta;
end

Wc=Phi'*Phi; lambda_min=min(eig(Wc));
gamma_eff=params.K_obs*100;
gain=gamma_eff/(1+lambda_min*1e6);
dc=gain*Phi'*e_vel;
dc=max(-params.max_dc,min(params.max_dc,dc));
c_hat=c_hat+dc;
% 关键改变: 无GM衰减
c_hat=max(-params.c_max,min(params.c_max,c_hat));
nu_prev=nu_meas;
aux.c_hat=c_hat;
end

% NoClamp: Δc_max→∞
function [c_hat, aux] = ucco_no_clamp(c_hat, nu_meas, tau, psi, M, params, dt)
persistent nu_prev is_init
if isempty(is_init), is_init=true; end
if isempty(nu_prev), nu_prev=nu_meas; end
if isempty(c_hat), c_hat=[0;0]; end

u_c=c_hat(1)*cos(psi)+c_hat(2)*sin(psi); v_c=-c_hat(1)*sin(psi)+c_hat(2)*cos(psi);
nu_c=[u_c;v_c;0;0;0;0]; nu_r=nu_prev-nu_c;
[tau_drag,~]=xhy_drag_cfd(nu_r);
[C_nu,g_nu]=compute_cg_standalone(nu_r,psi,M);
r=nu_prev(6); Dnu_c=[r*v_c;-r*u_c;0;0;0;0];
a_model=Dnu_c+M\(tau+tau_drag-C_nu*nu_r-g_nu);
nu_pred=nu_prev+a_model*dt;
e_vel=nu_meas(1:2)-nu_pred(1:2);

delta=params.sens_pert; Phi=zeros(2,2);
for j=1:2
    cp=c_hat; cp(j)=cp(j)+delta;
    u_c_p=cp(1)*cos(psi)+cp(2)*sin(psi); v_c_p=-cp(1)*sin(psi)+cp(2)*cos(psi);
    nu_r_p=nu_prev-[u_c_p;v_c_p;0;0;0;0];
    [td_p,~]=xhy_drag_cfd(nu_r_p);
    [Cp,gp]=compute_cg_standalone(nu_r_p,psi,M);
    Dnc_p=[nu_prev(6)*v_c_p;-nu_prev(6)*u_c_p;0;0;0;0];
    a_p=Dnc_p+M\(tau+td_p-Cp*nu_r_p-gp);
    nu_p=nu_prev+a_p*dt;
    Phi(:,j)=(nu_p(1:2)-nu_pred(1:2))/delta;
end

Wc=Phi'*Phi; lambda_min=min(eig(Wc));
gamma_eff=params.K_obs*100;
gain=gamma_eff/(1+lambda_min*1e6);
dc=gain*Phi'*e_vel;
% 关键改变: 无限幅
c_hat=c_hat+dc;
alpha_gm=exp(-dt/params.tau_c);
c_hat=alpha_gm*c_hat+(1-alpha_gm)*params.c_mean;
c_hat=max(-params.c_max,min(params.c_max,c_hat));
nu_prev=nu_meas;
aux.c_hat=c_hat;
end

%% 辅助
function M=compute_M_constant()
m=85.832; Ix=0.553864787+0.865274; Iy=2.162341935+5.011187; Iz=1.849137+4.541468;
MRB=diag([m,m,m,Ix,Iy,Iz]); MA=diag([15.81,124.73,42.87,0.014,0.041,0.123]); M=MRB+MA;
end
function p=get_thr_params()
p.rho=1026; p.D_prop_main=0.10; p.D_prop_aux=0.06;
p.KT_main_fwd=0.0293; p.KT_main_rev=0.0201; p.KT_aux_fwd=0.327; p.KT_aux_rev=0.327;
p.n_max=2500; p.x_vert_f=+0.344; p.x_vert_r=-0.293; p.x_side_f=+0.424; p.x_side_r=-0.376;
end
