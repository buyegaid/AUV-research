%% 完整实验矩阵
% 3场景 × 4失配水平 × 5方法 × N种子
% 2026-06-04

function full_experiment()
addpath('./Lib','./guidance','./controller/xhy','./controller/remus','./model','./eso','./post','./traj');

scenarios = {'circle','straight','cstep'};
mismatches = [0, 10, 20, 30];
methods = [3,4,5,6,2];  % LESO, PIESO, EKF-tuned, UCCO, KIN
method_names = {'LESO','PIESO','EKF-tuned','UCCO','KIN'};
n_seeds = 2;  % 快速验证用2 seeds，正式实验用20+

h=0.01; T=100;
n_s = length(scenarios); n_mm = length(mismatches); n_m = length(methods);
all_results = cell(n_s, n_mm, n_m, n_seeds);

for si = 1:n_s
    scenario = scenarios{si};
    for mmi = 1:n_mm
        mm_pct = mismatches(mmi);
        for seed = 1:n_seeds
            fprintf('\n[%s mm=%d%% seed=%d] ', scenario, mm_pct, seed);
            for mi = 1:n_m
                method = methods(mi);
                res = run_one(scenario, method, mm_pct, seed, h, T);
                all_results{si,mmi,mi,seed} = res;
                fprintf('.');
            end
            fprintf(' OK');
        end
    end
end

% 汇总报告
fprintf('\n\n========== 完整实验结果 ==========\n');
for si = 1:n_s
    fprintf('\n--- %s ---\n', scenarios{si});
    fprintf('%-10s','Mismatch');
    for mi=1:n_m, fprintf('%12s',method_names{mi}); end
    fprintf('\n');
    for mmi=1:n_mm
        fprintf('%-10d',mismatches(mmi));
        for mi=1:n_m
            vals=zeros(n_seeds,1);
            for seed=1:n_seeds
                if ~isempty(all_results{si,mmi,mi,seed})
                    vals(seed)=all_results{si,mmi,mi,seed}.rmse_Vc;
                end
            end
            fprintf('%8.4f+-%.4f',mean(vals),std(vals));
        end
        fprintf('\n');
    end
end

% 保存结果
save('full_experiment_results.mat','all_results','scenarios','mismatches','method_names');
fprintf('\nResults saved to full_experiment_results.mat\n');
end

function res = run_one(scenario, ObserverMode, mismatch_pct, seed, h, T)
rng(20260604 + seed*1000);
params = get_params;
t = 0:h:T; N = length(t);

% 海流场景
switch scenario
    case 'circle'
        gm = struct('Vc_mean',0.3,'betaVc_mean',pi/4,'sigma_Vc',0.1,'tau_c',100);
        [Vs,bs,ws] = gauss_markov_current(2,t,gm);
        pts = traj(50,50);
    case 'straight'
        gm = struct('Vc_mean',0.3,'betaVc_mean',pi/4,'sigma_Vc',0.05,'tau_c',100);
        [Vs,bs,ws] = gauss_markov_current(2,t,gm);
        pts = line_traj([0;0;10],0,1000,1000);
    case 'cstep'
        Vs = 0.15*ones(1,N); Vs(round(N/2):end)=0.45;
        bs = pi/6*ones(1,N); bs(round(N/2):end)=pi/3;
        ws = zeros(1,N);
        pts = traj(50,50);
end

x=[1;0;0;0;0;0;0;0;10;0;0;0]; psi0=0;
Z=zeros(6,3); c_hat=[0;0];
if params.ekf.use_bias, x_ekf=[x(1:6);0;0;zeros(6,1)]; P_ekf=0.01*eye(14);
else, x_ekf=[x(1:6);0;0]; P_ekf=0.01*eye(8); end
Vc_kin=0; beta_kin=0; x_hat_kin=[0;0;10];
psi_d=0; r_d=0; theta_d=0; q_d=0; u_d=1; z_d=10; w_d=0;
ui=zeros(5,1); thr=get_thruster_params_fe();

% EKF tuning
params.ekf.Q_b = 0.005*(1+mismatch_pct/10);
params.ekf.Q_nu = 0.005*(1+mismatch_pct/10);

c_est_hist=zeros(N,2); c_true_hist=zeros(N,2);

for i=1:N
    u=x(1); v=x(2); w=x(3); r=x(6); xn=x(7); yn=x(8); zn=x(9);
    theta=x(11); psi=x(12);
    Vc=Vs(i); bc=bs(i); wc=ws(i);

    pert=ones(6,1);
    if mismatch_pct>0, pert=1+mismatch_pct/100*(2*rand(6,1)-1); end

    [~,~,M,C,D,g_vec,tau_thr]=xhy(x,ui,Vc,bc,wc);
    u_c_x=Vc*cos(bc-psi); u_c_y=Vc*sin(bc-psi);
    nu_r=x(1:6)-[u_c_x;u_c_y;wc;0;0;0];
    a_known=M\(tau_thr-C*nu_r-D*nu_r-g_vec);

    hat_d=zeros(6,1); Vc_est=0; beta_est=0;

    switch ObserverMode
        case 2  % KIN
            [Vc_kin,beta_kin,x_hat_kin,~]=kin_current_observer(Vc_kin,beta_kin,x_hat_kin,x(1:6),[xn;yn;zn],psi,theta,params.kin,h);
            Vc_est=Vc_kin; beta_est=beta_kin;
        case 3  % LESO
            [Z,~]=vec_leso_update_adv(Z,x(1:6),a_known,params.eso,h);
            hat_d=M*Z(:,3);
        case 4  % PIESO
            [Z,~]=vec_pieso_update(Z,x(1:6),a_known,params.pieso,h);
            hat_d=M*Z(:,3);
        case 5  % EKF-tuned
            [x_ekf,P_ekf,aux]=ekf_current_estimator(x_ekf,P_ekf,x(1:6),tau_thr,psi,M,params,h);
            Vc_est=norm(aux.c_hat); beta_est=atan2(aux.c_hat(2),aux.c_hat(1));
            hd=comp_curr_fe(Vc_est,beta_est,x(1:6),psi,tau_thr,M);
            hat_d=[0;0;0;0;0;hd(6)];
        case 6  % UCCO
            [c_hat,~]=eg_ucco_simple(c_hat,x(1:6),tau_thr,psi,M,params.ucco,h);
            Vc_est=norm(c_hat); beta_est=atan2(c_hat(2),c_hat(1));
            hd=comp_curr_fe(Vc_est,beta_est,x(1:6),psi,tau_thr,M);
            hat_d=[0;0;0;0;0;hd(6)];
    end

    [psi_ref,theta_ref,~,~,~,~,~]=my_ALOS3D(xn,yn,zn,h,pts,params.alos);
    [psi_d,r_d]=LOSobserver(psi_d,r_d,psi_ref,h,params.alos.K_f); r_d=sat(r_d,0.5);
    Z_cmd=smc_heave_xhy(zn,w,z_d,w_d,0,h,params.xhy.heave);
    X_cmd=smc_surge_xhy(u,u_d,0,h,params.xhy.surge)-hat_d(1);
    N_cmd=smc_yaw_xhy(psi,r,psi_d,r_d,0,h,params.xhy.yaw)-hat_d(6);
    [ui,~]=thrust_allocation_xhy([X_cmd;0;Z_cmd;0;0;N_cmd],thr);

    cN_t=Vc*cos(bc); cE_t=Vc*sin(bc);
    c_est_hist(i,:)=[Vc_est*cos(beta_est),Vc_est*sin(beta_est)];
    c_true_hist(i,:)=[cN_t,cE_t];

    [xdot,~,~,~,~,~,~]=xhy(x,ui,Vc,bc,wc);
    if mismatch_pct>0
        D_pert=D; for k=1:6, D_pert(k,k)=D(k,k)*pert(k); end
        xdot(1:6)=xdot(1:6)+M\((D-D_pert)*nu_r);
    end
    x=x+xdot*h; x(12)=ssa(x(12));
end

c_err=c_est_hist-c_true_hist;
res.rmse_Vc=sqrt(mean(c_err(:,1).^2+c_err(:,2).^2));
res.mae_Vc=mean(sqrt(c_err(:,1).^2+c_err(:,2).^2));
res.final_Vc_err=sqrt(c_err(end,1)^2+c_err(end,2)^2);
end

function hd=comp_curr_fe(Vc,b,nu,psi,tau,M)
if Vc<1e-6, hd=zeros(6,1); return; end
uc=Vc*cos(b-psi); vc=Vc*sin(b-psi); nc=[uc;vc;0;0;0;0];
nr=nu-nc;
[td,~]=xhy_drag_cfd(nr,M); [Cc,gc]=compute_cg_standalone(nr,psi,M);
Dnc=[nu(6)*vc;-nu(6)*uc;0;0;0;0];
ad=Dnc+M\(tau+td-Cc*nr-gc);
[td0,~]=xhy_drag_cfd(nu,M); [C0,g0]=compute_cg_standalone(nu,psi,M);
a0=M\(tau+td0-C0*nu-g0);
hd=M*(ad-a0);
end

function thr=get_thruster_params_fe()
thr.rho=1026; thr.D_prop_main=0.10; thr.D_prop_aux=0.06;
thr.KT_main_fwd=0.0293; thr.KT_main_rev=0.0201;
thr.KT_aux_fwd=0.327; thr.KT_aux_rev=0.327;
thr.n_max=2500; thr.x_vert_f=+0.344; thr.x_vert_r=-0.293;
thr.x_side_f=+0.424; thr.x_side_r=-0.376;
end
