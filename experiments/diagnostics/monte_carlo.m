%% Monte Carlo final validation
% 20 seeds × 2 scenarios × 2 mismatches × 4 methods
% 2026-06-04

function monte_carlo()
project_root = setup_paths();

scenarios = {'circle','straight'};
mismatches = [0, 20];
methods = [2,5,6];  % KIN, EKF-tuned, UCCO
method_names = {'KIN','EKF-tuned','UCCO'};
n_seeds = 10;

h=0.01; T=100; t=0:h:T; N=length(t);

fprintf('Monte Carlo: %d seeds × %d scenarios × %d mismatches × %d methods = %d runs\n', ...
    n_seeds, length(scenarios), length(mismatches), length(methods), ...
    n_seeds*length(scenarios)*length(mismatches)*length(methods));

all_rmse = zeros(n_seeds, length(scenarios), length(mismatches), length(methods));

for si=1:length(scenarios)
    sc=scenarios{si};
    for mmi=1:length(mismatches)
        mm=mismatches(mmi);
        for seed=1:n_seeds
            rng(20260604+seed*1000);
            for mi=1:length(methods)
                res=run_mc(sc,methods(mi),mm,seed,h,T,N,t);
                all_rmse(seed,si,mmi,mi)=res;
            end
        end
        fprintf('[%s mm=%d%%] ',sc,mm);
        for mi=1:length(methods)
            vals=all_rmse(:,si,mmi,mi);
            fprintf('%s=%.4f+-%.4f  ',method_names{mi},mean(vals),std(vals)/sqrt(n_seeds));
        end
        fprintf('\n');
    end
end

fprintf('\n========== Monte Carlo Summary ==========\n');
for si=1:length(scenarios)
    fprintf('\n--- %s ---\n',scenarios{si});
    fprintf('%-12s %10s %10s %10s\n','Mismatch','KIN','EKF-tuned','UCCO');
    for mmi=1:length(mismatches)
        fprintf('%-12d',mismatches(mmi));
        for mi=1:length(methods)
            vals=all_rmse(:,si,mmi,mi);
            fprintf('%8.4f+-%.4f',mean(vals),std(vals));
        end
        fprintf('\n');
    end
end

save('monte_carlo_results.mat','all_rmse','scenarios','mismatches','method_names');
fprintf('\nSaved.\n');
end

function rmse_Vc = run_mc(scenario, method, mm_pct, seed, h, T, N, t)
params=get_params;
gm=struct('Vc_mean',0.3,'betaVc_mean',pi/4,'sigma_Vc',0.1,'tau_c',100);
[Vs,bs,ws]=gauss_markov_current(2,t,gm);

if strcmp(scenario,'circle'), pts=traj(50,50);
else, pts=line_traj([0;0;10],0,1000,1000); end

x=[1;0;0;0;0;0;0;0;10;0;0;0];
Z=zeros(6,3); c_hat=[0;0];
x_ekf=[x(1:6);0;0;zeros(6,1)]; P_ekf=0.01*eye(14);
Vc_kin=0; beta_kin=0; x_hat_kin=[0;0;10];
psi_d=0; r_d=0; u_d=1; z_d=10; w_d=0; ui=zeros(5,1);
thr=get_thruster_params_mc();

% EKF tuning
params.ekf.Q_b=0.005*(1+mm_pct/10);
params.ekf.Q_nu=0.005*(1+mm_pct/10);

% Low sensor noise
noise_vel=0.02; noise_yaw=deg2rad(0.5);

c_est=zeros(N,2); c_true=zeros(N,2);

for i=1:N
    u=x(1); v=x(2); w=x(3); r=x(6); xn=x(7); yn=x(8); zn=x(9);
    theta=x(11); psi=x(12);
    Vc=Vs(i); bc=bs(i); wc=ws(i);

    pert=ones(6,1);
    if mm_pct>0, pert=1+mm_pct/100*(2*rand(6,1)-1); end

    nu_meas=x(1:6)+noise_vel*randn(6,1);
    psi_meas=psi+noise_yaw*randn;

    [~,~,M,C,D,g_vec,tau_thr]=xhy(x,ui,Vc,bc,wc);
    nu_r=x(1:6)-[Vc*cos(bc-psi);Vc*sin(bc-psi);wc;0;0;0];
    a_known=M\(tau_thr-C*nu_r-D*nu_r-g_vec);

    hat_d=zeros(6,1); Vc_est_=0; beta_est_=0;

    switch method
        case 2  % KIN
            [Vc_kin,beta_kin,x_hat_kin,~]=kin_current_observer(Vc_kin,beta_kin,x_hat_kin,nu_meas,[xn;yn;zn],psi_meas,theta,params.kin,h);
            Vc_est_=Vc_kin; beta_est_=beta_kin;
        case 5  % EKF-tuned
            [x_ekf,P_ekf,aux]=ekf_current_estimator(x_ekf,P_ekf,nu_meas,tau_thr,psi_meas,M,params,h);
            Vc_est_=norm(aux.c_hat); beta_est_=atan2(aux.c_hat(2),aux.c_hat(1));
            hd=comp_curr_mc(Vc_est_,beta_est_,x(1:6),psi,tau_thr,M);
            hat_d=[0;0;0;0;0;hd(6)];
        case 6  % UCCO
            [c_hat,~]=eg_ucco_simple(c_hat,nu_meas,tau_thr,psi_meas,M,params.ucco,h);
            Vc_est_=norm(c_hat); beta_est_=atan2(c_hat(2),c_hat(1));
            hd=comp_curr_mc(Vc_est_,beta_est_,x(1:6),psi,tau_thr,M);
            hat_d=[0;0;0;0;0;hd(6)];
    end

    [psi_ref,theta_ref,~,~,~,~,~]=my_ALOS3D(xn,yn,zn,h,pts,params.alos);
    [psi_d,r_d]=LOSobserver(psi_d,r_d,psi_ref,h,params.alos.K_f); r_d=sat(r_d,0.5);
    Z_cmd=smc_heave_xhy(zn,w,z_d,w_d,0,h,params.xhy.heave);
    N_cmd=smc_yaw_xhy(psi,r,psi_d,r_d,0,h,params.xhy.yaw)-hat_d(6);
    [ui,~]=thrust_allocation_xhy([smc_surge_xhy(u,u_d,0,h,params.xhy.surge);0;Z_cmd;0;0;N_cmd],thr);

    cN_t=Vc*cos(bc); cE_t=Vc*sin(bc);
    c_est(i,:)=[Vc_est_*cos(beta_est_),Vc_est_*sin(beta_est_)];
    c_true(i,:)=[cN_t,cE_t];

    [xdot,~,~,~,~,~,~]=xhy(x,ui,Vc,bc,wc);
    if mm_pct>0, D_pert=D; for k=1:6, D_pert(k,k)=D(k,k)*pert(k); end
        xdot(1:6)=xdot(1:6)+M\((D-D_pert)*nu_r); end
    x=x+xdot*h; x(12)=ssa(x(12));
end

c_err=c_est-c_true;
rmse_Vc=sqrt(mean(c_err(:,1).^2+c_err(:,2).^2));
end

function hd=comp_curr_mc(Vc,b,nu,psi,tau,M)
if Vc<1e-6, hd=zeros(6,1); return; end
uc=Vc*cos(b-psi); vc=Vc*sin(b-psi); nc=[uc;vc;0;0;0;0]; nr=nu-nc;
[td,~]=xhy_drag_cfd(nr,M); [Cc,gc]=compute_cg_standalone(nr,psi,M);
Dnc=[nu(6)*vc;-nu(6)*uc;0;0;0;0]; ad=Dnc+M\(tau+td-Cc*nr-gc);
[td0,~]=xhy_drag_cfd(nu,M); [C0,g0]=compute_cg_standalone(nu,psi,M);
a0=M\(tau+td0-C0*nu-g0); hd=M*(ad-a0);
end

function thr=get_thruster_params_mc()
thr.rho=1026; thr.D_prop_main=0.10; thr.D_prop_aux=0.06;
thr.KT_main_fwd=0.0293; thr.KT_main_rev=0.0201;
thr.KT_aux_fwd=0.327; thr.KT_aux_rev=0.327;
thr.n_max=2500; thr.x_vert_f=+0.344; thr.x_vert_r=-0.293;
thr.x_side_f=+0.424; thr.x_side_r=-0.376;
end
