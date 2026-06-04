%% Mismatch sweep + EKF tuning
% 扫描 0%-30% CF阻力系数扰动，验证鲁棒性
% 2026-06-04

function mismatch_sweep()
addpath('./Lib','./guidance','./controller/xhy','./controller/remus','./model','./eso','./post','./traj');

method_names = {'LESO','PIESO','EKF-nom','EKF-tuned','UCCO'};
mismatch_levels = [0, 10, 20, 30];
n_m = length(mismatch_levels);
results = cell(n_m, 1);

h=0.01; T=100; t=0:h:T; N=length(t);
gm_params=struct('Vc_mean',0.3,'betaVc_mean',pi/4,'sigma_Vc',0.1,'tau_c',100);
[Vc_seq,beta_seq,wc_seq]=gauss_markov_current(2,t,gm_params);
thr=get_thruster_params_ms();

fprintf('Mismatch sweep: %d levels x %d methods\n', n_m, length(method_names));

for mm = 1:n_m
    mismatch_pct = mismatch_levels(mm);
    fprintf('\n=== Mismatch %d%% ===\n', mismatch_pct);
    res_mm = cell(5,1);

    for m = 1:5
        params=get_params;
        % EKF tuning: 增大Q_b让EKF能吸收模型失配
        if m==4  % EKF-tuned
            params.ekf.Q_b = 0.005 * (1 + mismatch_pct/10);  % 自适应Q_b
            params.ekf.Q_nu = 0.005 * (1 + mismatch_pct/10);
        end

        x=[1;0;0;0;0;0;0;0;10;0;0;0]; psi0=0;
        Z=zeros(6,3); c_hat=[0;0];
        if params.ekf.use_bias, x_ekf=[x(1:6);0;0;zeros(6,1)]; P_ekf=0.01*eye(14);
        else, x_ekf=[x(1:6);0;0]; P_ekf=0.01*eye(8); end

        psi_d=0; r_d=0; theta_d=0; q_d=0; u_d=1; z_d=10; w_d=0;
        ui=zeros(5,1); pts=traj(50,50);

        c_est_hist=zeros(N,2); c_true_hist=zeros(N,2);

        for i=1:N
            u=x(1); v=x(2); w=x(3); r=x(6); xn=x(7); yn=x(8); zn=x(9);
            theta=x(11); psi=x(12);
            Vc=Vc_seq(i); beta_c=beta_seq(i); wc=wc_seq(i);

            % 模型失配: 扰动CFD阻力
            if mismatch_pct>0, pert=1+mismatch_pct/100*(2*rand(6,1)-1);
            else, pert=ones(6,1); end

            [~,~,M,C,D,g_vec,tau_thr]=xhy(x,ui,Vc,beta_c,wc);
            u_c_x=Vc*cos(beta_c-psi); u_c_y=Vc*sin(beta_c-psi);
            nu_r=x(1:6)-[u_c_x;u_c_y;wc;0;0;0];
            a_known=M\(tau_thr-C*nu_r-D*nu_r-g_vec);

            hat_d=zeros(6,1); Vc_est=0; beta_est=0;

            switch m
                case 1  % LESO
                    [Z,~]=vec_leso_update_adv(Z,x(1:6),a_known,params.eso,h);
                    hat_d=M*Z(:,3);
                case 2  % PIESO
                    [Z,~]=vec_pieso_update(Z,x(1:6),a_known,params.pieso,h);
                    hat_d=M*Z(:,3);
                case {3,4}  % EKF
                    [x_ekf,P_ekf,aux]=ekf_current_estimator(x_ekf,P_ekf,x(1:6),tau_thr,psi,M,params,h);
                    Vc_est=norm(aux.c_hat); beta_est=atan2(aux.c_hat(2),aux.c_hat(1));
                    hd=comp_curr_ms(Vc_est,beta_est,x(1:6),psi,tau_thr,M);
                    hat_d=[0;0;0;0;0;hd(6)];
                case 5  % UCCO
                    [c_hat,~]=eg_ucco_simple(c_hat,x(1:6),tau_thr,psi,M,params.ucco,h);
                    Vc_est=norm(c_hat); beta_est=atan2(c_hat(2),c_hat(1));
                    hd=comp_curr_ms(Vc_est,beta_est,x(1:6),psi,tau_thr,M);
                    hat_d=[0;0;0;0;0;hd(6)];
            end

            [psi_ref,theta_ref,~,~,~,~,~]=my_ALOS3D(xn,yn,zn,h,pts,params.alos);
            [psi_d,r_d]=LOSobserver(psi_d,r_d,psi_ref,h,params.alos.K_f); r_d=sat(r_d,0.5);
            Z_cmd=smc_heave_xhy(zn,w,z_d,w_d,0,h,params.xhy.heave);
            X_cmd=smc_surge_xhy(u,u_d,0,h,params.xhy.surge)-hat_d(1);
            N_cmd=smc_yaw_xhy(psi,r,psi_d,r_d,0,h,params.xhy.yaw)-hat_d(6);
            [ui,~]=thrust_allocation_xhy([X_cmd;0;Z_cmd;0;0;N_cmd],thr);

            cN_t=Vc*cos(beta_c); cE_t=Vc*sin(beta_c);
            c_est_hist(i,:)=[Vc_est*cos(beta_est),Vc_est*sin(beta_est)];
            c_true_hist(i,:)=[cN_t,cE_t];

            [xdot,~,~,~,~,~,~]=xhy(x,ui,Vc,beta_c,wc);
            if mismatch_pct>0
                D_pert=D; for k=1:6, D_pert(k,k)=D(k,k)*pert(k); end
                xdot(1:6)=xdot(1:6)+M\((D-D_pert)*nu_r);
            end
            x=x+xdot*h; x(12)=ssa(x(12));
        end

        c_err=c_est_hist-c_true_hist;
        res_mm{m}.rmse_Vc=sqrt(mean(c_err(:,1).^2+c_err(:,2).^2));
        res_mm{m}.name=method_names{m};
    end
    results{mm}=res_mm;
    fprintf('  ');
    for m=1:5, fprintf('%s=%.4f  ',res_mm{m}.name,res_mm{m}.rmse_Vc); end
    fprintf('\n');
end

% Summary
fprintf('\n========== Mismatch Sweep Summary ==========\n');
fprintf('%-10s','Mismatch%');
for m=1:5, fprintf('%12s',method_names{m}); end
fprintf('\n');
for mm=1:n_m
    fprintf('%-10d',mismatch_levels(mm));
    for m=1:5, fprintf('%12.4f',results{mm}{m}.rmse_Vc); end
    fprintf('\n');
end

% Degradation rates
fprintf('\n=== Degradation (%% from nominal) ===\n');
fprintf('%-10s','Mismatch%');
for m=1:5, fprintf('%12s',method_names{m}); end
fprintf('\n');
for mm=1:n_m
    fprintf('%-10d',mismatch_levels(mm));
    for m=1:5
        base=results{1}{m}.rmse_Vc;
        deg=(results{mm}{m}.rmse_Vc-base)/base*100;
        fprintf('%11.1f%%',deg);
    end
    fprintf('\n');
end
end

function hd=comp_curr_ms(Vc,b,nu,psi,tau,M)
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

function thr=get_thruster_params_ms()
thr.rho=1026; thr.D_prop_main=0.10; thr.D_prop_aux=0.06;
thr.KT_main_fwd=0.0293; thr.KT_main_rev=0.0201;
thr.KT_aux_fwd=0.327; thr.KT_aux_rev=0.327;
thr.n_max=2500; thr.x_vert_f=+0.344; thr.x_vert_r=-0.293;
thr.x_side_f=+0.424; thr.x_side_r=-0.376;
end
