%% 批处理对比脚本 — 独立运行各方法避免嵌套函数复杂性
% 用法: batch_compare
% 2026-06-04

function batch_compare(useMismatch, mismatch_pct, useNoise)
if nargin<1, useMismatch=0; end
if nargin<2, mismatch_pct=10; end
if nargin<3, useNoise=0; end
addpath('./Lib','./guidance','./controller/xhy','./controller/remus','./model','./eso','./post','./traj');

method_names = {'SMC','KIN','LESO','PIESO','EKF','UCCO'};
results = cell(6,1);
params = get_params;

h = 0.01; T = 100; t = 0:h:T; N = length(t);
gm_params = struct('Vc_mean',0.3,'betaVc_mean',pi/4,'sigma_Vc',0.1,'tau_c',100);
[Vc_seq,beta_seq,wc_seq] = gauss_markov_current(2, t, gm_params);
thr = get_thruster_params_batch();
pts = traj(50,50);
xn=0; yn=0; zn=10; psi0=0;

for m = 1:6
    fprintf('\n===== %s =====\n', method_names{m});
    x = [1;0;0;0;0;0;xn;yn;zn;0;0;psi0];
    Z = zeros(6,3); c_hat=[0;0]; nu_obs=x(1:6);
    if params.ekf.use_bias
        x_ekf=[x(1:6);0;0;zeros(6,1)]; P_ekf=0.01*eye(14);
    else
        x_ekf=[x(1:6);0;0]; P_ekf=0.01*eye(8);
    end
    Vc_kin=0; beta_kin=0; x_hat_kin=[xn;yn;zn];
    psi_d=psi0; r_d=0; theta_d=0; q_d=0; u_d=1; u_d_dot=0; z_d=zn; w_d=0; w_d_dot=0;
    ui=zeros(5,1);

    hist_Vc=zeros(N,1); hist_beta=zeros(N,1); hist_psi=zeros(N,1); hist_psi_err=zeros(N,1);
    hist_c_est=zeros(N,2); hist_c_true=zeros(N,2); hist_x=zeros(N,12);

    for i = 1:N
        u=x(1); v=x(2); w=x(3); r=x(6); xn_i=x(7); yn_i=x(8); zn_i=x(9); theta=x(11); psi=x(12);
        Vc=Vc_seq(i); beta_c=beta_seq(i); wc=wc_seq(i);

        % 模型失配: 扰动CFD阻力系数
        if useMismatch
            pert = 1 + mismatch_pct/100 * (2*rand(6,1)-1);  % ±mismatch_pct% 随机扰动
        else
            pert = ones(6,1);
        end

        [~,~,M,C,D,g_vec,tau_thr] = xhy(x, ui, Vc, beta_c, wc);
        % 应用阻力系数扰动（通过缩放D矩阵对角元模拟）
        if useMismatch
            D_pert = D;
            for k=1:6, D_pert(k,k)=D(k,k)*pert(k); end
        else
            D_pert = D;
        end

        u_c_x=Vc*cos(beta_c-psi); u_c_y=Vc*sin(beta_c-psi);
        nu_c_true=[u_c_x;u_c_y;wc;0;0;0]; nu_r=x(1:6)-nu_c_true;
        a_known = M\(tau_thr - C*nu_r - D*nu_r - g_vec);  % 名义D（ESO用）

        hat_d=zeros(6,1); Vc_est=0; beta_est=0;

        switch m
            case 1  % SMC
            case 2  % KIN
                [Vc_kin,beta_kin,x_hat_kin,~]=kin_current_observer(Vc_kin,beta_kin,x_hat_kin,x(1:6),[xn_i;yn_i;zn_i],psi,theta,params.kin,h);
                Vc_est=Vc_kin; beta_est=beta_kin;
                hat_d=comp_curr(Vc_est,beta_est,x(1:6),psi,tau_thr,M);
            case 3  % LESO
                [Z,~]=vec_leso_update_adv(Z,x(1:6),a_known,params.eso,h);
                hat_d=M*Z(:,3);
            case 4  % PIESO
                [Z,~]=vec_pieso_update(Z,x(1:6),a_known,params.pieso,h);
                hat_d=M*Z(:,3);
            case 5  % EKF
                [x_ekf,P_ekf,aux_ekf]=ekf_current_estimator(x_ekf,P_ekf,x(1:6),tau_thr,psi,M,params,h);
                Vc_est=norm(aux_ekf.c_hat); beta_est=atan2(aux_ekf.c_hat(2),aux_ekf.c_hat(1));
                hat_d=comp_curr(Vc_est,beta_est,x(1:6),psi,tau_thr,M);
            case 6  % UCCO
                [c_hat,~]=eg_ucco_simple(c_hat,x(1:6),tau_thr,psi,M,params.ucco,h);
                Vc_est=norm(c_hat); beta_est=atan2(c_hat(2),c_hat(1));
                hd=comp_curr(Vc_est,beta_est,x(1:6),psi,tau_thr,M);
                hat_d=[0;0;0;0;0;hd(6)];  % 仅Yaw前馈
        end

        [psi_ref,theta_ref,~,~,~,~,~]=my_ALOS3D(xn_i,yn_i,zn_i,h,pts,params.alos);
        [psi_d,r_d]=LOSobserver(psi_d,r_d,psi_ref,h,params.alos.K_f); r_d=sat(r_d,0.5);
        Z_cmd=smc_heave_xhy(zn_i,w,z_d,w_d,w_d_dot,h,params.xhy.heave);
        X_cmd=smc_surge_xhy(u,u_d,u_d_dot,h,params.xhy.surge)-hat_d(1);
        N_cmd=smc_yaw_xhy(psi,r,psi_d,r_d,0,h,params.xhy.yaw)-hat_d(6);
        tau_cmd=[X_cmd;0;Z_cmd;0;0;N_cmd];
        [ui,~]=thrust_allocation_xhy(tau_cmd,thr);

        cN_t=Vc*cos(beta_c); cE_t=Vc*sin(beta_c);
        hist_c_true(i,:)=[cN_t,cE_t];
        hist_c_est(i,:)=[Vc_est*cos(beta_est),Vc_est*sin(beta_est)];
        hist_Vc(i)=Vc_est; hist_beta(i)=beta_est;
        hist_psi_err(i)=psi-psi_d; hist_x(i,:)=x';

        % 状态更新（含模型失配扰动）
        [xdot,~,~,~,~,~,~] = xhy(x, ui, Vc, beta_c, wc);
        if useMismatch
            xdot(1:6) = xdot(1:6) + M \ ((D - D_pert) * nu_r);
        end
        x = x + xdot * h;
        x(12) = ssa(x(12));
    end

    c_err=hist_c_est-hist_c_true;
    results{m}.rmse_Vc=sqrt(mean((c_err(:,1).^2+c_err(:,2).^2)));
    results{m}.rmse_psi=sqrt(mean(hist_psi_err.^2));
    ref_x = pts.pos.x(1:min(N,length(pts.pos.x)));
    ref_y = pts.pos.y(1:min(N,length(pts.pos.y)));
    results{m}.rmse_xy=sqrt(mean((hist_x(1:length(ref_x),7)-ref_x(:)).^2+(hist_x(1:length(ref_y),8)-ref_y(:)).^2));
    results{m}.name=method_names{m};
    fprintf('  RMSE_Vc=%.4f  RMSE_psi=%.4f  RMSE_xy=%.4f\n',results{m}.rmse_Vc,results{m}.rmse_psi,results{m}.rmse_xy);
end

fprintf('\n========== 对比结果 ==========\n');
fprintf('%-10s %10s %10s %10s\n','方法','RMSE_Vc','RMSE_psi','RMSE_xy');
for i=1:6
    fprintf('%-10s %10.4f %10.4f %10.4f\n',results{i}.name,results{i}.rmse_Vc,results{i}.rmse_psi,results{i}.rmse_xy);
end
end

function hd = comp_curr(Vc,b,nu,psi,tau,M)
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

function thr=get_thruster_params_batch()
thr.rho=1026; thr.D_prop_main=0.10; thr.D_prop_aux=0.06;
thr.KT_main_fwd=0.0293; thr.KT_main_rev=0.0201;
thr.KT_aux_fwd=0.327; thr.KT_aux_rev=0.327;
thr.n_max=2500; thr.x_vert_f=+0.344; thr.x_vert_r=-0.293;
thr.x_side_f=+0.424; thr.x_side_r=-0.376;
end
