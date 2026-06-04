%% Gate ablation + sensor noise
% 验证激励门控和噪声鲁棒性
% 2026-06-04

function gate_ablation()
addpath('./Lib','./guidance','./controller/xhy','./controller/remus','./model','./eso','./post','./traj');

h=0.01; T=100; t=0:h:T; N=length(t);
gm=struct('Vc_mean',0.3,'betaVc_mean',pi/4,'sigma_Vc',0.1,'tau_c',100);
[Vs,bs,ws]=gauss_markov_current(2,t,gm);
pts=traj(50,50);
thr=get_thruster_params_ga();

% 门控配置: [gate_mu, 描述]
gate_configs = {
    1e-8,  'Gramian gate (default)';
    0,     'No gate (always)';
    0.01,  'High threshold';
    };
noise_levels = [0, 1, 2];  % 0=无噪声, 1=低噪声, 2=高噪声
noise_names = {'noiseless','low noise','high noise'};

n_gates = size(gate_configs,1);

fprintf('Gate ablation + noise: %d gates x %d noise levels\n', n_gates, length(noise_levels));

for ng = 1:length(noise_levels)
    noise_lvl = noise_levels(ng);
    fprintf('\n=== %s ===\n', noise_names{ng});

    for gc = 1:n_gates
        gate_mu = gate_configs{gc,1};
        gate_name = gate_configs{gc,2};

        params=get_params;
        params.ucco.gate_mu = gate_mu;

        % 噪声参数
        if noise_lvl >= 1
            noise_scale_vel = 0.02;   % DVL速度噪声 std (m/s)
            noise_scale_yaw = deg2rad(0.5);  % 航向角噪声 std (rad)
        else
            noise_scale_vel = 0;
            noise_scale_yaw = 0;
        end
        if noise_lvl >= 2
            noise_scale_vel = 0.05;
            noise_scale_yaw = deg2rad(1.0);
        end

        x=[1;0;0;0;0;0;0;0;10;0;0;0];
        c_hat=[0;0]; nu_obs=x(1:6);
        psi_d=0; r_d=0; u_d=1; z_d=10; w_d=0;
        ui=zeros(5,1); thr_n=get_thruster_params_ga();

        c_est_hist=zeros(N,2); c_true_hist=zeros(N,2);
        update_count=0; excite_count=0;

        for i=1:N
            u=x(1); v=x(2); w=x(3); r=x(6); xn=x(7); yn=x(8); zn=x(9);
            theta=x(11); psi=x(12);
            Vc=Vs(i); bc=bs(i); wc=ws(i);

            % 传感器噪声
            nu_meas = x(1:6);
            psi_meas = psi;
            if noise_lvl > 0
                nu_meas = nu_meas + noise_scale_vel*randn(6,1);
                psi_meas = psi + noise_scale_yaw*randn;
            end

            [~,~,M,C,D,g_vec,tau_thr]=xhy(x,ui,Vc,bc,wc);
            u_c_x=Vc*cos(bc-psi); u_c_y=Vc*sin(bc-psi);
            nu_r=x(1:6)-[u_c_x;u_c_y;wc;0;0;0];
            a_known=M\(tau_thr-C*nu_r-D*nu_r-g_vec);

            % UCCO with current gate setting
            [c_hat,aux]=eg_ucco_simple(c_hat,nu_meas,tau_thr,psi_meas,M,params.ucco,h);

            Vc_est=norm(c_hat); beta_est=atan2(c_hat(2),c_hat(1));
            hd=comp_curr_ga(Vc_est,beta_est,x(1:6),psi,tau_thr,M);
            hat_d=[0;0;0;0;0;hd(6)];

            if aux.excited, excite_count=excite_count+1; end
            if norm(c_hat) > 1e-6, update_count=update_count+1; end

            [psi_ref,theta_ref,~,~,~,~,~]=my_ALOS3D(xn,yn,zn,h,pts,params.alos);
            [psi_d,r_d]=LOSobserver(psi_d,r_d,psi_ref,h,params.alos.K_f); r_d=sat(r_d,0.5);
            Z_cmd=smc_heave_xhy(zn,w,z_d,w_d,0,h,params.xhy.heave);
            N_cmd=smc_yaw_xhy(psi,r,psi_d,r_d,0,h,params.xhy.yaw)-hat_d(6);
            [ui,~]=thrust_allocation_xhy([smc_surge_xhy(u,u_d,0,h,params.xhy.surge);0;Z_cmd;0;0;N_cmd],thr_n);

            cN_t=Vc*cos(bc); cE_t=Vc*sin(bc);
            c_est_hist(i,:)=[Vc_est*cos(beta_est),Vc_est*sin(beta_est)];
            c_true_hist(i,:)=[cN_t,cE_t];

            x=rk4(@xhy,h,x,ui,Vc,bc,wc); x(12)=ssa(x(12));
        end

        c_err=c_est_hist-c_true_hist;
        rmse_Vc=sqrt(mean(c_err(:,1).^2+c_err(:,2).^2));
        excite_rate=excite_count/N*100;
        update_rate=update_count/N*100;

        fprintf('  %-25s: RMSE_Vc=%.4f  excite=%.0f%%  update=%.0f%%\n', ...
            gate_name, rmse_Vc, excite_rate, update_rate);
    end
end

fprintf('\nDone.\n');
end

function hd=comp_curr_ga(Vc,b,nu,psi,tau,M)
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

function thr=get_thruster_params_ga()
thr.rho=1026; thr.D_prop_main=0.10; thr.D_prop_aux=0.06;
thr.KT_main_fwd=0.0293; thr.KT_main_rev=0.0201;
thr.KT_aux_fwd=0.327; thr.KT_aux_rev=0.327;
thr.n_max=2500; thr.x_vert_f=+0.344; thr.x_vert_r=-0.293;
thr.x_side_f=+0.424; thr.x_side_r=-0.376;
end
