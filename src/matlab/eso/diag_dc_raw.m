%% 诊断: dc_raw统计 + λ_min时程 (straight 30% low noise)
% 2026-06-11

project_root = setup_paths();
params = get_params();

dt = 0.01; T_end = 100; N = T_end/dt + 1; rng(42);
Vc = 0.3; betaVc = deg2rad(45); wc = 0;
c_true = [Vc*cos(betaVc); Vc*sin(betaVc)];

% 直线轨迹
pts.pos.x = (0:20:1000)'; pts.pos.y = zeros(51,1); pts.pos.z = 10*ones(51,1);
x_state = [1;0;0;0;0;0;0;0;10;0;0;0];
M_const = compute_M_constant();

thr_params.rho=1026; thr_params.D_prop_main=0.08; thr_params.D_prop_aux=0.06;
thr_params.KT_main_fwd=0.1489; thr_params.KT_main_rev=0.0506;
thr_params.KT_aux_fwd=0.53; thr_params.KT_aux_rev=0.71;  % 0616非饱和
thr_params.n_max=2500;
thr_params.x_vert_f=+0.344; thr_params.x_vert_r=-0.293;
thr_params.x_side_f=+0.424; thr_params.x_side_r=-0.376;
opt_xhy = struct('mode','rpm','mismatch_pct',30,'mismatch_seed',42);

c_hat = [0;0]; clear eg_ucco_simple;
nu_prev = x_state(1:6);
psi_d=0; r_d=0; u_d=1.0; ui=zeros(5,1);

dc_raw_log = zeros(N,2); dc_clamped_log = zeros(N,2);
lambda_log = zeros(N,1); e_vel_log = zeros(N,2);
c_hat_log = zeros(N,2);

for i = 1:N
    nu = x_state(1:6); xn=x_state(7); yn=x_state(8); zn=x_state(9); psi=x_state(12);
    nu_meas = nu + [0.02*randn(2,1); 0;0;0;0];

    [~,~,M,~,~,~,tau_thr] = xhy(x_state,ui,Vc,betaVc,wc,opt_xhy);
    [psi_ref,~,~,~,~,~,~] = my_ALOS3D(xn,yn,zn,dt,pts,params.alos);
    [psi_d,r_d] = LOSobserver(psi_d,r_d,psi_ref,dt,params.alos.K_f);
    r_d=sat(r_d,0.5);
    X=smc_surge_xhy(nu(1),u_d,0,dt,params.xhy.surge);
    Nc=smc_yaw_xhy(psi,nu(6),psi_d,r_d,0,dt,params.xhy.yaw);
    Z=smc_heave_xhy(zn,nu(3),10,0,0,dt,params.xhy.heave);
    tau_cmd=[X;0;Z;0;0;Nc];
    [ui,~]=thrust_allocation_xhy(tau_cmd,thr_params);

    % ---- UCCO 手动实现（记录dc_raw） ----
    u_c=c_hat(1)*cos(psi)+c_hat(2)*sin(psi);
    v_c=-c_hat(1)*sin(psi)+c_hat(2)*cos(psi);
    nu_c=[u_c;v_c;0;0;0;0]; nu_r=nu_prev-nu_c;
    [tau_drag,~]=xhy_drag_cfd(nu_r);
    [C_nu,g_nu]=compute_cg_standalone(nu_r,psi,M);
    r_nu=nu_prev(6); Dnu_c=[r_nu*v_c;-r_nu*u_c;0;0;0;0];
    a_model=Dnu_c+M_const\(tau_thr+tau_drag-C_nu*nu_r-g_nu);
    nu_pred=nu_prev+a_model*dt;
    e_vel=nu_meas(1:2)-nu_pred(1:2);

    delta=params.ucco.sens_pert; Phi=zeros(2,2);
    for j=1:2
        cp=c_hat; cp(j)=cp(j)+delta;
        u_c_p=cp(1)*cos(psi)+cp(2)*sin(psi);
        v_c_p=-cp(1)*sin(psi)+cp(2)*cos(psi);
        nu_r_p=nu_prev-[u_c_p;v_c_p;0;0;0;0];
        [td_p,~]=xhy_drag_cfd(nu_r_p);
        [Cp,gp]=compute_cg_standalone(nu_r_p,psi,M);
        Dnc_p=[nu_prev(6)*v_c_p;-nu_prev(6)*u_c_p;0;0;0;0];
        a_p=Dnc_p+M_const\(tau_thr+td_p-Cp*nu_r_p-gp);
        nu_p=nu_prev+a_p*dt;
        Phi(:,j)=(nu_p(1:2)-nu_pred(1:2))/delta;
    end

    Wc=Phi'*Phi; lambda_min=min(eig(Wc));
    gamma_eff=params.ucco.K_obs*100;
    gain=gamma_eff/(1+lambda_min*1e6);
    dc_raw=gain*Phi'*e_vel;
    dc=max(-params.ucco.max_dc,min(params.ucco.max_dc,dc_raw));
    c_hat=c_hat+dc;
    alpha=exp(-dt/params.ucco.tau_c);
    c_hat=alpha*c_hat+(1-alpha)*params.ucco.c_mean;
    c_hat=max(-params.ucco.c_max,min(params.ucco.c_max,c_hat));
    nu_prev=nu_meas;

    dc_raw_log(i,:)=dc_raw';
    dc_clamped_log(i,:)=dc';
    lambda_log(i)=lambda_min;
    e_vel_log(i,:)=e_vel';
    c_hat_log(i,:)=c_hat';

    x_state=rk4(@xhy,dt,x_state,ui,Vc,betaVc,wc,opt_xhy);
    x_state(12)=ssa(x_state(12));
end

% 统计
burn=round(N*0.2); v=burn:N;
fprintf('===== dc_raw 诊断 =====\n');
fprintf('|dc_raw| mean: [%.6f, %.6f] m/s\n', mean(abs(dc_raw_log(v,1))), mean(abs(dc_raw_log(v,2))));
fprintf('|dc_raw| max:  [%.6f, %.6f] m/s\n', max(abs(dc_raw_log(v,1))), max(abs(dc_raw_log(v,2))));
fprintf('|dc_raw| > 0.1 比例: [%.1f%%, %.1f%%]\n', ...
    100*mean(abs(dc_raw_log(v,1))>0.1), 100*mean(abs(dc_raw_log(v,2))>0.1));
fprintf('clamp触发比例: dc≠dc_raw: [%.1f%%, %.1f%%]\n', ...
    100*mean(dc_clamped_log(v,1)~=dc_raw_log(v,1)), 100*mean(dc_clamped_log(v,2)~=dc_raw_log(v,2)));
fprintf('lambda_min: mean=%.2e, min=%.2e, max=%.2e\n', ...
    mean(lambda_log(v)), min(lambda_log(v)), max(lambda_log(v)));
fprintf('|e_vel| mean: [%.6f, %.6f] m/s\n', mean(abs(e_vel_log(v,1))), mean(abs(e_vel_log(v,2))));
fprintf('GM衰减/步: %.4f %% (tau_c=%.0f)\n', (1-exp(-dt/params.ucco.tau_c))*100, params.ucco.tau_c);
fprintf('最终c_hat: [%.4f, %.4f] (真实: [%.4f, %.4f])\n', c_hat(1), c_hat(2), c_true(1), c_true(2));
fprintf('RMSE(c_hat vs c_true): %.4f\n', sqrt(mean(sum((c_hat_log(v,:)-repmat(c_true',length(v),1)).^2,2))));

% 分段统计
fprintf('\n===== 分段分析 =====\n');
seg1=1:2000; seg2=2001:5000; seg3=5001:N;
fprintf('t=0-20s:  |dc_raw|=[%.4f,%.4f], |e_vel|=[%.4f,%.4f]\n', ...
    mean(abs(dc_raw_log(seg1,1))),mean(abs(dc_raw_log(seg1,2))),...
    mean(abs(e_vel_log(seg1,1))),mean(abs(e_vel_log(seg1,2))));
fprintf('t=20-50s: |dc_raw|=[%.4f,%.4f], |e_vel|=[%.4f,%.4f]\n', ...
    mean(abs(dc_raw_log(seg2,1))),mean(abs(dc_raw_log(seg2,2))),...
    mean(abs(e_vel_log(seg2,1))),mean(abs(e_vel_log(seg2,2))));
fprintf('t=50-100s:|dc_raw|=[%.4f,%.4f], |e_vel|=[%.4f,%.4f]\n', ...
    mean(abs(dc_raw_log(seg3,1))),mean(abs(dc_raw_log(seg3,2))),...
    mean(abs(e_vel_log(seg3,1))),mean(abs(e_vel_log(seg3,2))));

function M=compute_M_constant()
m=85.832; Ix=0.553864787+0.865274; Iy=2.162341935+5.011187; Iz=1.849137+4.541468;
MRB=diag([m,m,m,Ix,Iy,Iz]); MA=diag([15.81,124.73,42.87,0.014,0.041,0.123]); M=MRB+MA;
end
