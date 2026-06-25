function [c_hat, aux] = pcrco_no_predcorr(c_hat, nu_meas, tau, psi, M, params, dt, resetFlag)
% PCRCO_NO_PREDCORR V2  消融变体: 加速度残差替代速度预测-校正
%   V2: 连续GM正则化 + 自适应噪声补偿
%   2026-06-21 初版, 2026-06-22 V2

persistent nu_prev a_filt_mem is_initialized noise_est

if nargin < 8, resetFlag = false; end
if resetFlag
    nu_prev=[]; a_filt_mem=[]; is_initialized=[]; noise_est=[];
    c_hat=[0;0]; aux=struct('c_hat',c_hat,'excited',true); return;
end
if isempty(is_initialized)
    nu_prev=nu_meas; a_filt_mem=zeros(6,1); noise_est=0.02; is_initialized=true;
end
if isempty(nu_prev), nu_prev=nu_meas; end
if isempty(c_hat), c_hat=[0;0]; end

delta_c = params.sens_pert;

%% 加速度残差
u_c=c_hat(1)*cos(psi)+c_hat(2)*sin(psi); v_c=-c_hat(1)*sin(psi)+c_hat(2)*cos(psi);
nu_c=[u_c;v_c;0;0;0;0]; nu_r=nu_prev-nu_c;
[tau_drag,~]=xhy_drag_cfd(nu_r); [C_nu,g_nu]=compute_cg_standalone(nu_r,psi,M);
Dnu_c=[nu_prev(6)*v_c;-nu_prev(6)*u_c;0;0;0;0];
a_model=Dnu_c+M\(tau+tau_drag-C_nu*nu_r-g_nu);

a_raw=(nu_meas-nu_prev)/dt;
a_filt_mem=params.accel_lpf_alpha*a_raw+(1-params.accel_lpf_alpha)*a_filt_mem;
e_acc=a_filt_mem(1:2)-a_model(1:2);

%% 加速度灵敏度
Phi_a=zeros(2,2);
for j=1:2
    cp=c_hat; cp(j)=cp(j)+delta_c;
    ucp=cp(1)*cos(psi)+cp(2)*sin(psi); vcp=-cp(1)*sin(psi)+cp(2)*cos(psi);
    nrp=nu_prev-[ucp;vcp;0;0;0;0];
    [tdp,~]=xhy_drag_cfd(nrp); [Cp,gp]=compute_cg_standalone(nrp,psi,M);
    Dnp=[nu_prev(6)*vcp;-nu_prev(6)*ucp;0;0;0;0];
    Phi_a(:,j)=(Dnp+M\(tau+tdp-Cp*nrp-gp)-a_model)/delta_c;
    Phi_a(:,j)=Phi_a(:,j)(1:2);
end
Phi_a=Phi_a(1:2,:);

%% 自适应
noise_est=0.995*noise_est+0.005*norm(e_acc);
nr=noise_est/0.02; mda=params.max_dc*max(0.5,min(3.0,nr)); mda=min(mda,0.20);

Wc=Phi_a'*Phi_a; lm=min(eig(Wc));
gain=params.K_obs*100/(1+lm*1e6);
dc=gain*Phi_a'*e_acc; dc=max(-mda,min(mda,dc));
c_hat=c_hat+dc;

%% 连续GM正则化
alpha=exp(-dt/params.tau_c);
c_hat=alpha*c_hat+(1-alpha)*params.c_mean;
c_hat=max(-params.c_max,min(params.c_max,c_hat));

nu_prev=nu_meas;
aux.c_hat=c_hat; aux.e_acc=e_acc; aux.lambda_min=lm;
aux.excited=true; aux.noise_est=noise_est; aux.predcorr_mode='acceleration';
end
