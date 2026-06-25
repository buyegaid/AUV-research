function [c_hat, aux] = pcrco_no_clamp(c_hat, nu_meas, tau, psi, M, params, dt, resetFlag)
% PCRCO_NO_CLAMP  消融变体: 无更新限幅
persistent nu_prev is_initialized noise_est
if nargin<8, resetFlag=false; end
if resetFlag, nu_prev=[]; is_initialized=[]; noise_est=[]; c_hat=[0;0]; aux=struct('c_hat',c_hat); return; end
if isempty(is_initialized), nu_prev=nu_meas; noise_est=0.02; is_initialized=true; end
if isempty(nu_prev), nu_prev=nu_meas; end
if isempty(c_hat), c_hat=[0;0]; end
core=pcrco_core(c_hat,nu_meas,nu_prev,tau,psi,M,params,dt);
noise_est=0.995*noise_est+0.005*norm(core.e_vel);
dc=params.K_obs*100/(1+core.lambda_min*1e6)*core.Phi'*core.e_vel;
% 无限幅
c_hat=c_hat+dc;
alpha=exp(-dt/params.tau_c); c_hat=alpha*c_hat+(1-alpha)*params.c_mean;
c_hat=max(-params.c_max,min(params.c_max,c_hat)); nu_prev=nu_meas;
aux.c_hat=c_hat; aux.e_vel=core.e_vel; aux.lambda_min=core.lambda_min; aux.excited=true; aux.dc_unclamped=dc;
end
