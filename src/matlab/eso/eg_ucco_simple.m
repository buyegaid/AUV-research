function [c_hat, aux] = eg_ucco_simple(c_hat, nu, tau, psi, M, params, dt)
% EG-UCCO (PC-RCO): CFD先验海流观测器
%   速度预测-校正 + 连续GM正则化 + 自适应噪声补偿
%
%   核心:
%     1. 速度预测 + 灵敏度                    [pcrco_core]
%     2. 梯度更新: dc = γ·Φ'·e_ν / (1+λ·κ)
%     3. 自适应限幅: noise_est → max_dc动态缩放
%     4. 连续GM正则化: ĉ = α·ĉ + (1-α)·c̄ (始终激活)
%
%   2026-06-22 最终版: 去除幅值平滑和失配惩罚, 保持核心算法简洁

persistent nu_prev is_initialized noise_est

if isempty(is_initialized)
    nu_prev = nu;
    noise_est = 0.02;
    is_initialized = true;
end
if isempty(nu_prev), nu_prev = nu; end
if isempty(c_hat), c_hat = [0; 0]; end

%% 1. 速度预测 + 灵敏度
core = pcrco_core(c_hat, nu, nu_prev, tau, psi, M, params, dt);

%% 2. 噪声估计 (EMA)
noise_est = 0.995 * noise_est + 0.005 * norm(core.e_vel);

%% 3. 自适应限幅
noise_ratio = noise_est / max(0.005, 0.02);
max_dc_adaptive = params.max_dc * max(0.5, min(3.0, noise_ratio));
max_dc_adaptive = min(max_dc_adaptive, 0.20);

%% 4. 梯度更新
gamma_eff = params.K_obs * 100;
gain = gamma_eff / (1 + core.lambda_min * 1e6);
dc = gain * core.Phi' * core.e_vel;
dc = max(-max_dc_adaptive, min(max_dc_adaptive, dc));
c_hat = c_hat + dc;

%% 5. 连续GM正则化
alpha = exp(-dt / params.tau_c);
c_hat = alpha * c_hat + (1-alpha) * params.c_mean;

%% 6. 全局约束
c_hat = max(-params.c_max, min(params.c_max, c_hat));
nu_prev = nu;

%% 7. 诊断
aux.c_hat           = c_hat;
aux.e_vel           = core.e_vel;
aux.lambda_min      = core.lambda_min;
aux.excited         = true;
aux.gm_applied      = true;
aux.noise_est       = noise_est;
aux.max_dc_adaptive = max_dc_adaptive;
end
