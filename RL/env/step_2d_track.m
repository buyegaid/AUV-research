function [nextObs, reward, isDone, loggedSignals] = step_2d_track(action, loggedSignals)
% 兼容 cell action
if iscell(action); action = action{1}; end
%% ===== 当前状态 =====
x = loggedSignals.x;        % AUV 状态
u   = x(1);
psi = x(12);
r   = x(6);
xN = x(7);
yE = x(8);
zD = x(9);
n = loggedSignals.ui(3); % 当前转速

params = loggedSignals.params;
alos_params = loggedSignals.alos_params;
wpt = loggedSignals.wpt;
h      = loggedSignals.h;

%% ===== 1. DDPG 参数更新 =====
alpha = 0.05;   % 更新步长

params.K_d(1)      = params.K_d(1)      + alpha * action(1);
params.K_sigma(1)  = params.K_sigma(1)  + alpha * action(2);
params.lambda(1)   = params.lambda(1)   + alpha * action(3);
params.phi_b(1)    = params.phi_b(1)    + alpha * action(4);
params.K_d(3)      = params.K_d(3)      + alpha * action(5);
params.K_sigma(3)  = params.K_sigma(3)  + alpha * action(6);
params.lambda(3)   = params.lambda(3)   + alpha * action(7);
params.phi_b(3)    = params.phi_b(3)    + alpha * action(8);
%% ===== 2. 参数安全约束 =====
params = saturateHeadingParams(params);

%% ===== 3. ISMC 控制 =====
[psi_ref, theta_ref, y_e, z_e, alpha_c_hat, beta_c_hat, d] = ...
    my_ALOS3D(xN, yE, zD, h, wpt, alos_params);

% 航向角滤波
[psi_d, r_d] = LOSobserver(loggedSignals.psi_d, loggedSignals.r_d, psi_ref, h, alos_params.K_f);
r_d = sat(r_d, params.r_max);

delta_r_d = my_integralSMCheading(psi,r,psi_d,r_d,loggedSignals.a_d,h,params);
delta_r = sat(delta_r_d, params.delta_max); % 垂直舵控制（航向轴）

n_rps = my_SMCsurge(u, loggedSignals.u_d, loggedSignals.u_d_dot, h, params);
n_rpm = 60 * n_rps;
if n < n_rpm
    n = n + params.n_rate;
elseif n > n_rpm
    n = n - params.n_rate;
end
n = sat(n, params.n_max);

ui = [delta_r;0;n];
%% ===== 4.  动力学更新 =====
x = rk4(@remus100, h, x, ui, params.Vc, params.betaVc, params.wc);
x(12) = ssa(x(12));
%% ===== 5. 构造误差 =====
psi = x(12);
r   = x(6);
e_psi = wrapToPi(ssa(psi - psi_d));
e_r   = r - r_d;

sigma = e_r + 2*params.lambda(1)*e_psi;
e_u = u - loggedSignals.u_d;
%% ===== 6. 奖励函数 =====
% TODO:需要包含轨迹误差
reward = - 10*e_psi^2 ...
    - 2*e_r^2 ...
    - 0.5*sigma^2 ...
    - 0.2*e_u^2 ...
    - 0.05*y_e^2 ...
    - 0.05*z_e^2;

%% ===== 7. 下一状态 =====
nextObs = [e_psi; e_r; sigma; y_e; z_e; e_u; u; loggedSignals.u_d; psi; r];

%% ===== 8. 终止条件 =====
isDone = false;
final = [wpt.pos.x(end); wpt.pos.y(end); wpt.pos.z(end)];
distFinal = norm([xN; yE; zD] - final);
% fprintf(num2str(distFinal));
% fprintf('\n');
if distFinal < alos_params.R_switch
    isDone = true;  % 到达最终目标
end

if abs(y_e) > 50 || abs(z_e) > 50
    isDone = true;  % 严重偏离路径
end

%% ===== 更新缓存 =====
loggedSignals.x = x;
loggedSignals.params = params;
loggedSignals.psi_d = psi_d;
loggedSignals.r_d = r_d;
loggedSignals.ui = ui;

loggedSignals.psi_ref = psi_ref;
loggedSignals.theta_ref = theta_ref;
loggedSignals.y_e = y_e;
loggedSignals.z_e = z_e;
loggedSignals.alpha_c_hat = alpha_c_hat;
loggedSignals.beta_c_hat  = beta_c_hat;
loggedSignals.d = d;
end