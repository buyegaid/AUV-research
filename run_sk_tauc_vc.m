function pos_rmse = run_sk_tauc_vc(omega0, seed, usePIESO, tau_c_val, Vc_mean)
% 定点控制测试：可变omega0、种子、tau_c、Vc_mean
clear smc_yaw_xhy smc_heave_xhy smc_surge_xhy vec_leso_update_adv vec_pieso_update

addpath('.', './Lib', './guidance', './controller/xhy', './model', './eso');

params = get_params;
params.xhy.surge.Ks = 3.0;
params.eso.omega0_base = omega0;
params.eso.omega0_max  = omega0 * 1.5;
params.pieso = params.eso;
params.pieso.tau_c = tau_c_val;

h = 0.01; T = 200; t = 0:h:T; N = length(t);

rng(seed);
gm.Vc_mean = Vc_mean; gm.betaVc_mean = pi/4;
gm.sigma_Vc = Vc_mean * 0.1; gm.tau_c = tau_c_val;
[Vc_s, bVc_s, ~] = gauss_markov_current(2, t, gm);

x = [0.5; 0; 0; 0; 0; 0; 5; 5; 12; 0; 0; 0];
Z = zeros(6, 3);
thr.rho=1026; thr.D_prop=0.10; thr.KT=0.22; thr.n_max=2500;
thr.x_vert_f=0.344; thr.x_vert_r=-0.293;
thr.x_side_f=0.424; thr.x_side_r=-0.376;

target = [0; 0; 10];
ui = zeros(5, 1);
hist_pos = zeros(N, 3);

for i = 1:N
    psi = x(12); Vc = Vc_s(i); bVc = bVc_s(i);
    [~, ~, M, C, ~, g_vec, tau_thr] = xhy(x, ui, Vc, bVc, 0);
    uc_x = Vc*cos(bVc-psi); uc_y = Vc*sin(bVc-psi);
    nu_r = x(1:6) - [uc_x; uc_y; 0; 0; 0; 0];
    a_k = M \ (tau_thr - C*nu_r - g_vec);

    if usePIESO
        [Z, ~] = vec_pieso_update(Z, x(1:6), a_k, params.pieso, h);
    else
        [Z, ~] = vec_leso_update_adv(Z, x(1:6), a_k, params.eso, h);
    end
    hat_d = M * Z(:, 3);

    e_NED = target - x(7:9);
    vNED = [x(1)*cos(psi)-x(2)*sin(psi); x(1)*sin(psi)+x(2)*cos(psi); x(3)];
    Vd = 0.3*e_NED - 0.8*vNED;
    u_d = max(-0.5, min(1.0, Vd(1)*cos(psi)+Vd(2)*sin(psi)));
    psi_d = psi;
    if norm(Vd(1:2)) > 0.1, psi_d = atan2(Vd(2), Vd(1)); end

    Xc = smc_surge_xhy(x(1), u_d, 0, h, params.xhy.surge) - hat_d(1);
    Nc = smc_yaw_xhy(psi, x(6), psi_d, 0, 0, h, params.xhy.yaw) - hat_d(6);
    Zc = smc_heave_xhy(x(9), x(3), 10, 0, 0, h, params.xhy.heave) - hat_d(3);

    tau = [Xc; 0; Zc; 0; 0; Nc];
    ui = thrust_allocation_xhy(tau, thr);
    x = rk4(@xhy, h, x, ui, Vc, bVc, 0);
    x(12) = ssa(x(12));
    hist_pos(i, :) = x(7:9)';
end

steady = max(1, N-10000):N;
ep = hist_pos(steady, :) - target';
pos_rmse = sqrt(mean(sum(ep.^2, 2)));
end
