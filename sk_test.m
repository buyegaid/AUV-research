function pos_rmse = sk_test(Ks, Vc, useESO, usePIESO)
% 定点控制测试：支持自定义Ks和Vc
clear smc_yaw_xhy smc_heave_xhy smc_surge_xhy vec_leso_update_adv vec_pieso_update

addpath('.', './Lib', './guidance', './controller/xhy', './model', './eso');

params = get_params;
params.xhy.surge.Ks = Ks;
params.xhy.yaw.Ks   = Ks * 0.33;

h = 0.01; T = 200; t = 0:h:T; N = length(t);

rng(42);
gm.Vc_mean = Vc; gm.betaVc_mean = pi/4;
gm.sigma_Vc = Vc * 0.1; gm.tau_c = 50;
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
    psi = x(12);
    Vc_i = Vc_s(i); bVc_i = bVc_s(i);

    [~, ~, M, C, ~, g_vec, tau_thr] = xhy(x, ui, Vc_i, bVc_i, 0);
    uc_x = Vc_i * cos(bVc_i - psi);
    uc_y = Vc_i * sin(bVc_i - psi);
    nu_c = [uc_x; uc_y; 0; 0; 0; 0];
    nu_r = x(1:6) - nu_c;
    a_k = M \ (tau_thr - C*nu_r - g_vec);

    if usePIESO
        [Z, ~] = vec_pieso_update(Z, x(1:6), a_k, params.pieso, h);
    elseif useESO
        [Z, ~] = vec_leso_update_adv(Z, x(1:6), a_k, params.eso, h);
    end

    if useESO
        hat_d = M * Z(:, 3);
    else
        hat_d = zeros(6, 1);
    end

    e_NED = target - x(7:9);
    Kp = 0.3; Kd = 0.8;
    vNED = [x(1)*cos(psi) - x(2)*sin(psi);
            x(1)*sin(psi) + x(2)*cos(psi); x(3)];
    Vd_NED = Kp * e_NED - Kd * vNED;
    u_d = Vd_NED(1)*cos(psi) + Vd_NED(2)*sin(psi);
    u_d = max(-0.5, min(1.0, u_d));
    if norm(Vd_NED(1:2)) > 0.1
        psi_d = atan2(Vd_NED(2), Vd_NED(1));
    else
        psi_d = psi;
    end

    Xc = smc_surge_xhy(x(1), u_d, 0, h, params.xhy.surge) - hat_d(1);
    Nc = smc_yaw_xhy(psi, x(6), psi_d, 0, 0, h, params.xhy.yaw) - hat_d(6);
    Zc = smc_heave_xhy(x(9), x(3), 10, 0, 0, h, params.xhy.heave) - hat_d(3);

    tau = [Xc; 0; Zc; 0; 0; Nc];
    ui = thrust_allocation_xhy(tau, thr);
    x = rk4(@xhy, h, x, ui, Vc_i, bVc_i, 0);
    x(12) = ssa(x(12));
    hist_pos(i, :) = x(7:9)';
end

steady = max(1, N-10000):N;
ep = hist_pos(steady, :) - target';
pos_rmse = sqrt(mean(sum(ep.^2, 2)));
end
