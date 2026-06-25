%% 单步追踪：PC-RCO 完整算法 + 消融变体 逐值对比
% 目的：识别逐机制消融"零差异"的根本原因
% 2026-06-21

project_root = setup_paths();
params = get_params();

%% ===== 固定初始条件（圆形轨迹第一步）=====
rng(42);
dt = 0.01;

% XHY初始状态
u0=1.0; v0=0; w0=0; p0=0; q0=0; r0=0;  % 初始角速度为0（第一步）
x0=300; y0=0; z0=10; phi0=0; theta0=0; psi0=pi/2;

nu_meas = [u0; v0; w0; p0; q0; r0];
nu_prev = nu_meas;  % 第一步 nu_prev = nu_meas
psi = psi0;

% 海流真值
Vc = 0.3; betaVc = deg2rad(45);
c_true = [Vc*cos(betaVc); Vc*sin(betaVc)];

% 推力（初始为0，实际由控制器产生）
tau_thr = zeros(6,1);

% 惯性矩阵
m = 85.832;
Ix = 0.553864787 + 0.865274;
Iy = 2.162341935 + 5.011187;
Iz = 1.849137 + 4.541468;
MRB = diag([m,m,m,Ix,Iy,Iz]);
MA = diag([15.81, 124.73, 42.87, 0.014, 0.041, 0.123]);
M_const = MRB + MA;

% 初始海流估计
c_hat0 = [0; 0];

%% ===== 第一部分：PC-RCO (eg_ucco_simple) 完整追踪 =====
fprintf('==================== PC-RCO 单步完整追踪 ====================\n');
fprintf('初始: nu=[%.4f,%.4f,%.4f, %.4f,%.4f,%.4f], psi=%.1f°\n', ...
    nu_meas, rad2deg(psi));
fprintf('c_true=[%.4f,%.4f], c_hat=[%.4f,%.4f]\n\n', c_true, c_hat0);

c_hat = c_hat0;
clear eg_ucco_simple;

% --- Step 1: 海流船体系分量 ---
fprintf('--- Step 1: 海流船体系分量 ---\n');
u_c = c_hat(1)*cos(psi) + c_hat(2)*sin(psi);
v_c = -c_hat(1)*sin(psi) + c_hat(2)*cos(psi);
fprintf('  u_c = cN*cos(%.1f°) + cE*sin(%.1f°) = %.6f\n', rad2deg(psi), rad2deg(psi), u_c);
fprintf('  v_c = -cN*sin(%.1f°) + cE*cos(%.1f°) = %.6f\n', rad2deg(psi), rad2deg(psi), v_c);
nu_c = [u_c; v_c; 0; 0; 0; 0];

% --- Step 2: 相对速度 ---
fprintf('\n--- Step 2: 相对速度 ---\n');
nu_r = nu_prev - nu_c;
fprintf('  nu_r = nu_prev - nu_c = [%.4f,%.4f,%.4f, %.4f,%.4f,%.4f]\n', nu_r);

% --- Step 3: CFD阻力 ---
fprintf('\n--- Step 3: CFD阻力 (xhy_drag_cfd) ---\n');
[tau_drag, D_drag] = xhy_drag_cfd(nu_r);
fprintf('  tau_drag = [%.6f,%.6f,%.6f, %.6f,%.6f,%.6f]\n', tau_drag);
fprintf('  D_drag对角 = [%.6f,%.6f,%.6f, %.6f,%.6f,%.6f]\n', diag(D_drag)');

% --- Step 4: Coriolis + 重力 ---
fprintf('\n--- Step 4: Coriolis + 重力 ---\n');
[C_nu, g_nu] = compute_cg_standalone(nu_r, psi, M_const);
fprintf('  C_nu(1,:) = [%.6f,%.6f,%.6f, %.6f,%.6f,%.6f]\n', C_nu(1,:));
fprintf('  C_nu(2,:) = [%.6f,%.6f,%.6f, %.6f,%.6f,%.6f]\n', C_nu(2,:));
fprintf('  g_nu = [%.6f,%.6f,%.6f, %.6f,%.6f,%.6f]\n', g_nu);

% --- Step 5: Dnu_c项 ---
fprintf('\n--- Step 5: Dnu_c项 ---\n');
r_nu = nu_prev(6);
Dnu_c = [r_nu*v_c; -r_nu*u_c; 0; 0; 0; 0];
fprintf('  Dnu_c = [r*v_c; -r*u_c; 0;0;0;0] = [%.6f,%.6f,0,0,0,0]\n', Dnu_c(1), Dnu_c(2));

% --- Step 6: 模型加速度 ---
fprintf('\n--- Step 6: 模型加速度 ---\n');
a_model = Dnu_c + M_const \ (tau_thr + tau_drag - C_nu*nu_r - g_nu);
fprintf('  M\\(tau+tau_drag-C*nu_r-g) = [%.6f,%.6f,%.6f, %.6f,%.6f,%.6f]\n', ...
    (M_const \ (tau_thr + tau_drag - C_nu*nu_r - g_nu))');
fprintf('  a_model = [%.6f,%.6f,%.6f, %.6f,%.6f,%.6f]\n', a_model);

% --- Step 7: 速度预测 ---
fprintf('\n--- Step 7: 速度预测 ---\n');
nu_pred = nu_prev + a_model * dt;
fprintf('  nu_pred = nu_prev + dt*a_model = [%.6f,%.6f] (surge/sway)\n', nu_pred(1:2));

% --- Step 8: 速度新息 ---
fprintf('\n--- Step 8: 速度新息 ---\n');
e_vel = nu_meas(1:2) - nu_pred(1:2);
fprintf('  e_vel = nu_meas - nu_pred = [%.8f,%.8f]\n', e_vel);
fprintf('  |e_vel| = %.8f m/s\n', norm(e_vel));

% --- Step 9: 数值灵敏度 ---
fprintf('\n--- Step 9: 数值灵敏度 (delta=%.4f) ---\n', params.ucco.sens_pert);
delta_c = params.ucco.sens_pert;
Phi = zeros(2, 2);

for j = 1:2
    cp = c_hat; cp(j) = cp(j) + delta_c;

    u_c_p =  cp(1)*cos(psi) + cp(2)*sin(psi);
    v_c_p = -cp(1)*sin(psi) + cp(2)*cos(psi);
    nu_c_p = [u_c_p; v_c_p; 0; 0; 0; 0];
    nu_r_p = nu_prev - nu_c_p;

    [td_p, ~] = xhy_drag_cfd(nu_r_p);
    [Cp, gp] = compute_cg_standalone(nu_r_p, psi, M_const);
    Dnc_p = [nu_prev(6)*v_c_p; -nu_prev(6)*u_c_p; 0; 0; 0; 0];

    a_p = Dnc_p + M_const \ (tau_thr + td_p - Cp*nu_r_p - gp);
    nu_p = nu_prev + a_p * dt;

    Phi(:, j) = (nu_p(1:2) - nu_pred(1:2)) / delta_c;

    fprintf('  扰动c(%d)+=%.2f: nu_c_p=[%.4f,%.4f], tau_drag_p=[%.4f,%.4f], nu_p=[%.8f,%.8f]\n', ...
        j, delta_c, u_c_p, v_c_p, td_p(1), td_p(2), nu_p(1), nu_p(2));
end
fprintf('  Phi = [%.8f, %.8f; %.8f, %.8f]\n', Phi(1,1), Phi(1,2), Phi(2,1), Phi(2,2));

% --- Step 10: Gramian + 门控 ---
fprintf('\n--- Step 10: 激励门控 ---\n');
Wc = Phi' * Phi;
lambda_min = min(eig(Wc));
fprintf('  Wc = [%.4e, %.4e; %.4e, %.4e]\n', Wc(1,1), Wc(1,2), Wc(2,1), Wc(2,2));
fprintf('  λ_min = %.4e  (gate_mu = %.1e)\n', lambda_min, params.ucco.gate_mu);
excited = (lambda_min > params.ucco.gate_mu);
fprintf('  门控状态: %s\n', condstr(excited));

% --- Step 11: 更新计算 ---
fprintf('\n--- Step 11: 更新计算 ---\n');
gamma_eff = params.ucco.K_obs * 100;
fprintf('  gamma_eff = K_obs*100 = %.0f*100 = %.0f\n', params.ucco.K_obs, gamma_eff);

if excited
    gain = gamma_eff / (1 + lambda_min * 1e6);
    fprintf('  gain = %.0f / (1 + %.4e*1e6) = %.4f\n', gamma_eff, lambda_min, gain);

    dc_raw = gain * Phi' * e_vel;
    fprintf('  dc_raw = gain * Phi'' * e_vel = [%.8f, %.8f]\n', dc_raw);
    fprintf('  |dc_raw| = %.8f  (max_dc = %.4f)\n', norm(dc_raw), params.ucco.max_dc);

    dc = max(-params.ucco.max_dc, min(params.ucco.max_dc, dc_raw));
    fprintf('  dc_clamped = [%.8f, %.8f]\n', dc);
    fprintf('  限幅触发: %s\n', condstr(any(dc ~= dc_raw)));

    c_hat_new = c_hat + dc;
    fprintf('  c_hat_new = [%.8f, %.8f]\n', c_hat_new);
else
    alpha = exp(-dt / params.ucco.tau_c);
    c_hat_new = alpha * c_hat + (1-alpha) * params.ucco.c_mean;
    fprintf('  GM传播: alpha=%.8f, c_hat_new=[%.8f,%.8f]\n', alpha, c_hat_new);
end

c_hat = max(-params.ucco.c_max, min(params.ucco.c_max, c_hat_new));

fprintf('\n===== PC-RCO 单步结果: c_hat = [%.8f, %.8f] =====\n\n', c_hat);


%% ===== 第二部分：消融变体对比 =====
fprintf('==================== 消融变体单步对比 ====================\n\n');

% 为每个变体重置
variants = {'PC-RCO (eg_ucco_simple)', 'NoGM', 'NoClamp', 'NoPredCorr'};
c_hat_results = zeros(2, 4);
dc_results = zeros(2, 4);

for vi = 1:4
    vn = variants{vi};
    ch = [0; 0];

    switch vi
        case 1  % PC-RCO
            clear eg_ucco_simple;
            [ch, aux] = eg_ucco_simple(ch, nu_meas, tau_thr, psi, M_const, params.ucco, dt);

        case 2  % NoGM
            % 手动实现：跳过GM衰减
            clear eg_ucco_simple;
            % 先按PC-RCO计算dc_raw
            [~, aux_ref] = eg_ucco_simple([0;0], nu_meas, tau_thr, psi, M_const, params.ucco, dt);
            % 重新计算（不跳过，只是对比）
            alpha_gm = exp(-dt / params.ucco.tau_c);
            [ch_ref, ~] = eg_ucco_simple([0;0], nu_meas, tau_thr, psi, M_const, params.ucco, dt);
            ch = ch_ref;  % just reference
            fprintf('%s: GM衰减/步=%.6f%%, alpha=%.8f\n', vn, (1-alpha_gm)*100, alpha_gm);

        case 3  % NoClamp
            clear eg_ucco_simple;
            [ch, ~] = eg_ucco_simple(ch, nu_meas, tau_thr, psi, M_const, params.ucco, dt);
            % max_dc=0.06, 检查是否触发
            fprintf('%s: max_dc=%.4f\n', vn, params.ucco.max_dc);

        case 4  % NoPredCorr (加速度残差)
            % 手动实现加速度残差方法
            persistent a_filt_mem
            if isempty(a_filt_mem), a_filt_mem = zeros(6,1); end

            % 计算模型加速度
            u_c_a = ch(1)*cos(psi) + ch(2)*sin(psi);
            v_c_a = -ch(1)*sin(psi) + ch(2)*cos(psi);
            nu_c_a = [u_c_a; v_c_a; 0; 0; 0; 0];
            nu_r_a = nu_prev - nu_c_a;
            [tau_drag_a, ~] = xhy_drag_cfd(nu_r_a);
            [C_a, g_a] = compute_cg_standalone(nu_r_a, psi, M_const);
            Dnu_c_a = [nu_prev(6)*v_c_a; -nu_prev(6)*u_c_a; 0; 0; 0; 0];
            a_model_a = Dnu_c_a + M_const \ (tau_thr + tau_drag_a - C_a*nu_r_a - g_a);

            % 加速度测量（差分+滤波）
            a_raw = (nu_meas - nu_prev) / dt;
            alpha_lpf = params.ucco.accel_lpf_alpha;
            a_filt = alpha_lpf * a_raw + (1-alpha_lpf) * a_filt_mem;
            a_filt_mem = a_filt;
            e_acc = a_filt(1:2) - a_model_a(1:2);

            fprintf('%s: e_acc=[%.8f,%.8f], |e_acc|=%.8f\n', vn, e_acc, norm(e_acc));
            fprintf('  a_raw=[%.4f,%.4f], a_filt=[%.4f,%.4f]\n', a_raw(1), a_raw(2), a_filt(1), a_filt(2));

            % 数值灵敏度（加速度层面）
            Phi_a = zeros(2,2);
            for j_a = 1:2
                cp_a = ch; cp_a(j_a) = cp_a(j_a) + delta_c;
                u_c_pa =  cp_a(1)*cos(psi) + cp_a(2)*sin(psi);
                v_c_pa = -cp_a(1)*sin(psi) + cp_a(2)*cos(psi);
                nu_r_pa = nu_prev - [u_c_pa; v_c_pa; 0; 0; 0; 0];
                [td_pa, ~] = xhy_drag_cfd(nu_r_pa);
                [Cpa, gpa] = compute_cg_standalone(nu_r_pa, psi, M_const);
                Dnc_pa = [nu_prev(6)*v_c_pa; -nu_prev(6)*u_c_pa; 0; 0; 0; 0];
                a_pa = Dnc_pa + M_const \ (tau_thr + td_pa - Cpa*nu_r_pa - gpa);
                Phi_a(:, j_a) = (a_pa(1:2) - a_model_a(1:2)) / delta_c;
            end

            fprintf('  Phi_a = [%.8f, %.8f; %.8f, %.8f]\n', Phi_a(1,1), Phi_a(1,2), Phi_a(2,1), Phi_a(2,2));

            Wc_a = Phi_a' * Phi_a;
            lambda_min_a = min(eig(Wc_a));
            gamma_eff_a = params.ucco.K_obs * 100;
            gain_a = gamma_eff_a / (1 + lambda_min_a * 1e6);
            dc_a_raw = gain_a * Phi_a' * e_acc;

            fprintf('  λ_min=%.4e, gain=%.4f, dc_raw=[%.8f,%.8f]\n', lambda_min_a, gain_a, dc_a_raw);

            dc_a = max(-params.ucco.max_dc, min(params.ucco.max_dc, dc_a_raw));
            ch = ch + dc_a;
            alpha_gm_a = exp(-dt / params.ucco.tau_c);
            ch = alpha_gm_a * ch + (1-alpha_gm_a) * params.ucco.c_mean;
            ch = max(-params.ucco.c_max, min(params.ucco.c_max, ch));

            dc_results(:, vi) = dc_a;
    end

    if vi <= 3
        % 用eg_ucco_simple的结果
        [ch_temp, aux_temp] = eg_ucco_simple([0;0], nu_meas, tau_thr, psi, M_const, params.ucco, dt);
        c_hat_results(:, vi) = ch_temp;
        if vi == 1
            % 记录dc_raw
            % 重新计算以获取dc
            u_c_x = 0; v_c_x = 0;  % c_hat=0
            nu_c_x = [0;0;0;0;0;0]; nu_r_x = nu_prev;
            [td_x, ~] = xhy_drag_cfd(nu_r_x);
            [Cx, gx] = compute_cg_standalone(nu_r_x, psi, M_const);
            Dnc_x = [0;0;0;0;0;0];
            am_x = Dnc_x + M_const \ (tau_thr + td_x - Cx*nu_r_x - gx);
            np_x = nu_prev + am_x * dt;
            ev_x = nu_meas(1:2) - np_x(1:2);

            % Phi (already computed above)
            Wc_x = Phi' * Phi;
            lm_x = min(eig(Wc_x));
            ge_x = params.ucco.K_obs * 100;
            gn_x = ge_x / (1 + lm_x * 1e6);
            dc_x_raw = gn_x * Phi' * ev_x;
            dc_x = max(-params.ucco.max_dc, min(params.ucco.max_dc, dc_x_raw));
            dc_results(:, 1) = dc_x;
        end
    end
    c_hat_results(:, vi) = ch;
end

fprintf('\n===== 单步结果对比 =====\n');
fprintf('%-25s %-16s %-16s %-10s\n', '变体', 'c_hat', 'dc_clamped', '差异vs PC-RCO');
fprintf('%-25s [%+.8f,%+.8f]  [%+.8f,%+.8f]  --\n', ...
    variants{1}, c_hat_results(:,1), dc_results(:,1));

for vi = 2:4
    diff_c = norm(c_hat_results(:,vi) - c_hat_results(:,1));
    fprintf('%-25s [%+.8f,%+.8f]  [%+.8f,%+.8f]  %.2e\n', ...
        variants{vi}, c_hat_results(:,vi), dc_results(:,vi), diff_c);
end

%% ===== 第三部分：关键参数诊断 =====
fprintf('\n==================== 关键参数诊断 ====================\n');

% 1. 限幅阈值分析
fprintf('\n--- 限幅阈值分析 ---\n');
fprintf('max_dc = %.4f m/s\n', params.ucco.max_dc);
fprintf('典型|dc_raw| ≈ %.6f m/s (基于当前步估算)\n', norm(dc_results(:,1)));
fprintf('限幅裕度 = max_dc / |dc_raw| ≈ %.0fx\n', params.ucco.max_dc / max(1e-10, norm(dc_results(:,1))));
fprintf('结论: 限幅阈值是典型更新的 %.0f 倍 → %s\n', ...
    params.ucco.max_dc / max(1e-10, norm(dc_results(:,1))), ...
    condstr(params.ucco.max_dc / max(1e-10, norm(dc_results(:,1))) > 100));

% 2. GM衰减分析
fprintf('\n--- GM衰减分析 ---\n');
alpha_per_step = exp(-dt / params.ucco.tau_c);
decay_per_step_pct = (1 - alpha_per_step) * 100;
fprintf('tau_c = %.0f s\n', params.ucco.tau_c);
fprintf('alpha/步 = %.10f\n', alpha_per_step);
fprintf('衰减/步 = %.6f%% \n', decay_per_step_pct);
fprintf('100s总衰减 = %.4f%%\n', (1 - exp(-100/params.ucco.tau_c))*100);
fprintf('|梯度更新/步| ≈ %.6f, GM衰减/步 ≈ %.10f\n', norm(dc_results(:,1)), ...
    (1-alpha_per_step) * norm(c_hat0));
fprintf('结论: 梯度更新是GM衰减的 %.0f 倍 → GM衰减在活跃估计阶段 %s\n', ...
    norm(dc_results(:,1)) / max(1e-15, (1-alpha_per_step)*max(norm(c_hat0), 0.01)), ...
    condstr(norm(dc_results(:,1)) > 100*(1-alpha_per_step)*max(norm(c_hat0),0.01)));

% 3. 门控分析
fprintf('\n--- 门控分析 ---\n');
fprintf('gate_mu = %.1e\n', params.ucco.gate_mu);
fprintf('λ_min(典型) = %.4e\n', lambda_min);
fprintf('结论: λ_min >> gate_mu → 在圆形轨迹中门控 %s\n', condstr(lambda_min > params.ucco.gate_mu));

% 4. 预测校正 vs 加速度残差 对比
fprintf('\n--- 预测校正 vs 加速度残差 ---\n');
fprintf('速度新息: e_vel=[%.8f,%.8f], |e_vel|=%.8f\n', e_vel, norm(e_vel));
% 加速度残差
a_raw_step = (nu_meas - nu_prev) / dt;
fprintf('加速度残差: a_raw=[%.4f,%.4f], SNR≈%.2f\n', a_raw_step(1), a_raw_step(2), norm(a_raw_step(1:2))/0.02*dt);

%% ===== 第四部分：持续追踪（前100步） =====
fprintf('\n==================== 前10步持续追踪 ====================\n');
fprintf('%-4s %-12s %-12s %-12s %-12s %-8s\n', '步', 'c_hat(1)', 'c_hat(2)', '|dc|', 'λ_min', 'gate');

ch_track = [0; 0];
clear eg_ucco_simple;
nu_persist = nu_meas;

for step = 1:10
    [ch_track, aux_track] = eg_ucco_simple(ch_track, nu_meas, tau_thr, psi, M_const, params.ucco, dt);
    % 手动计算dc
    u_ct = ch_track(1)*cos(psi) + ch_track(2)*sin(psi);
    v_ct = -ch_track(1)*sin(psi) + ch_track(2)*cos(psi);

    % 简化：用前一步的c计算dc
    if step > 1
        fprintf('%-4d %-12.8f %-12.8f %-12.8f %-12.4e %-8s\n', ...
            step, ch_track(1), ch_track(2), norm(ch_track - ch_prev), ...
            aux_track.lambda_min, condstr(aux_track.excited));
    else
        fprintf('%-4d %-12.8f %-12.8f %-12s %-12.4e %-8s\n', ...
            step, ch_track(1), ch_track(2), '--', ...
            aux_track.lambda_min, condstr(aux_track.excited));
    end
    ch_prev = ch_track;
end

%% ===== 第五部分：多步全消融对比 =====
fprintf('\n==================== 100步消融对比 (相同初始条件) ===================\n');

N_test = 100;
variants_short = {'PCRCO', 'NoGM', 'NoClamp', 'NoPredCorr'};
c_final = zeros(2, 4);

for vi = 1:4
    ch = [0; 0];
    clear eg_ucco_simple;
    nu_p = nu_meas;

    for k = 1:N_test
        switch vi
            case 1  % PCRCO
                [ch, ~] = eg_ucco_simple(ch, nu_meas, tau_thr, psi, M_const, params.ucco, dt);
            case 2  % NoGM: 去掉GM衰减
                % 复制eg_ucco_simple但跳过GM衰减
                u_c_n = ch(1)*cos(psi) + ch(2)*sin(psi);
                v_c_n = -ch(1)*sin(psi) + ch(2)*cos(psi);
                nu_c_n = [u_c_n; v_c_n; 0;0;0;0];
                nu_r_n = nu_p - nu_c_n;
                [td_n, ~] = xhy_drag_cfd(nu_r_n);
                [Cn, gn] = compute_cg_standalone(nu_r_n, psi, M_const);
                Dnc_n = [nu_p(6)*v_c_n; -nu_p(6)*u_c_n; 0;0;0;0];
                am_n = Dnc_n + M_const \ (tau_thr + td_n - Cn*nu_r_n - gn);
                np_n = nu_p + am_n * dt;
                ev_n = nu_meas(1:2) - np_n(1:2);

                % Phi
                Phi_n = zeros(2,2);
                for jn = 1:2
                    cp_n = ch; cp_n(jn) = cp_n(jn) + delta_c;
                    ucp = cp_n(1)*cos(psi)+cp_n(2)*sin(psi);
                    vcp = -cp_n(1)*sin(psi)+cp_n(2)*cos(psi);
                    nrp = nu_p - [ucp;vcp;0;0;0;0];
                    [tdp,~] = xhy_drag_cfd(nrp);
                    [Cpp,gpp] = compute_cg_standalone(nrp,psi,M_const);
                    Dnp = [nu_p(6)*vcp; -nu_p(6)*ucp; 0;0;0;0];
                    ap_n = Dnp + M_const \ (tau_thr + tdp - Cpp*nrp - gpp);
                    npp = nu_p + ap_n * dt;
                    Phi_n(:,jn) = (npp(1:2) - np_n(1:2)) / delta_c;
                end

                Wc_n = Phi_n' * Phi_n;
                lm_n = min(eig(Wc_n));
                if lm_n > params.ucco.gate_mu
                    gn_n = (params.ucco.K_obs*100) / (1 + lm_n*1e6);
                    dc_n = gn_n * Phi_n' * ev_n;
                    dc_n = max(-params.ucco.max_dc, min(params.ucco.max_dc, dc_n));
                    ch = ch + dc_n;
                    % === 关键差异: 无GM衰减 ===
                end
                ch = max(-params.ucco.c_max, min(params.ucco.c_max, ch));
                nu_p = nu_meas;

            case 3  % NoClamp: 去掉限幅
                u_c_c = ch(1)*cos(psi) + ch(2)*sin(psi);
                v_c_c = -ch(1)*sin(psi) + ch(2)*cos(psi);
                nu_c_c = [u_c_c; v_c_c; 0;0;0;0];
                nu_r_c = nu_p - nu_c_c;
                [td_c, ~] = xhy_drag_cfd(nu_r_c);
                [Cc, gc] = compute_cg_standalone(nu_r_c, psi, M_const);
                Dnc_c = [nu_p(6)*v_c_c; -nu_p(6)*u_c_c; 0;0;0;0];
                am_c = Dnc_c + M_const \ (tau_thr + td_c - Cc*nu_r_c - gc);
                np_c = nu_p + am_c * dt;
                ev_c = nu_meas(1:2) - np_c(1:2);

                Phi_c = zeros(2,2);
                for jc = 1:2
                    cp_c = ch; cp_c(jc) = cp_c(jc) + delta_c;
                    ucp_c = cp_c(1)*cos(psi)+cp_c(2)*sin(psi);
                    vcp_c = -cp_c(1)*sin(psi)+cp_c(2)*cos(psi);
                    nrp_c = nu_p - [ucp_c;vcp_c;0;0;0;0];
                    [tdp_c,~] = xhy_drag_cfd(nrp_c);
                    [Cpp_c,gpp_c] = compute_cg_standalone(nrp_c,psi,M_const);
                    Dnp_c = [nu_p(6)*vcp_c; -nu_p(6)*ucp_c; 0;0;0;0];
                    ap_c = Dnp_c + M_const \ (tau_thr + tdp_c - Cpp_c*nrp_c - gpp_c);
                    npp_c = nu_p + ap_c * dt;
                    Phi_c(:,jc) = (npp_c(1:2) - np_c(1:2)) / delta_c;
                end

                Wc_c = Phi_c' * Phi_c;
                lm_c = min(eig(Wc_c));
                if lm_c > params.ucco.gate_mu
                    gn_c = (params.ucco.K_obs*100) / (1 + lm_c*1e6);
                    dc_c = gn_c * Phi_c' * ev_c;
                    % === 关键差异: 无限幅 ===
                    ch = ch + dc_c;
                end
                alpha_c = exp(-dt / params.ucco.tau_c);
                ch = alpha_c * ch + (1-alpha_c) * params.ucco.c_mean;
                ch = max(-params.ucco.c_max, min(params.ucco.c_max, ch));
                nu_p = nu_meas;

            case 4  % NoPredCorr: 加速度残差
                u_c_a2 = ch(1)*cos(psi) + ch(2)*sin(psi);
                v_c_a2 = -ch(1)*sin(psi) + ch(2)*cos(psi);
                nu_c_a2 = [u_c_a2; v_c_a2; 0;0;0;0];
                nu_r_a2 = nu_p - nu_c_a2;
                [td_a2, ~] = xhy_drag_cfd(nu_r_a2);
                [Ca2, ga2] = compute_cg_standalone(nu_r_a2, psi, M_const);
                Dnc_a2 = [nu_p(6)*v_c_a2; -nu_p(6)*u_c_a2; 0;0;0;0];
                am_a2 = Dnc_a2 + M_const \ (tau_thr + td_a2 - Ca2*nu_r_a2 - ga2);

                % 加速度残差
                a_raw2 = (nu_meas - nu_p) / dt;
                persistent af_mem2
                if isempty(af_mem2), af_mem2 = zeros(6,1); end
                af_mem2 = params.ucco.accel_lpf_alpha * a_raw2 + (1-params.ucco.accel_lpf_alpha) * af_mem2;
                ea2 = af_mem2(1:2) - am_a2(1:2);

                Phi_a2 = zeros(2,2);
                for ja = 1:2
                    cp_a2 = ch; cp_a2(ja) = cp_a2(ja) + delta_c;
                    ucp_a2 = cp_a2(1)*cos(psi)+cp_a2(2)*sin(psi);
                    vcp_a2 = -cp_a2(1)*sin(psi)+cp_a2(2)*cos(psi);
                    nrp_a2 = nu_p - [ucp_a2;vcp_a2;0;0;0;0];
                    [tdp_a2,~] = xhy_drag_cfd(nrp_a2);
                    [Cpp_a2,gpp_a2] = compute_cg_standalone(nrp_a2,psi,M_const);
                    Dnp_a2 = [nu_p(6)*vcp_a2; -nu_p(6)*ucp_a2; 0;0;0;0];
                    ap_a2 = Dnp_a2 + M_const \ (tau_thr + tdp_a2 - Cpp_a2*nrp_a2 - gpp_a2);
                    Phi_a2(:,ja) = (ap_a2(1:2) - am_a2(1:2)) / delta_c;
                end

                Wc_a2 = Phi_a2' * Phi_a2;
                lm_a2 = min(eig(Wc_a2));
                if lm_a2 > params.ucco.gate_mu
                    gn_a2 = (params.ucco.K_obs*100) / (1 + lm_a2*1e6);
                    dc_a2 = gn_a2 * Phi_a2' * ea2;
                    dc_a2 = max(-params.ucco.max_dc, min(params.ucco.max_dc, dc_a2));
                    ch = ch + dc_a2;
                end
                alpha_a2 = exp(-dt / params.ucco.tau_c);
                ch = alpha_a2 * ch + (1-alpha_a2) * params.ucco.c_mean;
                ch = max(-params.ucco.c_max, min(params.ucco.c_max, ch));
                nu_p = nu_meas;
        end
    end
    c_final(:, vi) = ch;
    fprintf('%-12s: c_hat_100=[%.8f,%.8f], 误差=[%.6f,%.6f], RMSE_c=%.6f\n', ...
        variants_short{vi}, ch(1), ch(2), ch(1)-c_true(1), ch(2)-c_true(2), norm(ch-c_true));
end

fprintf('\n===== 消融对比 (100步) =====\n');
fprintf('%-12s %-20s %-12s\n', '变体', 'c_hat_100', '差异vs PCRCO');
for vi = 1:4
    if vi == 1
        fprintf('%-12s [%+.8f,%+.8f]  --\n', variants_short{vi}, c_final(:,vi));
    else
        fprintf('%-12s [%+.8f,%+.8f]  %.2e\n', variants_short{vi}, c_final(:,vi), ...
            norm(c_final(:,vi) - c_final(:,1)));
    end
end

%% ===== 辅助函数 =====
function s = condstr(cond)
if cond
    s = 'YES ⚠';
else
    s = 'no';
end
end

fprintf('\n诊断完成。\n');
