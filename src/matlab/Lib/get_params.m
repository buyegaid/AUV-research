function params = get_params()
%% 海流参数
params.current.Vc = 0;
params.current.betaVc = 0; % 海流方向相对于正北的夹角，顺时针为正
params.current.wc = 0;

%% 航向 ISMC 参数
params.heading.K_d = 0;          % 误差项增益
params.heading.K_sigma = 0.05;   % 滑模面增益
params.heading.lambda = 0.1;     % 滑模面权重
params.heading.phi_b = 0.1;      % 边界层厚度
params.heading.K_nomoto = 0.1;   % Nomoto 模型参数
params.heading.T_nomoto = 1;     % Nomoto 模型时间常数
params.heading.kd_range     = [0, 0.1];
params.heading.ksigma_range = [deg2rad(1.0), deg2rad(8.0)];
params.heading.lambda_range = [0, 0.1];
params.heading.phi_b_range  = [deg2rad(1.0), deg2rad(10.0)];
params.heading.rho_eso = 0.5; % ESO补偿系数
params.heading.delta_eso_max = deg2rad(5); % ESO补偿最大舵角
%% 俯仰 ISMC 参数
params.pitch.K_d = 0.01;       % 误差项
params.pitch.K_sigma = 0.1;    % 滑模面增益
params.pitch.lambda = 0.1;     % 滑模面权
params.pitch.phi_b = 0.05;     % 边界层厚度
params.pitch.K_nomoto = 0.1;   % Nomoto 模型参数
params.pitch.T_nomoto = 1;     % Nomoto 模型时间
params.pitch.kd_range     = [0, 0.5];
params.pitch.ksigma_range = [deg2rad(1.0), deg2rad(8.0)];
params.pitch.lambda_range = [0.1, 0.1];
params.pitch.phi_b_range  = [deg2rad(1.0), deg2rad(5.0)];

%% 速度 PID 参数
params.surge.Kp = 0.05;        % 比例增益
params.surge.Ki = 0.01;        % 积分增益
params.surge.Kd = 0.02;        % 微分增益
params.surge.Ka = 0;
params.surge.int_sep_th = 0.05; % 积分分离阈值
params.surge.use_accel_feedback = 1; % 是否使用加速度误差补偿
params.surge.Poutrange = [-500, 500];   % 比例项输出范围
params.surge.Ioutrange = [-300, 300];   % 积分
params.surge.Doutrange = [-200, 200];   % 微分项输出范围
params.surge.Aoutrange = [-100, 100];   % 加速补偿项输出范围
params.surge.nrange = [0, 1525];    % 总输出范围 (RPM)
params.surge.int_range = [-2, 2];   % 积分状态范围
params.surge.kp_range      = [0, 400];
params.surge.ki_range     = [0, 40];
params.surge.kd_range     = [0, 80];
params.surge.int_sep_th_range = [0.01, 0.1];
params.surge.K_d = 0.35;
params.surge.K_sigma = 0.05;
params.surge.lambda = 0.1;
params.surge.phi_b = 0.1;
params.surge.K_nomoto = 1/31.3;
params.surge.T_nomoto =1.04;
%% AUV 参数
params.auv.m      = 31.9;     % 示例值，你替换为真实参数
params.auv.Xu_dot = -5.5;
params.auv.d1     = 2.5;
params.auv.d2     = 3.0;
params.auv.K_T    = 0.012;
params.auv.delta_max = deg2rad(20);   % 舵机最大角度（rad）
params.auv.r_max = deg2rad(5.0);      % 最大允许转弯速率（rad/s）
params.auv.q_max = deg2rad(5.0);      % 最大允许俯仰速率（rad/s）
params.auv.n_rate = 1;                % 推进器速率更新限制 (RPM per update)
params.auv.n_max = 1525;              % 推进器最大速度 (RPM)
params.auv.n_min = 0;                 % 推进器最小速度 (RPM)
%% ALOS3D参数
% 3DALOS路径跟踪参数
params.alos.delta_h = 20;            % 水平前视距离 (m)
params.alos.delta_v = 20;            % 垂直前视距离 (m)
params.alos.gamma_h = 0;             % 自适应增益关闭（由ESO补偿海流）
params.alos.gamma_v = 0;             % 自适应增益关闭
params.alos.M_theta = deg2rad(20);   % 最大估计角 (rad)
params.alos.R_switch = 10;           % 航点切换半径 (m)
params.alos.K_f = 0.5;               % LOS观测器的增益 LOS observer gain

%% XHY SMC参数
% 纵荡通道（m_eff = m + Xu_dot: 85.832 + 15.81 = 101.642）
% 注：m_eff 基于 XHY 实际质量 m=85.832kg（非 REMUS 的 33kg）
params.xhy.surge.m_eff  = 101.642;
params.xhy.surge.T1     = 20;      % 纵荡时间常数 (与xhy.m中T1一致)
params.xhy.surge.lambda = 0.3;
params.xhy.surge.Kd     = 5;
params.xhy.surge.Ks     = 3;
params.xhy.surge.phi_b  = 0.1;

%% XHY Surge PID参数（pid_surge_xhy.m）
% 纵荡速度单环PID，输出推力X_cmd (N)，替代SMC用于速度保持/跟踪
% 参数整定依据：开环传递函数 G(s) ≈ 0.197/(20s+1)，目标带宽 ~0.5 rad/s
params.xhy.surge_pid.m_eff       = 101.642;   % 有效质量 m+Xu_dot (kg)
params.xhy.surge_pid.T1          = 20;        % 纵荡时间常数 (s)
params.xhy.surge_pid.Kp          = 50;        % 比例增益 (~0.5 rad/s带宽)
params.xhy.surge_pid.Ki          = 5;         % 积分增益 (消除稳态误差)
params.xhy.surge_pid.Kd          = 30;        % 微分增益 (抑制超调)
params.xhy.surge_pid.X_max       = 10;        % 推力上限 (N), 对应T5约2500RPM正向
params.xhy.surge_pid.X_min       = -3;        % 推力下限 (N), 对应T5约2500RPM反向
params.xhy.surge_pid.int_sep_th  = 0.1;       % 积分分离阈值 (m/s), |e|≤阈值时积分使能
params.xhy.surge_pid.int_max     = 3;         % 积分状态幅值上限 (m·s)
params.xhy.surge_pid.lpf_alpha   = 0.7;       % 微分低通滤波系数, τ_filt≈0.023s
params.xhy.surge_pid.use_ff      = true;      % 启用模型前馈
params.xhy.surge_pid.reset       = 0;         % 复位标志（仿真启动时自动复位）

% 沉浮通道（m_eff_z: 85.832 + 42.87 = 128.702）
% xhy.m 中 B=W（中性浮力），g_z=0
params.xhy.heave.m_eff_z = 128.702;
params.xhy.heave.d_w     = 128.702 / 20;  % 沉浮线性阻尼（时间常数约20s）
params.xhy.heave.g_z     = 0;              % 净浮力（B=W, 中性）
params.xhy.heave.lambda  = 0.3;
params.xhy.heave.Kd      = 8;
params.xhy.heave.Ks      = 5;
params.xhy.heave.phi_b   = 0.05;

% 俯仰通道（Iy_eff: 7.1735 + 0.041 = 7.215）
params.xhy.pitch.Iy_eff = 7.215;
params.xhy.pitch.lambda = 0.5;
params.xhy.pitch.Kd     = 2;
params.xhy.pitch.Ks     = 1;
params.xhy.pitch.phi_b  = 0.05;

% 偏航通道（Iz_eff: 6.3906 + 0.123 = 6.514）
params.xhy.yaw.Iz_eff   = 6.514;
params.xhy.yaw.lambda   = 0.5;
params.xhy.yaw.Kd       = 2;
params.xhy.yaw.Ks       = 1;
params.xhy.yaw.phi_b    = 0.05;

%% ESO参数
% 自适应带宽
params.eso.omega0_base = 3;
params.eso.omega0_max = 5;

params.eso.adapt_err_scale = 0.5;
params.eso.alpha1 = 0.6;
params.eso.alpha2 = 0.6;
params.eso.alpha3 = 0.5;
params.eso.delta = 0.02;
params.eso.meas_lpf_fc = 0.5; % 低通截止频率
params.eso.z3_lpf_fc = 1; % Z3输出一阶低通滤波截止频率
params.eso.use_rk4 = true;
params.eso.z3_sat = 20; % adjust depending units (accels)

%% 物理信息ESO参数（PI-ESO，Gauss-Markov海流模型）
params.pieso = params.eso;  % 继承标准ESO参数
params.pieso.tau_c = 50;    % Gauss-Markov相关时间常数 (s)，对应低频海流

%% EG-UCCO参数（激励门控不确定性标定海流观测器, 2026-06-04）
params.ucco.tau_c = 100;          % GM海流相关时间常数 (s)
params.ucco.c_mean = [0; 0];      % 海流均值 [cN; cE] (m/s)
params.ucco.c_max = 1.5;          % 海流最大速度 (m/s)
params.ucco.K_obs = 10.0;         % 观测器增益 [0623重调确认: 10为最优, 5虽终值好但收敛慢→RMSE差]
params.ucco.Q_c = 0.001;          % 海流过程噪声协方差
params.ucco.R_inv_diag = [10, 10, 1, 1, 1, 1];  % 测量噪声逆（surge/sway高权重）
params.ucco.window_size = 20;     % 滑动窗口大小（用于Gramian累积）
params.ucco.gate_mu = 1e-8;% 激励门控阈值（速度层面Gramian，dt倍数后很小）
params.ucco.sens_pert = 0.01;     % 灵敏度数值扰动步长 (m/s)
params.ucco.max_dc = 0.06;        % 单步最大海流更新量 (m/s)
params.ucco.reg_lambda = 0.1;     % 正则化系数（防病态）
params.ucco.accel_lpf_alpha = 0.3;% 加速度低通滤波系数
params.ucco.conf_scale = 0.01;    % 置信度缩放
% 模型不确定性分量化界（Δ_i = Δ_single + Δ_coupling + Δ_thruster + Δ_sensor）
params.ucco.delta_single = [0.02; 0.05; 0.03; 0.01; 0.01; 0.01];  % 单自由度CFD残差界
params.ucco.delta_coupling = [0.05; 0.03; 0.03; 0; 0.05; 0.05];   % 耦合项不确定性系数
params.ucco.delta_thruster = 0.02;  % 推进器模型不确定性
params.ucco.delta_sensor = 0.01;    % 传感器噪声界

%% Børhaug 2007 模型基海流观测器（简化速度预测型, 2026-06-10）
% 海流估计参数
params.borhaug.K_c = [10; 10];        % 海流积分增益 [0623重调: 80→10, 扫描10-320后选10]
params.borhaug.max_dc = 0.5;          % 单步最大海流变化 (m/s²)
params.borhaug.tau_c = 100;           % GM海流相关时间常数 (s)
params.borhaug.c_mean = [0; 0];       % 海流均值 [cN; cE] (m/s)
params.borhaug.c_max = 1.5;           % 海流最大速度 (m/s)

%% CFD-Augmented EKF参数（强基线, 2026-06-04）
params.ekf.use_bias = false;       % 8状态EKF-nom（无力偏置，展示模型失配敏感性）
params.ekf.tau_c = 100;           % 海流GM时间常数
params.ekf.tau_b = 50;            % 力偏置时间常数
params.ekf.c_mean = [0; 0];       % 海流均值
params.ekf.c_max = 1.5;           % 海流约束
params.ekf.P0 = 0.1;              % 初始协方差
params.ekf.Q_nu = 0.001;           % 速度过程噪声
params.ekf.Q_c = 0.0005;           % 海流过程噪声
params.ekf.Q_b = 0.0001;           % 偏置过程噪声
params.ekf.Q0 = 0.001;            % 默认过程噪声
params.ekf.R0 = 0.0004;           % 测量噪声 (m/s)² [0623重调: 0.01→4e-4, 扫描4e-4-1e-1选最优]
params.ekf.jac_pert = 0.001;      % Jacobian数值扰动

%% 运动学海流观测器参数（速度残差型, 2026-06-10）
% K3: 从速度残差 (m/s) → 海流变化率 (m/s²) 的增益
%     收敛时间 ≈ 1/K3 ≈ 10s
params.kin.K3 = [0.02; 0.02];     % 海流估计速度增益 [cN; cE] (1/s)
                                   % 慢平均≈50s, 滤除转弯侧滑的周期性分量
params.kin.tau_c = 100;           % GM海流相关时间常数 (s)
params.kin.c_mean = [0; 0];       % 海流均值
params.kin.c_max = 1.5;           % 海流上限

end
