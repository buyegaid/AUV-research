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
% 纵荡通道 (m_eff = m + Xu_dot = 33 + 11.2773)
params.xhy.surge.m_eff  = 44.2773;
params.xhy.surge.T1     = 20;      % 纵荡时间常数 (与xhy.m中T1一致)
params.xhy.surge.lambda = 0.3;
params.xhy.surge.Kd     = 5;
params.xhy.surge.Ks     = 3;
params.xhy.surge.phi_b  = 0.1;

% 沉浮通道 (m_eff_z = m + Zw_dot = 33 + 47.104)
% g_z = B - W = W*0.01 ≈ 33*9.81*0.01 ≈ 3.24 N（正浮力，NED中需向下施力补偿）
params.xhy.heave.m_eff_z = 80.104;
params.xhy.heave.d_w     = 80.104 / 20;  % 沉浮线性阻尼（时间常数约20s）
params.xhy.heave.g_z     = 33 * 9.81 * 0.01;  % 净浮力 B-W (N)
params.xhy.heave.lambda  = 0.3;
params.xhy.heave.Kd      = 8;
params.xhy.heave.Ks      = 5;
params.xhy.heave.phi_b   = 0.05;

% 俯仰通道 (Iy_eff = Iy + Mq_dot = 2.107488 + 0.043)
params.xhy.pitch.Iy_eff = 2.150488;
params.xhy.pitch.lambda = 0.5;
params.xhy.pitch.Kd     = 2;
params.xhy.pitch.Ks     = 1;
params.xhy.pitch.phi_b  = 0.05;

% 偏航通道 (Iz_eff = Iz + Nr_dot = 1.849137 + 0.138)
params.xhy.yaw.Iz_eff   = 1.987137;
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

end
