function hist = main_loop_remus(useESO, TrajMode,CurrentModel,ControlFlag,HeadingMode,KinematicsFlag,params)
%%  useESO: 是否使用ESO进行补偿
% TrajMode: 轨迹模式 (1-直线，2-圆形)
% CurrentModel: 海流模型 (1-静态，2-动态)
% ControlFlag: 控制器选择 (1-滑模控制，2-PID控制)
% KinematicsFlag: 运动学表示 (1-欧拉角，2-四元数)
clear my_ALOS3D
clear my_integralSMCheading
clear my_integralSMCpitch
clear my_SMCsurge
clear vec_leso_update_adv
%% 参数设置

% 初始化
T = 500;                % 仿真时间（s）
h = 0.01;               % 时间步长（s） Time step (seconds)
t = 0:h:T;              % 时间向量 Time vector
nTimeSteps = length(t); % 时间步的数量

% 初始化AUV的状态变量（x代表AUV的真实状态，x_hat代表估计状态）
xn = 300; yn = 0; zn = 10;          % 初始北东地位置 (m)
phi = 0; theta = 0; psi = -pi/2;    % 初始欧拉角 (rad)
U = 1;                              % 初始前向速度 (m/s)

% 状态向量初始化
if KinematicsFlag == 1  % 欧拉角表示
    x = [U; zeros(5,1); xn; yn; zn; phi; theta; psi];
else % 四元数表示
    quat = euler2q(phi, theta, psi);
    x = [U; zeros(5,1); xn; yn; zn; quat];
end
%   u:       Surge velocity          (m/s)
%   v:       Sway velocity           (m/s)
%   w:       Heave velocity          (m/s)
%   p:       Roll rate               (rad/s)
%   q:       Pitch rate              (rad/s)
%   r:       Yaw rate                (rad/s)
%   x:       North position          (m)
%   y:       East position           (m)
%   z:       Downwards position      (m)
%   phi:     Roll angle              (rad)
%   theta:   Pitch angle             (rad)
%   psi:     Yaw angle               (rad)

% 初始化控制和状态设置 Initial control and state setup
theta_d = pi/2; q_d = 0; q_dd = 0;     % 初始化俯仰参考
psi_d = psi; r_d = 0; r_dd = 0;        % 初始化航向参考
xf_z_d = zn;                           % 初始化低通滤波器状态（深度控制需要做低通滤波）
u_c = 1.5; u_dd = 0;                   % 初始化前向速度参考
u_d = U;
n = 600;                       % 初始转速 (RPM)
n_d = 1300;                    % 期望转速 (RPM)

% 初始化海流参数
Vc = params.current.Vc;      % 流速(m/s)
betaVc = params.current.betaVc; % 流向(rad)
wc = params.current.wc;      % 垂直流速(m/s)

% 初始化推进器参数
n_rate = params.auv.n_rate;        % 速率更新限制 (RPM per update)
n_max = params.auv.n_max;          % 最大推进器速度 (RPM)
delta_max = params.auv.delta_max;  % 舵机最大角度（rad）
r_max = params.auv.r_max;          % 最大允许转弯速率 Maximum allowable rate of turn (rad/s)
q_max = params.auv.q_max;          % 最大允许俯仰速率 Maximum allowable pitch rate (rad/s)

% PID控制参数设置
z_max = 100;                   % 最大深度 (m)
psi_step = deg2rad(-60);       % 航向角最大变化角度 (rad)
z_step = 30;                   % 深度最大步长, max 100 m

% 积分状态记录
z_int = 0;                     % 深度积分
theta_int = 0;                 % 俯仰积分
psi_int = 0;                   % 航向积分

% 深度控制器（连续闭环）
z_d = zn;                      % 初始目标深度 (m)
wn_d_z = 0.02;                 % 深度控制的自然频率Natural frequency for depth control
Kp_z = 0.1;                    % K增益 Proportional gain for depth
T_z = 100;                     % 积分作用的时间常数 Time constant for integral action in depth control
k_grad = 0.1;                  % 俯仰积分增益 Gain for computation of theta_d

[~,~,Mauv] = remus100();                  % Remus 100 质量矩阵
w_theta = 0.8;                            % 俯仰轴自然频率 Natural frequency in pitch (rad/s)
Kp_theta = Mauv(5,5) * w_theta^2;         % 俯仰控制的比例P增益 Proportional gain for pitch control
Kd_theta = Mauv(5,5) * 2 * 0.8 * w_theta; % 俯仰控制的微分D增益 Derivative gain for pitch control
Ki_theta = Kp_theta * w_theta / 10;       % 俯仰控制的积分I增益 Integral gain for pitch control

% SMC控制参数
% 三个维度，分别对应三列
smc_params.heading = params.heading;
smc_params.pitch = params.pitch;
smc_params.surge = params.surge;
smc_params.surge = params.surge; % 从参数文件中获取速度控制参数
% smc_params.heading.K_d = 0;               % Derivative gain inactive in SMC mode
% smc_params.heading.K_sigma = 0.05;        % Sliding mode control gain
% smc_params.heading.lambda = 0.1;          % 滑模参数
% smc_params.heading.phi_b = 0.1;           % Boundary layer thickness
% smc_params.heading.K_nomoto = 5/20;       % 增益 Gain, max pitch rate over max stern plane angle
% smc_params.heading.T_nomoto = 1;         % 时间常数 Time constant for pitch dynamics
% smc_params.pitch.K_d = 0;
% smc_params.pitch.K_sigma = 0.05;
% smc_params.pitch.lambda = 0.1;
% smc_params.pitch.phi_b = 0.1;
% smc_params.pitch.K_nomoto = 5/20;
% smc_params.pitch.T_nomoto = 1;
% smc_params.surge.K_d = 0.35;
% smc_params.surge.K_sigma = 0.05;
% smc_params.surge.lambda = 0.1;
% smc_params.surge.phi_b = 0.1;
% smc_params.surge.K_nomoto = 1/31.3;
% smc_params.surge.T_nomoto =1.04;
smc_params.surge.K_T = params.auv.K_T;                      % N/s^2 % 在巡航速度附近线性化
smc_params.surge.m = params.auv.m;                    % 质量 Mass 31.9kg
smc_params.surge.Xu_dot = params.auv.Xu_dot;                    % Added mass in surge -5.5kg
smc_params.surge.d1 = params.auv.d1;                         % 线性阻尼 Linear drag
smc_params.surge.d2 = params.auv.d2;                         % 二次阻尼 Quadratic drag

% 速度参考模型
surge_omega = 0.12;
surge_zetta = 1.2;

% 3DALOS路径跟踪参数
alos_params = params.alos; % 从参数文件中获取ALOS参数
% alos_params.delta_h = 20;            % 水平前视距离 (m)
% alos_params.delta_v = 20;            % 垂直前视距离 (m)
% alos_params.gamma_h = 0.002;         % 自适应增益，水平方向
% alos_params.gamma_v = 0.002;         % 自适应增益，垂直方向
% alos_params.M_theta = deg2rad(20);   % 最大估计角 (rad)
% alos_params.R_switch = 10;           % 航点切换半径 (m)
% alos_params.K_f = 0.5;               % LOS观测器的增益 LOS observer gain

% LESO 参数列表
eso_params = params.eso; % 从参数文件中获取ESO参数
if useESO
    smc_params.heading.rho_Eso = 1.0; % 完全补偿
else
    smc_params.heading.rho_eso = 0.0; % 不使用ESO时，设置rho_eso为0
end

% 初始 ESO状态变量
% Z(6x3): Z1 估计速度变量, Z2 est nu_dot, Z3 est disturbance
Z = zeros(6,3);
Z(:,1) = x(1:6);
a_known = zeros(6,1); % 已建模加速度

% 数据存储 storage
hist.t = t(:);
hist.useESO = useESO;
hist.x = zeros(length(t), length(x)); % 状态变量
hist.x_d = zeros(length(t),11);       % 参考信号
hist.ui = zeros(length(t),7);         % 控制输入
hist.Z = zeros(length(t),18);         % 6 * 3 维向量
hist.tau = zeros(length(t),6);        % 力矩
hist.wave = zeros(length(t),3);
hist.e_psi = zeros(length(t),1);
hist.e_y = zeros(length(t),1);
hist.sigma_heading = zeros(length(t),1);
hist.hat_dr = zeros(length(t),1);

% 输入范围检查
rangeCheck(n_d, 0, n_max);
rangeCheck(z_step, 0, z_max);
rangeCheck(U, 0, 5);

% 获得轨迹
if TrajMode == 1
    pts = line_traj([xn;yn;zn],0,50,50);
else
    pts = traj(50, 50);
end
hist.traj.x = [xn;pts.pos.x];
hist.traj.y = [yn;pts.pos.y];
hist.traj.z = [zn;pts.pos.z];

timebar(1, nTimeSteps, 'AUV轨迹跟踪'); % 显示进度条

%% 主循环
for i = 1:nTimeSteps
    
    % 1. 观测
    u = x(1);                  % Surge velocity (m/s)
    v = x(2);                  % Sway veloiciy (m/s)
    w = x(3);                  % Heave velocity (m/s)
    q = x(5);                  % Pitch rate (rad/s)
    r = x(6);                  % Yaw rate (rad/s)
    xn = x(7);                 % North position (m)
    yn = x(8);                 % East position (m)
    zn = x(9);                 % Down position (m), depth
    switch KinematicsFlag % Kinematic representation 运动学表示
        case 1
            phi = x(10); theta = x(11); psi = x(12); % 欧拉角
        otherwise
            [~,theta,psi] = q2euler(x(10:13)); % 四元数转换为欧拉角
    end
    
    % 海流状态更新：模拟海流的动态变化，并根据仿真时间逐步更新水平海流速度Vc和方向betaVc
    if CurrentModel ==2
        if i > 4000
            Vc_d = 0.65;
            w_V = 0.05;
            Vc = exp(-h*w_V) * Vc + (1 - exp(-h*w_V)) * Vc_d;
        else
            Vc = 0.5;
        end
        
        if i > 2000
            betaVc_d = deg2rad(160);
            w_beta = 0.1;
            betaVc = exp(-h*w_beta) * betaVc + (1 - exp(-h*w_beta)) * betaVc_d;
        else
            betaVc = deg2rad(150);
        end
        
        betaVc = betaVc + randn / 1000;
        Vc = Vc + 0.002 * randn;
    end
    % LESO start
    % 3. 测量，在真值基础上增加偏差
    % meas_noise_std = [0.5;0.5;0.5;0.1;0.1;0.1]; % 测量噪声，来自于传感器
    % y = x(1:6) + meas_noise_std .* 0.1;
    y = x(1:6);
    % 4. 更新ESO
    [Z, aux] = vec_leso_update_adv(Z, y, a_known, eso_params, h);
    hat_dr = Z(6,3); % disturbance estimate for yaw acceleration
    
    % 制导 ALOS3D
    [psi_ref, theta_ref, y_e, z_e, alpha_c_hat, beta_c_hat,d] = ...
        my_ALOS3D(xn, yn, zn, h, pts, alos_params);
    
    % 航向角和俯仰角滤波
    [theta_d, q_d] = LOSobserver(theta_d, q_d, theta_ref, h, alos_params.K_f);
    [psi_d, r_d] = LOSobserver(psi_d, r_d, psi_ref, h, alos_params.K_f);
    
    % 速度限幅
    r_d = sat(r_d, r_max);
    q_d = sat(q_d, q_max);
    
    % 计算控制输入
    switch ControlFlag
        case 1 % 滑模控制
            % 控制器 SMC
            if HeadingMode == 2
                delta_r_d = SMCheading(psi, r, psi_d, r_d, r_dd, hat_dr, h, smc_params.heading);
            elseif HeadingMode == 1
                delta_r_d = my_integralSMCheading(psi, r, psi_d, r_d, r_dd, h, smc_params.heading);
            end
            
            delta_s_d = my_integralSMCpitch(theta, q, theta_d, q_d, q_dd, h, smc_params.pitch);
            
            % 速度控制器SMC
            %[u_d,u_dd] = surge_ref_model(u_c,u_d, u_dd,surge_omega,surge_zetta,h);
            % n_d = my_SMCsurge(u, u_d, u_dd, h, smc_params.surge) * 60;
            
            % n_d = 1000; % 目标转速保持不变
            % 螺旋桨转速更新：根据目标转速n_d和当前转速n更新螺旋桨转速，并确保螺旋桨转速不超过最大值
            % if n < n_d
            %     n = n + n_rate;
            % elseif n > n_d
            %     n = n - n_rate;
            % end
            
            delta_r = sat(delta_r_d, delta_max); % 垂直舵控制（航向轴）
            delta_s = sat(delta_s_d, delta_max); % 水平舵控制（俯仰轴）
            n = sat(n, n_max);                   % 转速限幅
            
            % 计算控制输入
            ui = [delta_r -delta_s 1000]';
            
        case 2 % PID控制
            % 水平舵控制(俯仰轴)
            delta_s = -Kp_theta * ssa(theta - theta_d)- Kd_theta * q - Ki_theta * theta_int;
            delta_s = sat(delta_s, delta_max);
            % 低通滤波
            [xf_z_d, z_d] = lowPassFilter(xf_z_d, z_ref, wn_d_z, h);
            % 外环残差表示AUV实际深度与目标深度之间的误差。根据这个误差计算出控制信号
            sigma = -u * sin(theta) + w * cos(theta) + Kp_z * ((zn - z_d) + (1/T_z) * z_int);
            % 通过最小误差函数更新参考俯仰角
            if abs(w/u) > 0.176 % theta > 10 deg
                % Large dive speeds
                gradJ = -(u * cos(theta) + w * sin(theta));
            else
                % Simplified gradient when theta < 10 deg
                gradJ = -sign(u);
            end
            theta_d = theta_d - h * k_grad * gradJ * sigma;
            
            % 垂直舵控制（航向轴）是用来一个积分滑模算法SMC
            delta_r = integralSMCheading(psi, r, psi_d, r_d, a_d, K_d, K_sigma, lambda, phi_b, K_yaw, T_yaw, h);
            delta_r = sat(delta_r, delta_max);
            
            % 航向角的三阶参考更新
            [psi_d, r_d, a_d] = refModel(psi_d, r_d, a_d, psi_ref, r_max, zeta_d_psi, wn_d_psi, h, 1);
            % 转速更新
            % 螺旋桨转速更新：根据目标转速n_d和当前转速n更新螺旋桨转速，并确保螺旋桨转速不超过最大值
            if n < n_d
                n = n + n_rate;
            elseif n > n_d
                n = n - n_rate;
            end
            n = sat(n, n_max);
            % 计算控制输入
            ui = [delta_r -delta_s n]';
    end
    % 保存误差和诊断量
    e_psi = ssa(psi - psi_d);
    sigma_heading = r - (r_d - 2 * smc_params.heading.lambda * e_psi ...
        - smc_params.heading.lambda^2 * psi_int);
    
    hist.x(i,:) = x';
    hist.x_d(i,:) = [theta_ref,psi_ref,theta_d,psi_d,q_d,r_d,y_e,z_e,alpha_c_hat,beta_c_hat,d];
    hist.ui(i,:) = [ui' delta_r_d -delta_s_d n_d u_d];
    hist.wave(i,:) = [Vc betaVc wc];
    hist.Z(i,:) = [Z(:,1)' Z(:,2)' Z(:,3)'];
    hist.tau(i,:) = a_known';
    hist.e_psi(i) = e_psi;
    hist.e_y(i) = y_e;
    hist.sigma_heading(i) = sigma_heading;
    hist.hat_dr(i) = hat_dr;
    
    % 真实状态更新
    if (KinematicsFlag == 1)
        % Euler angles x = [ u v w p q r x y z phi theta psi ]'
        % 使用rk4求解器求解（四阶龙格-库塔法）
        [xdot,U,M,C,D,g,tau,a_known] = remus100(x, ui, Vc, betaVc, wc);
        
        x = rk4(@remus100, h, x, ui, Vc, betaVc, wc);  % RK4 method x(k+1) % 模型计算的状态
        % 保证航向不发散
        x(12) = ssa(x(12));
    else
        % Unit quaternions x = [ u v w p q r x y z eta eps1 eps2 eps3 ]'
        % 如果使用四元数，用欧拉法来更新状态
        xdot = remus100(x, ui, Vc, betaVc, wc);
        quat = x(10:13);                            % Unit quaternion
        x(1:6) = x(1:6) + h * xdot(1:6);            % Forward Euler
        x(7:9) = x(7:9) + h * Rquat(quat) * x(1:3); % Backward Euler
        quat = expm(Tquat(x(4:6)) * h) * quat;      % Exact quat. discretization
        x(10:13) = quat / norm(quat);               % Normalization
    end
    
    % Euler's integration method (k+1)
    % 欧拉积分法（k+1）
    z_int = z_int + h * ( zn - z_d );                   % 深度误差积分
    theta_int = theta_int + h * ssa( theta - theta_d ); % 俯仰误差积分
    psi_int = psi_int + h * ssa( psi - psi_d );         % 航向误差积分
    
    timebar; % 更新进度条
end

end