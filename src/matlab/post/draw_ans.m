% 配套main_loop.m的绘图脚本，绘制仿真结果
%% Tags
DrawVel = false;
DrawControlInput = false;  % 绘制控制器输入
DrawPose = true;
DrawALOS = false;
DrawCurrent = false;
DrawESO = true;
% PLOTS
scrSz = get(0, 'ScreenSize'); % [left bottom width height]
legendLocation = 'best'; legendSize = 12;

% Extract from hist.x assuming order:
% [u; v; w; p; q; r; xn; yn; zn; phi; theta; psi]

ui      = hist.ui(:,1:3);
ui_d    = hist.ui(:,4:6);
u_d = hist.ui(:,7);
nu      = hist.x(:,1:6);
u = nu(:,1); v = nu(:,2); w = nu(:,3);
p = nu(:,4); q = nu(:,5); r = nu(:,6);
if (KinematicsFlag == 1) % Euler angle representation
    eta = hist.x(:,7:12);
else % Transform the unit quaternions to Euler angles
    quaternion = simData(:,20:23);
    phi = zeros(nTimeSteps,1); theta = zeros(nTimeSteps,1); psi = zeros(nTimeSteps,1);
    for i = 1:length(t)
        [phi(i,1),theta(i,1),psi(i,1)] = q2euler(quaternion(i,:));
    end
    eta = [simData(:,17:19) phi theta psi];
end

% 期望值
theta_ref   = hist.x_d(:,1);
psi_ref     = hist.x_d(:,2);
theta_d     = hist.x_d(:,3);
psi_d       = hist.x_d(:,4);
q_d         = hist.x_d(:,5);
r_d         = hist.x_d(:,6);
y_e         = hist.x_d(:,7);
z_e         = hist.x_d(:,8);
alpha_c_hat = hist.x_d(:,9);
beta_c_hat  = hist.x_d(:,10);
d           = hist.x_d(:,11);

% 海流速度
Vc      = hist.wave(:,1);
betaVc  = hist.wave(:,2);
wc      = hist.wave(:,3);
uc = Vc .* cos(betaVc);
vc = Vc .* sin(betaVc);

% 轨迹信息
traj_pos = hist.traj;

% eso观测器输出
Z1 = hist.Z(:,1:6);
Z2 = hist.Z(:,7:12);
Z3 = hist.Z(:,13:18);
%% ALOS调试
% 绘制LOS输出的航向角
if DrawALOS
    % 绘制航向
    figure('Name','ALOS调试输出');
    if ~isoctave; set(gcf,'Position',[1, 1, scrSz(3)/3, scrSz(4)]); end
    subplot(4,1,1),plot(t,rad2deg(ssa(psi_d)),t,rad2deg(ssa(psi_ref)))
    xlabel('Time (s)'),title('平滑前后航向角 (deg)'),grid
    legend('after','initial')
    subplot(4,1,2),plot(t,rad2deg(ssa(theta_d)),t,rad2deg(ssa(theta_ref)))
    xlabel('Time (s)'),title('平滑前后俯仰角 (deg)'),grid
    legend('after','initial')
    subplot(4,1,3),plot(t,y_e,t,z_e)
    xlabel('Time (s)'),title('横向跟踪误差 (m)  '),grid
    legend('水平面','垂直面')
    subplot(4,1,4),plot(t,rad2deg(ssa(alpha_c_hat)),t,rad2deg(ssa(beta_c_hat)))
    xlabel('Time (s)'),title('参数估计值 (deg)'),grid
    legend('alpha_c_hat','beta_c_hat')
    set(findall(gcf,'type','line'),'linewidth',2)
    set(findall(gcf,'type','text'),'FontSize',14)
    set(findall(gcf,'type','legend'),'FontSize',legendSize)
    
end
%% AUV速度
% 三轴速度和三轴角速度
if DrawVel
    figure('Name','auv六轴速度图像');
    if ~isoctave; set(gcf,'Position',[scrSz(3)/3, 1, scrSz(3)/3, scrSz(4)]); end
    subplot(611),plot(t,u,t,u_d)
    xlabel('时间 (s)'),title('前速度 (m/s)'),grid
    legend('True','desire')
    subplot(612),plot(t,v)
    xlabel('时间 (s)'),title('右速度 (m/s)'),grid
    legend('True')
    subplot(613),plot(t,w)
    xlabel('时间 (s)'),title('下速度 (m/s)'),grid
    legend('True')
    subplot(614),plot(t,(180/pi)*p)
    xlabel('时间 (s)'),title('横滚角速度 (deg/s)'),grid
    legend('True')
    subplot(615),plot(t,(180/pi)*q)
    xlabel('时间 (s)'),title('俯仰角速度 (deg/s)'),grid
    legend('True')
    subplot(616),plot(t,(180/pi)*r)
    xlabel('时间 (s)'),title('航向角速度 (deg/s)'),grid
    legend('True')
    set(findall(gcf,'type','line'),'linewidth',2)
    set(findall(gcf,'type','text'),'FontSize',14)
    set(findall(gcf,'type','legend'),'FontSize',legendSize)
end
%% 位姿
if DrawPose
    % 绘制位置
    figure('Name','auv位置和跟踪距离');
    if ~isoctave; set(gcf,'Position',[2*scrSz(3)/3,scrSz(4)/2,scrSz(3)/3,scrSz(4)/2]);end
    subplot(3,1,1),plot(eta(:,2),eta(:,1),traj_pos.y,traj_pos.x)
    xlabel('E Y'),ylabel('N X'),grid
    legend('True','Desired')
    subplot(3,1,2),plot(t,eta(:,3))
    xlabel('Time (s)'),title(' d(m)'),grid
    legend('True')
    subplot(3,1,3),plot(t,d);
    xlabel('Time (s)'),title(' 跟踪距离(m)'),grid
    set(findall(gcf,'type','line'),'linewidth',2)
    set(findall(gcf,'type','text'),'FontSize',14)
    set(findall(gcf,'type','legend'),'FontSize',legendSize)
    
    figure('Name','AUV姿态跟踪');
    if ~isoctave; set(gcf,'Position',[2*scrSz(3)/3,1,scrSz(3)/3,scrSz(4)/2]);end
    subplot(3,1,1),plot(t,rad2deg(eta(:,4)))
    xlabel('Time (s)'),title('横滚角 (deg)'),grid
    legend('True')
    subplot(3,1,2),plot(t,rad2deg(eta(:,5)),t,rad2deg(theta_d))
    xlabel('Time (s)'),title('俯仰角 (deg)'),grid
    legend('True','Desired')
    subplot(3,1,3),plot(t,rad2deg(eta(:,6)),t,rad2deg(ssa(psi_d)))
    xlabel('Time (s)'),title('航向角 (deg)'),grid
    legend('True','Desired')
    set(findall(gcf,'type','line'),'linewidth',2)
    set(findall(gcf,'type','text'),'FontSize',14)
    set(findall(gcf,'type','legend'),'FontSize',legendSize)
    
end
%% 控制输入
if DrawControlInput
    figure('Name','AUV期望输入和实际输入对比');
    if ~isoctave; set(gcf,'Position',[100,scrSz(4)/2,scrSz(3)/3,scrSz(4)/2-100]); end
    % if ~isoctave; set(gcf,'Position',[2*scrSz(3)/3,scrSz(4)/2,scrSz(3)/3,scrSz(4)/2]);end
    subplot(311),plot(t,rad2deg(ui(:,1)),t,rad2deg(ui_d(:,1)))
    xlabel('Time (s)'),title('垂舵 \delta_r (deg)'),grid
    legend('True','Desired')
    subplot(312),plot(t,rad2deg(ui(:,2)),t,rad2deg(ui_d(:,2)))
    xlabel('Time (s)'),title('水平舵 \delta_s (deg)'),grid
    legend('True','Desired')
    subplot(313),plot(t,ui(:,3),t,ui_d(:,3))
    xlabel('Time (s)'),title('主推转速 n (rpm)'),grid
    legend('True','Desired')
    set(findall(gcf,'type','line'),'linewidth',2)
    set(findall(gcf,'type','text'),'FontSize',14)
end

%% 海流速度和AUV速度
if DrawCurrent
    figure('Name','auv速度和海流速度对比');
    if ~isoctave; set(gcf,'Position',[2*scrSz(3)/3,1,scrSz(3)/3,scrSz(4)/2]);end
    subplot(311),plot(t,sqrt(nu(:,1).^2+nu(:,2).^2),t,Vc)
    xlabel('时间 (s)'),grid
    legend('AUV 水平(合)速度 (m/s)','海流水平(合)速度 (m/s)',...
        'Location',legendLocation)
    subplot(312),plot(t,nu(:,3),t,wc)
    xlabel('时间(s)'),grid
    legend('AUV 纵向速度 (m/s)','海流纵向速度 (m/s)',...
        'Location',legendLocation)
    subplot(313),plot(t,rad2deg(betaVc),'r')
    xlabel('时间 (s)'),grid
    legend('流向 (deg)','Location',legendLocation)
    set(findall(gcf,'type','line'),'linewidth',2)
    set(findall(gcf,'type','text'),'FontSize',14)
    set(findall(gcf,'type','legend'),'FontSize',legendSize)
end
%% 比较eso输出和真实状态
if DrawESO
    % 检查维度
    [nSamples, nStates] = size(Z1);
    
    if size(nu,2) ~= nStates
        error('hist.Z 和 hist.X 的列数（状态维度）不一致！');
    end
    
    figure('Name','观测器观测值');
    set(gcf, 'Position', [100 100 900 600]);
    
    for i = 1:nStates
        
        subplot(nStates,1,i);
        plot(t, nu(:,i), 'b', 'LineWidth', 1.4); hold on;
        % plot(t, nu_real(:,i), 'g', 'LineWidth', 1.4); hold on;
        plot(t, Z1(:,i), 'r--', 'LineWidth', 1.4);
        
        grid on;
        ylabel(sprintf('State %d', i));
        if i == 1
            title('Comparison of true states and ESO estimated states');
        end
        if i == nStates
            xlabel('Time (s)');
        end
        
        legend(sprintf('True State %d', i), sprintf('ESO Estimate %d', i));
        
    end
    %% 绘制加速度估计项
    figure('Name','角速度项估计值');
    subplot(2,3,1),plot(t,Z3(:,1))
    xlabel('Time (s)'),title('AX'),grid
    legend('Est')
    subplot(2,3,2),plot(t,Z3(:,2))
    xlabel('Time (s)'),title('AY'),grid
    legend('Est')
    subplot(2,3,3),plot(t,Z3(:,3))
    xlabel('Time (s)'),title('AZ'),grid
    legend('Est')
    
    subplot(2,3,4),plot(t,Z3(:,4))
    xlabel('Time (s)'),title('GX'),grid
    legend('Est')
    subplot(2,3,5),plot(t,Z3(:,5))
    xlabel('Time (s)'),title('GY'),grid
    legend('Est')
    subplot(2,3,6),plot(t,Z3(:,6))
    xlabel('Time (s)'),title('GZ'),grid
    legend('Est')
    set(findall(gcf,'type','line'),'linewidth',2)
    set(findall(gcf,'type','text'),'FontSize',14)
    set(findall(gcf,'type','legend'),'FontSize',legendSize)
    
    
    E = Z1 - nu;
    
    figure('Name','ESO估计误差');
    set(gcf, 'Position', [150 150 900 600]);
    
    for i = 1:nStates
        subplot(nStates,1,i);
        plot(t, E(:,i), 'k', 'LineWidth', 1.2);
        grid on;
        ylabel(sprintf('e_%d', i));
        if i == 1
            title('ESO State Estimation Error: Z1 - true');
        end
        if i == nStates
            xlabel('Time (s)');
        end
    end
    E = Z1 - nu;
    RMSE = sqrt(mean(E.^2,1));
    MAE  = mean(abs(E),1);
    MAXE = max(abs(E),[],1);
    
    disp('===== ESO估计性能指标 =====');
    disp('RMSE = '); disp(RMSE);
    disp('MAE  = '); disp(MAE);
    disp('MAXE = '); disp(MAXE);
    
end
