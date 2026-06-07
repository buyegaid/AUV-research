clc; clear; close all;

%% =========================
%  Test for ESO raw/filt comparison
% ==========================

% 仿真时间
T_end = 100;
dt = 0.01;
t = 0:dt:T_end;
N = length(t);

% 状态初始化
Z = zeros(6,3);

% 已知输入（这里先设为0，可替换为你的实际已知加速度）
a_known = zeros(6,1);

% 重置 ESO 内部 persistent
resetFlag = true;

%% 参数设置
params = get_params();
params = params.eso;

%% 构造测试测量信号 y
% 只在第1维上加扰动，其它维保持0，便于观察
Y = zeros(6,N);

for k = 1:N
    tk = t(k);

    % 低频真实扰动成分
    low_freq = 0.8 * sin(2*pi*0.05*tk);

    % 中频成分
    mid_freq = 0.25 * sin(2*pi*0.4*tk);

    % 高频噪声/扰动
    high_freq = 0.12 * sin(2*pi*6*tk);

    % 随机噪声
    noise = 0.03 * randn;

    % 在30s之后加入阶跃扰动
    step_dist = 0;
    if tk > 30
        step_dist = 0.5;
    end

    y1 = low_freq + mid_freq + high_freq + noise + step_dist;

    Y(:,k) = zeros(6,1);
    Y(1,k) = y1;
end

%% 数据记录
z3_raw_log   = zeros(6,N);
z3_filt_log  = zeros(6,N);
y_raw_log    = zeros(6,N);
y_filt_log   = zeros(6,N);
omega0_log   = zeros(1,N);
e_log        = zeros(6,N);

%% 主循环
for k = 1:N
    y = Y(:,k);

    [Z, aux] = vec_leso_update_adv(Z, y, a_known, params, dt, resetFlag);

    % 仅首步重置
    resetFlag = false;

    % 记录
    y_raw_log(:,k)   = y;
    y_filt_log(:,k)  = aux.y_filt;
    z3_raw_log(:,k)  = aux.z3_raw;
    z3_filt_log(:,k) = aux.z3_filt;
    omega0_log(k)    = aux.omega0;
    e_log(:,k)       = aux.e;
end

%% =========================
% 作图
% ==========================

figure('Name','ESO输出滤波前后对比','Color','w');

subplot(4,1,1);
plot(t, y_raw_log(1,:), 'k-', 'LineWidth', 0.8); hold on;
plot(t, y_filt_log(1,:), 'r-', 'LineWidth', 1.2);
grid on;
xlabel('Time (s)');
ylabel('y');
title('测量信号：原始 vs 滤波');
legend('y raw','y filt');

subplot(4,1,2);
plot(t, z3_raw_log(1,:), 'b-', 'LineWidth', 0.9); hold on;
plot(t, z3_filt_log(1,:), 'm-', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('z3');
title('ESO扰动估计输出：滤波前 vs 滤波后');
legend('z3 raw','z3 filt');

subplot(4,1,3);
plot(t, z3_raw_log(1,:) - z3_filt_log(1,:), 'g-', 'LineWidth', 1.0);
grid on;
xlabel('Time (s)');
ylabel('\Delta z3');
title('滤波抑制掉的高频分量');
legend('z3 raw - z3 filt');

subplot(4,1,4);
plot(t, e_log(1,:), 'c-', 'LineWidth', 1.0); hold on;
plot(t, omega0_log, 'r--', 'LineWidth', 1.2);
grid on;
xlabel('Time (s)');
ylabel('e / \omega_0');
title('ESO误差与带宽');
legend('e','\omega_0');

%% 频谱对比（可选）
figure('Name','ESO输出频谱对比','Color','w');

fs = 1/dt;

z3_raw_1  = z3_raw_log(1,:) - mean(z3_raw_log(1,:));
z3_filt_1 = z3_filt_log(1,:) - mean(z3_filt_log(1,:));

Nfft = 2^nextpow2(N);
f = fs*(0:(Nfft/2))/Nfft;

Zraw_fft  = fft(z3_raw_1, Nfft);
Zfilt_fft = fft(z3_filt_1, Nfft);

Praw  = abs(Zraw_fft/Nfft);
Pfilt = abs(Zfilt_fft/Nfft);

Praw_half  = Praw(1:Nfft/2+1);
Pfilt_half = Pfilt(1:Nfft/2+1);

Praw_half(2:end-1)  = 2*Praw_half(2:end-1);
Pfilt_half(2:end-1) = 2*Pfilt_half(2:end-1);

plot(f, Praw_half, 'b-', 'LineWidth', 1.0); hold on;
plot(f, Pfilt_half, 'r-', 'LineWidth', 1.2);
grid on;
xlim([0 10]);
xlabel('Frequency (Hz)');
ylabel('Amplitude');
title('z3滤波前后频谱对比');
legend('z3 raw','z3 filt');
