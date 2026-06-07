function [Vc_t, betaVc_t, wc_t] = gauss_markov_current(scenario, t, params)
% GAUSS_MARKOV_CURRENT 生成不同场景下的时变海流序列
%
% 场景说明:
%   1 - 均匀恒定流: Vc=0.3 m/s, betaVc=45°（基准场景）
%   2 - Gauss-Markov缓变流: 一阶Markov过程，tau_c=100s
%   3 - 深度剪切流: Vc随深度线性变化（需传入深度序列）
%   4 - 空间相关流场: Vc随位置(x,y)变化（需传入位置序列）
%
% 输入:
%   scenario: 场景编号 1-4
%   t:        时间向量 (1×N)
%   params:   参数结构体，含以下字段:
%     .Vc_mean    均值流速 (m/s)，默认0.3
%     .betaVc_mean 均值流向 (rad)，默认pi/4
%     .sigma_Vc   流速标准差，默认0.1
%     .tau_c      相关时间常数 (s)，默认100
%     .depth      深度序列 (1×N)，场景3用
%     .pos_x      x位置序列 (1×N)，场景4用
%     .pos_y      y位置序列 (1×N)，场景4用
%
% 输出:
%   Vc_t:     流速时间序列 (1×N)
%   betaVc_t: 流向时间序列 (1×N)
%   wc_t:     垂向流速时间序列 (1×N)

N  = length(t);
dt = t(2) - t(1);

% 默认参数
Vc_mean     = getfield_default(params, 'Vc_mean',     0.3);
betaVc_mean = getfield_default(params, 'betaVc_mean', pi/4);
sigma_Vc    = getfield_default(params, 'sigma_Vc',    0.1);
tau_c       = getfield_default(params, 'tau_c',       100);

switch scenario
    case 1
        % 均匀恒定流
        Vc_t     = Vc_mean * ones(1, N);
        betaVc_t = betaVc_mean * ones(1, N);
        wc_t     = zeros(1, N);

    case 2
        % Gauss-Markov缓变流（一阶Markov过程）
        % d_dot = -1/tau_c * d + sigma * sqrt(2/tau_c) * w(t)
        alpha = exp(-dt / tau_c);
        sigma_step = sigma_Vc * sqrt(1 - alpha^2);

        Vc_t     = zeros(1, N);
        betaVc_t = zeros(1, N);
        Vc_t(1)     = Vc_mean;
        betaVc_t(1) = betaVc_mean;

        % 随机种子由调用方控制（不在此处固定）
        noise_Vc    = randn(1, N) * sigma_step;
        noise_beta  = randn(1, N) * sigma_step * 0.3;  % 流向变化较小

        for k = 2:N
            Vc_t(k)     = alpha * Vc_t(k-1)     + noise_Vc(k)   + (1-alpha)*Vc_mean;
            betaVc_t(k) = alpha * betaVc_t(k-1) + noise_beta(k) + (1-alpha)*betaVc_mean;
            Vc_t(k)     = max(0, Vc_t(k));  % 流速非负
        end
        wc_t = zeros(1, N);

    case 3
        % 深度剪切流: Vc = Vc_surface * (1 - z/z_max)
        % 表层流速最大，随深度线性减小
        if isfield(params, 'depth')
            depth = params.depth;
        else
            depth = 10 * ones(1, N);  % 默认10m深度
        end
        z_max    = 50;  % 参考深度 (m)
        Vc_surf  = Vc_mean * 1.5;  % 表层流速
        Vc_t     = Vc_surf * max(0, 1 - depth / z_max);
        betaVc_t = betaVc_mean * ones(1, N);
        wc_t     = zeros(1, N);

    case 4
        % 空间相关流场: 高斯相关结构
        % Vc(x,y) = Vc_mean * (1 + A*exp(-((x-x0)^2+(y-y0)^2)/(2*L^2)))
        if isfield(params, 'pos_x') && isfield(params, 'pos_y')
            pos_x = params.pos_x;
            pos_y = params.pos_y;
        else
            pos_x = zeros(1, N);
            pos_y = zeros(1, N);
        end
        x0 = 25; y0 = 25;  % 涡流中心
        L  = 20;            % 空间相关长度 (m)
        A  = 0.5;           % 扰动幅度
        Vc_t     = Vc_mean * (1 + A * exp(-((pos_x-x0).^2 + (pos_y-y0).^2) / (2*L^2)));
        betaVc_t = betaVc_mean * ones(1, N);
        wc_t     = zeros(1, N);

    otherwise
        error('未知场景编号: %d（有效范围1-4）', scenario);
end
end

function val = getfield_default(s, field, default)
if isfield(s, field)
    val = s.(field);
else
    val = default;
end
end
