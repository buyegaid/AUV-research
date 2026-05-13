clc; clear; close all;

%% ===== 创建环境 =====
% 观测维度: 8
% [e_u, e_udot, u, udot, n, Kp, Ki, Kd]
%
% 动作维度: 3
% [Kp, Ki, Kd] 的归一化调节量，范围通常为 [-1, 1]
env = createEnv(8, 3);

obsInfo = getObservationInfo(env);
actInfo = getActionInfo(env);

%% ===== 创建 Actor / Critic =====
actor   = createActor(obsInfo, actInfo);
critic1 = createCritic(obsInfo, actInfo);
critic2 = createCritic(obsInfo, actInfo);

%% ===== TD3 Agent 参数 =====
agentOpts = rlTD3AgentOptions( ...
    SampleTime = 0.01, ...
    DiscountFactor = 0.99, ...
    ExperienceBufferLength = 1e6, ...
    MiniBatchSize = 128);

% Target policy smoothing
agentOpts.TargetPolicySmoothModel.StandardDeviation = 0.2;
agentOpts.TargetPolicySmoothModel.LowerLimit = -0.5;
agentOpts.TargetPolicySmoothModel.UpperLimit = 0.5;

% Exploration noise
agentOpts.ExplorationModel.StandardDeviation = 0.1;
agentOpts.ExplorationModel.StandardDeviationDecayRate = 1e-4;

% Delayed policy update
agentOpts.PolicyUpdateFrequency = 2;

% Actor optimizer
agentOpts.ActorOptimizerOptions.LearnRate = 1e-4;
agentOpts.ActorOptimizerOptions.GradientThreshold = 1;

% Critic optimizer
agentOpts.CriticOptimizerOptions(1).LearnRate = 1e-3;
agentOpts.CriticOptimizerOptions(1).GradientThreshold = 1;

agentOpts.CriticOptimizerOptions(2).LearnRate = 1e-3;
agentOpts.CriticOptimizerOptions(2).GradientThreshold = 1;

%% ===== 创建 Agent =====
agent = rlTD3Agent(actor, [critic1 critic2], agentOpts);

%% ===== 训练选项 =====
trainOpts = rlTrainingOptions( ...
    MaxEpisodes = 1000, ...
    MaxStepsPerEpisode = 1000, ...
    ScoreAveragingWindowLength = 50, ...
    Verbose = true, ...
    Plots = "training-progress", ...
    StopTrainingCriteria = "AverageReward", ...
    StopTrainingValue = 1200);

%% ===== 开始训练 =====
trainingStats = train(agent, env, trainOpts);

%% ===== 保存结果 =====
save("TD3agent_vel_260317-1.mat", "agent", "trainingStats");
