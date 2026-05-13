clc; clear; close all;

%% 创建环境
env = createEnv(10,4);

obsInfo = getObservationInfo(env);
actInfo = getActionInfo(env);

% ===== Actor / Critic =====
actor  = createActor(obsInfo, actInfo); % 创建 Actor 网络
critic1 = createCritic(obsInfo, actInfo); % 创建 Critic 网络
critic2 = createCritic(obsInfo, actInfo); % 创建第二个 Critic 网络

%% ===== TD3 Agent =====
% 采样时间和环境步长一致
agentOpts = rlTD3AgentOptions( ...
    SampleTime = 0.01, ...
    DiscountFactor = 0.99, ...
    ExperienceBufferLength = 1e6, ...
    MiniBatchSize = 128);

agentOpts.TargetPolicySmoothModel.StandardDeviation = 0.2;
agentOpts.TargetPolicySmoothModel.LowerLimit = -0.5;
agentOpts.TargetPolicySmoothModel.UpperLimit = 0.5;

agentOpts.ExplorationModel.StandardDeviation = 0.1; % 减少探索噪声
agentOpts.ExplorationModel.StandardDeviationDecayRate = 1e-4; % 加快训练曲线

agentOpts.PolicyUpdateFrequency = 2; % 延迟更新actor网络

agentOpts.ActorOptimizerOptions.LearnRate = 1e-4;
agentOpts.ActorOptimizerOptions.GradientThreshold = 1;

agentOpts.CriticOptimizerOptions(1).LearnRate = 1e-3;
agentOpts.CriticOptimizerOptions(1).GradientThreshold = 1;
agentOpts.CriticOptimizerOptions(2).LearnRate = 1e-3;
agentOpts.CriticOptimizerOptions(2).GradientThreshold = 1;

agent = rlTD3Agent(actor, [critic1 critic2], agentOpts);

%% ===== 训练选项 =====
trainOpts = rlTrainingOptions( ...
    MaxEpisodes = 1000, ...
    MaxStepsPerEpisode = 1000, ...
    ScoreAveragingWindowLength = 50, ...
    Verbose = true, ...
    Plots = "training-progress", ...
    StopTrainingCriteria = "AverageReward", ...
    StopTrainingValue = 250); %256会导致训练很慢，前期学习迟钝，更新不够灵活

%% ===== 开始训练 =====
trainingStats = train(agent, env, trainOpts);

save("TD3agent_head_260316-1.mat", "agent", "trainingStats");
