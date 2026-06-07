clear; close all; clc;
clear; close all; clc;

% 仿真设置
TrajMode = 1;        % 1-直线，2-圆形
CurrentModel = 1;    % 1-静态海流，2-动态海流
HeadingMode = 1;     % 1-ISMC，2-SMC
KinematicsFlag = 1;  % 1-欧拉角表示，2-四元数表示
params = get_params; % 从参数文件中获取参数

% 对比内容
ControlFlag = 1;     % 1-滑模控制，2-pid控制
HeadingMode = 2;  % SMC
useESO = 0; % 不使用ESO
hist1 = main_loop(useESO, TrajMode, CurrentModel, ControlFlag, HeadingMode,KinematicsFlag, params);
ControlFlag = 1;     % 1-滑模控制，2-pid控制
HeadingMode = 1; % iSMC
useESO = 0; % 使用ESO补偿
hist2   = main_loop(useESO, TrajMode, CurrentModel, ControlFlag,HeadingMode,KinematicsFlag, params);

compare_results(hist1, hist2,'1-smc vs 2-smc+eso');

