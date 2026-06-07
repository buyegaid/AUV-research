% 测试PI-ESO实现
clear; close all; clc;

% 添加路径
project_root = setup_paths();

% 获取参数
params = get_params;

% 初始化ESO状态
Z = zeros(6, 3);
x = [1; 0; 0; 0; 0; 0; 0; 0; 10; 0; 0; 0];
a_known = zeros(6, 1);
dt = 0.01;

% 测试PI-ESO更新
fprintf('测试PI-ESO更新...\n');
[Z_next, aux] = vec_pieso_update(Z, x(1:6), a_known, params.pieso, dt);

fprintf('PI-ESO测试成功!\n');
fprintf('  tau_c = %.1f s\n', params.pieso.tau_c);
fprintf('  Lambda对角元素 = %.4f\n', 1/params.pieso.tau_c);
fprintf('  Z3输出范围: [%.3f, %.3f]\n', min(Z_next(:,3)), max(Z_next(:,3)));
fprintf('  自适应带宽: %.2f\n', aux.omega0);
