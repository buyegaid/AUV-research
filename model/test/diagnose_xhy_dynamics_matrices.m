% 诊断 XHY 动力学矩阵、局部时间常数和静力恢复项。
% 运行方式:
%   cd model/test
%   diagnose_xhy_dynamics_matrices

script_dir = fileparts(mfilename('fullpath'));
project_root = fullfile(script_dir, '..', '..');
addpath(project_root, fullfile(project_root, 'Lib'), fullfile(project_root, 'guidance'), ...
    fullfile(project_root, 'controller', 'xhy'), fullfile(project_root, 'controller', 'remus'), ...
    fullfile(project_root, 'model'), fullfile(project_root, 'model', 'params'), ...
    fullfile(project_root, 'model', 'test'), fullfile(project_root, 'eso'), ...
    fullfile(project_root, 'post'), fullfile(project_root, 'traj'));

%% 当前 XHY 参数，与 model/xhy.m 保持一致
mu = 63.446827;
g_mu = gravity(mu);
rho = 1000;

m = 85.832;
Ix = 0.553864787 + 0.865274;
Iy = 2.162341935 + 5.011187;
Iz = 1.849137 + 4.541468;

r_bG = [0 0 0]';
r_bB = [0 0 -0.12]';
GM_eff = abs(r_bB(3) - r_bG(3));
W = m * g_mu;
B = W;
K_h = W * GM_eff;

MRB = diag([m m m Ix Iy Iz]);
MA = diag([15.81 124.73 42.87 0.014 0.041 0.123]);
M_manual = MRB + MA;

%% 通过 xhy 主模型读取当前矩阵
x0 = zeros(12,1);
ui0 = zeros(5,1);
[~, ~, M, C0, D0, g0, tau0] = xhy(x0, ui0, 0, 0, 0);

%% CFD 阻尼系数与工作点线性化
d1 = [0.7580; 0; 1.4485; 0; 0.001345; 0.027322];
d2 = [3.7729; 130.7023; 45.0442; 0; 0.00453; 0.067903];
nu_ref = [0.675; 0.70; 0.38; 0; 0; deg2rad(33.4)];
D_eff_ref = d1 + 2 .* d2 .* abs(nu_ref);

T_zero = nan(6,1);
idx_zero = d1 > 0;
M_diag = diag(M);
T_zero(idx_zero) = M_diag(idx_zero) ./ d1(idx_zero);

T_ref = nan(6,1);
idx_ref = D_eff_ref > 0;
T_ref(idx_ref) = M_diag(idx_ref) ./ D_eff_ref(idx_ref);

%% Roll/Pitch 小角度二阶模型
wn_roll = sqrt(K_h / M(4,4));
wn_pitch = sqrt(K_h / M(5,5));
zeta_roll = 0;
zeta_pitch = d1(5) / (2 * sqrt(K_h * M(5,5)));
period_roll = 2 * pi / wn_roll;
period_pitch = 2 * pi / wn_pitch;

%% 结构性质检查
eig_MRB = eig(MRB);
eig_MA = eig(MA);
eig_M = eig(M);

rng(20260606);
x_rand = zeros(12,1);
x_rand(1:6) = [0.31; -0.22; 0.17; 0.04; -0.03; 0.08];
[~, ~, ~, C_rand] = xhy(x_rand, ui0, 0, 0, 0);
nu_rand = x_rand(1:6);
c_power = nu_rand' * C_rand * nu_rand;

x_roll = zeros(12,1);
x_roll(10) = deg2rad(5);
[~, ~, ~, ~, ~, g_roll] = xhy(x_roll, ui0, 0, 0, 0);

x_pitch = zeros(12,1);
x_pitch(11) = deg2rad(5);
[~, ~, ~, ~, ~, g_pitch] = xhy(x_pitch, ui0, 0, 0, 0);

roll_restoring_ok = g_roll(4) > 0;
pitch_restoring_ok = g_pitch(5) > 0;

%% REMUS100 参考量级
remus.mass_kg = 31.9;
remus.T1_s = 20;
remus.T2_s = 20;
remus.T6_s = 1;

%% 输出报告
fprintf('========== XHY 动力学矩阵诊断 ==========\n');
fprintf('水密度 rho = %.1f kg/m^3\n', rho);
fprintf('质量 m = %.3f kg, W = B = %.3f N\n', m, W);
fprintf('等效稳心/浮心恢复力臂 GM_eff = %.3f m, K_h = W*GM_eff = %.3f N*m/rad\n\n', GM_eff, K_h);

fprintf('MRB diag = [%s]\n', join_num(diag(MRB)));
fprintf('MA  diag = [%s]\n', join_num(diag(MA)));
fprintf('M   diag = [%s]\n', join_num(diag(M)));
fprintf('M 与手工矩阵最大差值 = %.3e\n', max(abs(M(:) - M_manual(:))));
fprintf('min eig(MRB)=%.6g, min eig(MA)=%.6g, min eig(M)=%.6g\n\n', ...
    min(eig_MRB), min(eig_MA), min(eig_M));

fprintf('D(0) diag = [%s]\n', join_num(diag(D0)));
fprintf('D_eff 工作点 diag = [%s]\n', join_num(D_eff_ref));
fprintf('参考工作点 nu_ref = [u %.3f, v %.3f, w %.3f, p %.3f, q %.3f, r %.3f] (SI)\n\n', nu_ref);

fprintf('一阶速度时间常数 T=Mii/D:\n');
print_time_constant('Surge u', T_zero(1), T_ref(1), '可用一阶速度模型');
print_time_constant('Sway v', T_zero(2), T_ref(2), '零速无线性阻尼，仅报告工作点');
print_time_constant('Heave w', T_zero(3), T_ref(3), '可用一阶速度模型');
print_time_constant('Yaw r', T_zero(6), T_ref(6), 'yaw-rate 一阶；psi 为积分，不是二阶 Nomoto');
fprintf('\n');

fprintf('Roll/Pitch 小角度二阶模型:\n');
fprintf('  Roll:  M44=%.4f, K=%.3f, wn=%.3f rad/s, period=%.3f s, zeta=%.3e\n', ...
    M(4,4), K_h, wn_roll, period_roll, zeta_roll);
fprintf('  Pitch: M55=%.4f, K=%.3f, wn=%.3f rad/s, period=%.3f s, zeta=%.3e\n\n', ...
    M(5,5), K_h, wn_pitch, period_pitch, zeta_pitch);

fprintf('结构性质检查:\n');
fprintf('  nu''*C*nu = %.3e (应接近 0)\n', c_power);
fprintf('  g(0) = [%s]\n', join_num(g0));
fprintf('  tau(0) = [%s]\n', join_num(tau0));
fprintf('  +5deg roll:  g4=%.6f, 恢复方向=%s\n', g_roll(4), pass_fail(roll_restoring_ok));
fprintf('  +5deg pitch: g5=%.6f, 恢复方向=%s\n\n', g_pitch(5), pass_fail(pitch_restoring_ok));

fprintf('REMUS100 参考: m≈%.1f kg, T1=%.1f s, T2=%.1f s, T6=%.1f s\n', ...
    remus.mass_kg, remus.T1_s, remus.T2_s, remus.T6_s);
fprintf('XHY/REMUS 质量比 = %.2f\n', m / remus.mass_kg);
fprintf('诊断完成。\n');

function s = join_num(v)
parts = arrayfun(@(x) sprintf('%.6g', x), v(:), 'UniformOutput', false);
s = strjoin(parts, ', ');
end

function s = pass_fail(tf)
if tf
    s = 'PASS';
else
    s = 'FAIL';
end
end

function print_time_constant(name, t_zero, t_ref, note)
if isnan(t_zero)
    zero_text = 'N/A';
else
    zero_text = sprintf('%.3f s', t_zero);
end
if isnan(t_ref)
    ref_text = 'N/A';
else
    ref_text = sprintf('%.3f s', t_ref);
end
fprintf('  %-8s T_zero=%-10s T_ref=%-10s %s\n', name, zero_text, ref_text, note);
end
