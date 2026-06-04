% 诊断 XHY 水池实验输入链路与当前 M080/M060 模型的量级偏差。
% MATLAB R2024a: 从项目根目录或 model/test 目录均可运行。

script_dir = fileparts(mfilename('fullpath'));
project_root = fullfile(script_dir, '..', '..');
addpath(project_root, fullfile(project_root, 'Lib'), fullfile(project_root, 'guidance'), ...
    fullfile(project_root, 'controller', 'xhy'), fullfile(project_root, 'controller', 'remus'), ...
    fullfile(project_root, 'model'), fullfile(project_root, 'model', 'params'), ...
    fullfile(project_root, 'model', 'test'), fullfile(project_root, 'eso'), ...
    fullfile(project_root, 'post'), fullfile(project_root, 'traj'));

voltage_v = 24;
cases = {
    'TX6400', [6400; 0; 0; 0; 0; 0], 8.2, 'N';
    'TX8000', [8000; 0; 0; 0; 0; 0], 0.001282 * 8000, 'N';
    'MZ4000', [0; 0; 0; 0; 0; 4000], 0.000124 * 4000, 'N*m';
    'TY8000', [0; 8000; 0; 0; 0; 0], NaN, 'N';
    'TZ8000', [0; 0; 8000; 0; 0; 0], NaN, 'N';
    };

fprintf('========== XHY 水池输入链路量级诊断 ==========\n');
fprintf('链路: tau_cmd/force_cmd -> xhy_force_moment_to_pwm -> M080/M060 -> tau_thr\n');
fprintf('电压: %.1f V\n\n', voltage_v);

for i = 1:size(cases, 1)
    label = cases{i, 1};
    tau_cmd = cases{i, 2};
    target = cases{i, 3};
    unit = cases{i, 4};

    [~, tau_thr, thr, pwm_doc, info] = xhy_tau_cmd_test_step(zeros(12,1), tau_cmd, voltage_v);
    fprintf('%s\n', label);
    fprintf('  文档PWM [T1 T2 T3 T4 T5] us = [%7.1f %7.1f %7.1f %7.1f %7.1f]\n', pwm_doc);
    fprintf('  分配推力指令 [T1 T2 T3 T4 T5] g = [%7.1f %7.1f %7.1f %7.1f %7.1f]\n', info.thrust_g);
    fprintf('  推进器物理推力 [T5 T1 T2 T3 T4] N = [%7.2f %7.2f %7.2f %7.2f %7.2f]\n', thr.T_vec);
    fprintf('  生成tau [X Y Z K M N] = [%7.2f %7.2f %7.2f %7.2f %7.2f %7.2f]\n', tau_thr);

    if ~isnan(target)
        if startsWith(label, 'MZ')
            model_value = abs(tau_thr(6));
        else
            model_value = abs(tau_thr(1));
        end
        fprintf('  水池辨识目标约 %.3f %s, 当前模型/目标 = %.2fx\n', target, unit, model_value / target);
    end
    fprintf('\n');
end

u_exp = 0.675;
surge_drag_model = 0.7580 * u_exp + 3.7729 * u_exp * abs(u_exp);
surge_force_exp = 8.2;

r_exp = deg2rad(33.4);
yaw_drag_model = 0.027322 * r_exp + 0.067903 * r_exp * abs(r_exp);
yaw_drag_id = 0.765 * r_exp;

fprintf('--- 阻尼量级对比 ---\n');
fprintf('Surge: 当前阻力模型在 u=%.3f m/s 时为 %.3f N, 水池等效约 %.3f N, 需要约 %.2fx 阻力\n', ...
    u_exp, surge_drag_model, surge_force_exp, surge_force_exp / surge_drag_model);
fprintf('Yaw: 当前阻力模型在 r=33.4 deg/s 时为 %.3f N*m, 水池辨识线性阻尼约 %.3f N*m, 需要约 %.2fx 阻尼\n', ...
    yaw_drag_model, yaw_drag_id, yaw_drag_id / yaw_drag_model);

fprintf('\n诊断完成。\n');
