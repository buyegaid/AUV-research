%% 在Vc=0.8m/s条件下扫描omega0，验证PI-ESO优势
clear; close all; clc;
project_root = setup_paths();

fprintf('=== Vc=0.8m/s, Ks=3.0: omega0扫描验证PI-ESO优势 ===\n');
fprintf('%-8s | %-8s | %-8s | %-12s\n', 'omega0', 'LESO(m)', 'PIESO(m)', 'PIESO改善%');
fprintf('%s\n', repmat('-',1,50));

omega0_arr = [0.3, 0.5, 1.0, 2.0, 3.0];
for oi = 1:length(omega0_arr)
    leso_e  = run_sk_omega0(omega0_arr(oi), false);
    pieso_e = run_sk_omega0(omega0_arr(oi), true);
    imp = (leso_e - pieso_e) / leso_e * 100;
    fprintf('%-8.2f | %-8.3f | %-8.3f | %-12.1f\n', omega0_arr(oi), leso_e, pieso_e, imp);
end
