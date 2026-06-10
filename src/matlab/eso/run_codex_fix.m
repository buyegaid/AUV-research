function result = run_codex_fix(cfg, tau_c_in, max_dc_in, adapt_gm)
% Codex修复方案运行器
% 2026-06-11

params = get_params();
params.ucco.tau_c = tau_c_in;
params.ucco.max_dc = max_dc_in;
clear eg_ucco_simple;

cfg.observers = {'PCRCO'};
cfg.verbose = false;
cfg.adapt_gm = adapt_gm;  % custom field

% If using adaptive GM, temporarily modify tau_c in eg_ucco_simple
% via global-like approach: save adapt_gm flag
if adapt_gm
    assignin('base', '_ucco_adapt_gm', true);
    assignin('base', '_ucco_tau_c_base', tau_c_in);
else
    assignin('base', '_ucco_adapt_gm', false);
end

r = run_observer_comparison(cfg);
result.rmse = r.PCRCO.RMSE;
result.mae = r.PCRCO.MAE;

% cleanup
evalin('base', 'clear _ucco_adapt_gm _ucco_tau_c_base');
end
