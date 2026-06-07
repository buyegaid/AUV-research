% extract_timeline_v2.m - corrected: mode=1 manual, mode=4 auto station-keeping
clear; clc;

T = readtable('data/raw/debug_data260601-1.csv');
t = T.pc_timestamp - T.pc_timestamp(1);
N = length(t);

force_cmd = [T.force_cmd1, T.force_cmd2, T.force_cmd3, T.force_cmd4, T.force_cmd5, T.force_cmd6];
mode = T.mode;
ch_names = {'TX(Surge)','TY(Sway)','TZ(Heave)','MX(Roll)','MY(Pitch)','MZ(Yaw)'};

bj_h = 15; bj_m = 13; bj_s = 59.0;
bj0 = bj_h*3600 + bj_m*60 + bj_s;

u = T.linear_vel_x; v = T.linear_vel_y; w = T.linear_vel_z; r_deg = T.angular_vel_z;
P = T.control_voltage .* T.power_current;

fprintf('===== Timeline (mode=1 manual, mode=4 auto station-keeping) =====\n\n');

% Find mode transitions
mode_changes = find(diff(mode) ~= 0);
fprintf('Mode transitions: %d\n', length(mode_changes));
fprintf('\n%-4s %-5s %-18s %-18s %-7s %s\n', '#','Mode','Start','End','Dur(s)','Description');
fprintf('%s\n', repmat('-',1,100));

seg_num = 0;
i = 1;
while i <= N
    m = mode(i);
    t_start = t(i);
    j = i;
    while j <= N && mode(j) == m
        j = j + 1;
    end
    j = j - 1;
    t_end = t(j);
    dur = t_end - t_start;

    if dur > 3
        seg_num = seg_num + 1;
        ts_bj = bj0 + t_start; te_bj = bj0 + t_end;
        ts_h = floor(ts_bj/3600); ts_m = floor(mod(ts_bj,3600)/60); ts_s = mod(ts_bj,60);
        te_h = floor(te_bj/3600); te_m = floor(mod(te_bj,3600)/60); te_s = mod(te_bj,60);

        idx = i:j;
        fc = force_cmd(idx,:); total_f = sum(abs(fc),2);

        if m == 1
            if mean(total_f) < 200
                desc = 'idle (surface standby)';
            else
                ch_energy = sum(abs(fc),1);
                [~, dom] = max(ch_energy);
                dom_names = {'Surge_FwdBwd','Sway_LftRgt','Heave_UpDwn','Roll','Pitch','Yaw_Turn'};
                desc = ['manual: ' dom_names{dom}];
            end
        elseif m == 3
            desc = 'mode3: high-thrust mixed';
        elseif m == 4
            desc = 'AUTO station-keeping';
        else
            desc = sprintf('unknown mode %d', m);
        end

        fprintf('%-4d %-5d %02d:%02d:%04.1f - %02d:%02d:%04.1f  %-7.0f %s\n', ...
            seg_num, m, ts_h, ts_m, ts_s, te_h, te_m, te_s, dur, desc);

        if dur > 10
            fprintf('         |u|max=%.2f |v|max=%.2f |w|max=%.2f |r|max=%.1f deg/s  P=%.0fW\n', ...
                max(abs(u(idx))), max(abs(v(idx))), max(abs(w(idx))), max(abs(r_deg(idx))), mean(P(idx)));
            fprintf('         cmd: TX=%d TY=%d TZ=%d MZ=%d MY=%d\n', ...
                round(max(abs(fc(:,1)))), round(max(abs(fc(:,2)))), round(max(abs(fc(:,3)))), ...
                round(max(abs(fc(:,6)))), round(max(abs(fc(:,5)))));
        end
    end
    i = j + 1;
end

% Summary by mode
fprintf('\n=== Mode Summary ===\n');
for m = unique(mode)'
    mask = mode == m;
    dur_min = sum(mask)*median(diff(t))/60;
    fprintf('Mode %d: %.1f min (%.1f%%)\n', m, dur_min, sum(mask)/N*100);
    fc_m = force_cmd(mask,:);
    fprintf('  max cmd: TX=%d TY=%d TZ=%d MZ=%d\n', ...
        round(max(abs(fc_m(:,1)))), round(max(abs(fc_m(:,2)))), round(max(abs(fc_m(:,3)))), round(max(abs(fc_m(:,6)))));
end

% Manual test summary
fprintf('\n=== Manual Tests (mode=1, active) ===\n');
mask_man = mode==1 & sum(abs(force_cmd),2) > 500;
fprintf('Active manual: %.1f min\n', sum(mask_man)*median(diff(t))/60);
fprintf('Max speed: u=%.2f v=%.2f w=%.2f r=%.1f deg/s\n', ...
    max(abs(u(mask_man))), max(abs(v(mask_man))), max(abs(w(mask_man))), max(abs(r_deg(mask_man))));
