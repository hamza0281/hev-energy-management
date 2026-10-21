% import_cycle.m - convert Drive Cycle Source output to [time, speed m/s]
if ~isfile(fullfile('data','cycle_synthetic.mat'))
    copyfile(fullfile('data','cycle.mat'), fullfile('data','cycle_synthetic.mat'));
end
so = sim('get_cycle');          % newer MATLAB: results are inside the output object
cyc = so.cyc;
t0 = cyc.Time; v0 = squeeze(cyc.Data);
[t0, iu] = unique(t0); v0 = v0(iu);
tt = (0:1:1369)';               % UDDS portion only (exclude the 600 s soak)
cycle = [tt, interp1(t0, v0, tt, 'linear', 0)];
save(fullfile('data','cycle_UDDS.mat'), 'cycle');
save(fullfile('data','cycle.mat'), 'cycle');     % all run scripts read this file
fprintf('Duration: %d s | Max speed: %.1f km/h | Distance: %.2f km\n', ...
    tt(end), max(cycle(:,2))*3.6, trapz(cycle(:,1),cycle(:,2))/1000);
figure; plot(cycle(:,1), cycle(:,2)*3.6); grid on
xlabel('Time (s)'); ylabel('Speed (km/h)'); title('UDDS')