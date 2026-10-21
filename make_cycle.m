% make_cycle.m - synthetic urban drive cycle, [time(s) speed(m/s)]
pts = [0 0; 5 0; 20 40; 40 40; 55 0; 65 0; 85 55; 105 55; ...
    125 0; 135 0; 155 70; 195 70; 220 0; 230 0];   % [time s, speed kph]
t = pts(:,1); v = pts(:,2);
for k = 1:3
    t = [t; pts(2:end,1) + k*230];
    v = [v; pts(2:end,2)];
end
tt = (0:1:t(end))';
cycle = [tt, interp1(t, v, tt)/3.6];     % speed in m/s
save(fullfile('data','cycle.mat'), 'cycle');
plot(cycle(:,1), cycle(:,2)*3.6); grid on
xlabel('Time (s)'); ylabel('Speed (km/h)'); title('Drive cycle')