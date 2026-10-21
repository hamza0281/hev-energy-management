clear; clc; close all
addpath(genpath(pwd));
params;
load('cycle.mat');
tEnd = cycle(end,1);
out = sim('vehicle_base');

figure
subplot(3,1,1)
plot(out.ref.Time, out.ref.Data*3.6, out.v.Time, out.v.Data*3.6)
legend('Target','Actual'); ylabel('Speed (km/h)'); grid on
subplot(3,1,2)
plot(out.Ftrac.Time, out.Ftrac.Data); ylabel('Wheel force (N)'); grid on
subplot(3,1,3)
plot(out.Pwheel.Time, out.Pwheel.Data/1000); ylabel('Wheel power (kW)')
xlabel('Time (s)'); grid on

err = out.ref.Data - out.v.Data;
fprintf('Max speed error: %.2f m/s | RMS error: %.3f m/s\n', ...
    max(abs(err)), sqrt(mean(err.^2)));
fprintf('Peak wheel power: %.1f kW\n', max(out.Pwheel.Data)/1000);