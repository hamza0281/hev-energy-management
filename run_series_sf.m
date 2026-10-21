% run_series_sf.m - run the Simulink/Stateflow series HEV and verify
clear; clc; close all
addpath(genpath(pwd));
params; load('cycle.mat'); load('ice_baseline.mat');
tEnd = cycle(end,1);
out = sim('vehicle_base');
Pw_in = [out.Pwheel.Time, out.Pwheel.Data(:)];     % input for series_hev_sf

SOClow = 0.45; SOChigh = 0.70; SOC0 = 0.60; Qbat = 6.5*3600;
o = sim('series_hev_sf');

t = o.soc_log.Time;  SOC = o.soc_log.Data(:);
fuel_kg = o.fuel_log.Data(end);
v = out.v.Data;      dist_km = trapz(out.v.Time, v)/1000;
fuel_L  = fuel_kg/0.745;

% same SOC correction as run_series.m
loadFrac = [0 0.05 0.10 0.20 0.40 0.60 0.80 1.00];
eta_eng  = [0.05 0.14 0.20 0.27 0.33 0.34 0.33 0.31];
eta_chain = interp1(loadFrac, eta_eng, 20e3/70e3)*0.92*0.95;
dE = (SOC0 - SOC(end))*Qbat*mean([170 185 195 202 210 220]);
L100 = (fuel_L + dE/(eta_chain*42.5e6)/0.745)/dist_km*100;

S = load('results/series_thermostat.mat');
fprintf('Simulink/Stateflow: %.2f L/100km, final SOC %.3f\n', L100, SOC(end));
fprintf('Script version:     %.2f L/100km, final SOC %.3f\n', S.series.L100, S.series.SOC(end));
fprintf('Difference: %.1f %%\n', (L100/S.series.L100-1)*100);

figure
subplot(3,1,1); plot(t,SOC); yline(SOClow,'--'); yline(SOChigh,'--'); grid on; ylabel('SOC')
subplot(3,1,2); stairs(o.eng_log.Time, o.eng_log.Data); grid on; ylabel('engOn')
subplot(3,1,3); plot(o.Pbat_log.Time, o.Pbat_log.Data/1000); grid on
ylabel('Battery (kW)'); xlabel('Time (s)')