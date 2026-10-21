% run_compare.m - fair comparison across all vehicles
clear; clc; close all
addpath(genpath(pwd));
params; load('cycle.mat');
tEnd = cycle(end,1);
out = sim('vehicle_base');
t = out.v.Time; v = out.v.Data; Pw = out.Pwheel.Data;
dt = mean(diff(t)); dist_km = trapz(t,v)/1000;

% ---- ICE variants (same engine curve as run_ice.m) ----
eta_drv = 0.90; Pidle = 1500; Pe_max = 70e3; Paux = 500;
LHV = 42.5e6; rho_fuel = 0.745;
loadFrac = [0.00 0.05 0.10 0.20 0.40 0.60 0.80 1.00];
eta_eng  = [0.05 0.14 0.20 0.27 0.33 0.34 0.33 0.31];

Peng = min(max(Pw,0)/eta_drv + Paux, Pe_max);
eta  = interp1(loadFrac, eta_eng, Peng/Pe_max, 'linear','extrap');
mdot_run = Peng./(eta*LHV);
mdot_idle = Pidle/(0.2*LHV);
stopped = v < 0.1 & Pw <= 0;
braking = Pw < 0 & ~stopped;

m1 = mdot_run;  m1(stopped|braking) = mdot_idle;   % plain ICE (idles)
m2 = m1;        m2(stopped) = 0;                   % + start-stop
m3 = m2;        m3(braking) = 0;                   % + start-stop + fuel cut-off
L = @(m) sum(m)*dt/rho_fuel/dist_km*100;
ICE_plain = L(m1); ICE_ss = L(m2); ICE_ssdfco = L(m3);

% ---- Load hybrid results ----
S1 = load('results/series_thermostat.mat');
S2 = load('results/series_powerfollower.mat');
P1 = load('results/parallel.mat');

names = {'ICE (idles)','ICE + start-stop','ICE + start-stop + DFCO', ...
    'Series thermostat','Series power-follower','Parallel HEV'};
L100  = [ICE_plain ICE_ss ICE_ssdfco S1.series.L100 S2.pf.L100 P1.par.L100];

fprintf('\n%-26s %8s %12s %14s\n','Vehicle','L/100km','vs ICE-plain','vs ICE+SS+DFCO');
for i = 1:numel(L100)
    fprintf('%-26s %8.2f %11.1f%% %13.1f%%\n', names{i}, L100(i), ...
        (1-L100(i)/ICE_plain)*100, (1-L100(i)/ICE_ssdfco)*100);
end

T = table(names', L100', 'VariableNames', {'Vehicle','L_per_100km'});
writetable(T, fullfile('results','comparison.csv'));

figure
bar(L100); set(gca,'XTickLabel',names,'XTickLabelRotation',25); grid on
ylabel('Fuel consumption (L/100 km)'); title('Fuel economy comparison (SOC-corrected)')
saveas(gcf, fullfile('results','comparison.png'));