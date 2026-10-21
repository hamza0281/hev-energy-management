% run_ice.m - conventional ICE vehicle fuel economy (backward-facing)
clear; clc; close all
addpath(genpath(pwd));
params; load('cycle.mat');
tEnd = cycle(end,1);
out = sim('vehicle_base');

t   = out.v.Time;
v   = out.v.Data;                 % m/s
Pw  = out.Pwheel.Data;            % W at the wheels
dt  = mean(diff(t));

% ---- Powertrain parameters ----
eta_drv  = 0.90;                  % transmission + driveline efficiency
Pidle    = 1500;                  % W, fuel-equivalent idle loss
Pmax_eng = 70e3;                  % W
LHV      = 42.5e6;                % J/kg, gasoline
rho_fuel = 0.745;                 % kg/L
Paux     = 500;                   % W, accessories

% ---- Engine efficiency vs load (simple curve) ----
% load fraction -> brake thermal efficiency
loadFrac = [0.00 0.05 0.10 0.20 0.40 0.60 0.80 1.00];
eta_eng  = [0.05 0.14 0.20 0.27 0.33 0.34 0.33 0.31];

Peng = max(Pw,0)/eta_drv + Paux;           % engine mechanical power needed
Peng = min(Peng, Pmax_eng);                % engine power limit
stopped = v < 0.1 & Pw <= 0;               % engine idles at standstill
eta  = interp1(loadFrac, eta_eng, Peng/Pmax_eng, 'linear','extrap');
mdot = Peng ./ (eta*LHV);                  % kg/s fuel
mdot(stopped) = Pidle/(0.2*LHV);           % idle fuel flow
mdot(Pw<0 & ~stopped) = Pidle/(0.2*LHV);   % engine at idle while braking

fuel_kg = sum(mdot)*dt;
fuel_L  = fuel_kg/rho_fuel;
dist_km = trapz(t,v)/1000;
fprintf('Distance: %.2f km\n', dist_km);
fprintf('Fuel used: %.3f L\n', fuel_L);
fprintf('ICE fuel economy: %.2f L/100km  (%.1f km/L)\n', ...
    fuel_L/dist_km*100, dist_km/fuel_L);

% Save as the baseline to beat
ice.L100 = fuel_L/dist_km*100; ice.kmpl = dist_km/fuel_L;
save(fullfile('results','ice_baseline.mat'),'ice');

figure
plot(t, Peng/1000); grid on
xlabel('Time (s)'); ylabel('Engine power (kW)'); title('ICE power demand')