% run_series.m - series HEV, thermostat (ON/OFF) control, backward-facing
clear; clc; close all
addpath(genpath(pwd));
params; load('cycle.mat'); load('ice_baseline.mat');
tEnd = cycle(end,1);
out = sim('vehicle_base');
t = out.v.Time; v = out.v.Data; Pw = out.Pwheel.Data;
dt = mean(diff(t)); N = numel(t);

% ---- Parameters ----
eta_m = 0.92; eta_g = 0.92; eta_inv = 0.95;   % motor, generator, power electronics
Paux = 500;  Pregen_max = 20e3;               % W
Pe_on = 20e3;  Pe_max = 70e3;                 % engine ON power, engine max (W)
LHV = 42.5e6; rho_fuel = 0.745;
loadFrac = [0.00 0.05 0.10 0.20 0.40 0.60 0.80 1.00];
eta_eng  = [0.05 0.14 0.20 0.27 0.33 0.34 0.33 0.31];
Qbat = 6.5*3600; Rint = 0.15;                 % As, ohm
socTab = [0 0.2 0.4 0.6 0.8 1];
vocTab = [170 185 195 202 210 220];           % pack open-circuit voltage (V)
SOC0 = 0.60; SOClow = 0.45; SOChigh = 0.70;   % thermostat thresholds

% ---- Simulation loop ----
SOC = zeros(N,1); SOC(1) = SOC0;
Peng = zeros(N,1); Pbat = zeros(N,1); Ibat = zeros(N,1); mdot = zeros(N,1);
engOn = false;
for k = 1:N
    % Traction motor electrical power (positive = motoring)
    if Pw(k) >= 0
        Pmot = Pw(k)/eta_m;
    else
        Pmot = max(Pw(k)*eta_m, -Pregen_max);   % friction brakes take the rest
    end
    Pbus = Pmot + Paux;

    % Thermostat controller with hysteresis
    if SOC(k) < SOClow
        engOn = true;
    elseif SOC(k) > SOChigh
        engOn = false;
    end                                          % otherwise keep previous state
    if engOn, Peng(k) = Pe_on; end
    Pgen = Peng(k)*eta_g;

    % Battery (Rint model)
    Pbat(k) = Pbus - Pgen;                       % + discharge, - charge
    Pbat(k) = max(Pbat(k), -15e3);               % battery charge power limit (W)
    Voc = interp1(socTab, vocTab, SOC(k), 'linear', 'extrap');
    Pbat(k) = min(Pbat(k), 0.99*Voc^2/(4*Rint)); % physical power limit
    Ibat(k) = (Voc - sqrt(Voc^2 - 4*Rint*Pbat(k)))/(2*Rint);
    if k < N, SOC(k+1) = SOC(k) - Ibat(k)*dt/Qbat; end

    % Fuel
    if engOn
        eta = interp1(loadFrac, eta_eng, Peng(k)/Pe_max, 'linear', 'extrap');
        mdot(k) = Peng(k)/(eta*LHV);
    end
end

% ---- Results ----
fuel_L  = sum(mdot)*dt/rho_fuel;
dist_km = trapz(t,v)/1000;

% SOC correction: charge-sustaining equivalent fuel
eta_chain = interp1(loadFrac, eta_eng, Pe_on/Pe_max)*eta_g*eta_inv;
dE = (SOC0 - SOC(end))*Qbat*mean(vocTab);        % J drawn from battery (+) or stored (-)
fuel_corr_L = fuel_L + dE/(eta_chain*LHV)/rho_fuel;

L100_raw  = fuel_L/dist_km*100;
L100_corr = fuel_corr_L/dist_km*100;
fprintf('Final SOC: %.3f (start %.2f)\n', SOC(end), SOC0);
fprintf('Fuel (raw): %.3f L -> %.2f L/100km\n', fuel_L, L100_raw);
fprintf('Fuel (SOC-corrected): %.3f L -> %.2f L/100km (%.1f km/L)\n', ...
    fuel_corr_L, L100_corr, dist_km/fuel_corr_L);
fprintf('ICE baseline: %.2f L/100km\n', ice.L100);
fprintf('Improvement vs ICE: %.1f %%\n', (1 - L100_corr/ice.L100)*100);

series.L100 = L100_corr; series.SOC = SOC; series.t = t;
save(fullfile('results','series_thermostat.mat'),'series');

figure
subplot(3,1,1); plot(t,SOC); yline(SOClow,'--'); yline(SOChigh,'--'); grid on
ylabel('SOC')
subplot(3,1,2); plot(t,Peng/1000); grid on; ylabel('Engine power (kW)')
subplot(3,1,3); plot(t,Pbat/1000); grid on; ylabel('Battery power (kW)')
xlabel('Time (s)')