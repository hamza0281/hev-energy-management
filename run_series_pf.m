% run_series_pf.m - series HEV, power-follower control
clear; clc; close all
addpath(genpath(pwd));
params; load('cycle.mat'); load('ice_baseline.mat');
tEnd = cycle(end,1);
out = sim('vehicle_base');
t = out.v.Time; v = out.v.Data; Pw = out.Pwheel.Data;
dt = mean(diff(t)); N = numel(t);

% ---- Parameters (same hardware as thermostat case) ----
eta_m = 0.92; eta_g = 0.92; eta_inv = 0.95;
Paux = 500; Pregen_max = 20e3; Pchg_max = 15e3;
Pe_max = 70e3; Pe_min = 5e3;             % engine runs only between these
LHV = 42.5e6; rho_fuel = 0.745;
loadFrac = [0.00 0.05 0.10 0.20 0.40 0.60 0.80 1.00];
eta_eng  = [0.05 0.14 0.20 0.27 0.33 0.34 0.33 0.31];
Qbat = 6.5*3600; Rint = 0.15;
socTab = [0 0.2 0.4 0.6 0.8 1];
vocTab = [170 185 195 202 210 220];
SOC0 = 0.60; SOClow = 0.50; SOChigh = 0.70;

% ---- Power-follower tuning ----
Pon     = 8e3;      % engine starts only if bus demand exceeds this (W)
Kshare  = 0.6;      % fraction of demand supplied by engine in normal range
Pcharge = 6e3;      % extra engine power added when SOC is low (W)

SOC = zeros(N,1); SOC(1) = SOC0;
Peng = zeros(N,1); Pbat = zeros(N,1); Ibat = zeros(N,1); mdot = zeros(N,1);
chgMode = false;
for k = 1:N
    if Pw(k) >= 0, Pmot = Pw(k)/eta_m;
    else,          Pmot = max(Pw(k)*eta_m, -Pregen_max); end
    Pbus = Pmot + Paux;
    if SOC(k) < SOClow
        chgMode = true;
    elseif SOC(k) > SOClow + 0.05      % exit charge mode only 5% above the entry point
        chgMode = false;
    end

    % ---- Power-follower rules ----
    if Pw(k) < 0 || SOC(k) > SOChigh
        Pe_cmd = 0;                                   % braking or battery full: engine OFF
    elseif chgMode
        Pe_cmd = max(Pbus,0)/eta_g + Pcharge;         % low SOC: demand + charging
    elseif Pbus > Pon
        Pe_cmd = Kshare*Pbus/eta_g;                   % normal: engine shares demand
    else
        Pe_cmd = 0;                                   % low demand: electric only
    end
    if Pe_cmd < Pe_min, Pe_cmd = 0; end               % avoid inefficient low-load running
    Peng(k) = min(Pe_cmd, Pe_max);
    Pgen = Peng(k)*eta_g;

    Pb = Pbus - Pgen;
    Pb = max(Pb, -Pchg_max);                          % battery charge limit
    Voc = interp1(socTab, vocTab, SOC(k), 'linear', 'extrap');
    Pb = min(Pb, 0.99*Voc^2/(4*Rint));
    Pbat(k) = Pb;
    Ibat(k) = (Voc - sqrt(Voc^2 - 4*Rint*Pb))/(2*Rint);
    if k < N, SOC(k+1) = SOC(k) - Ibat(k)*dt/Qbat; end

    if Peng(k) > 0
        eta = interp1(loadFrac, eta_eng, Peng(k)/Pe_max, 'linear', 'extrap');
        mdot(k) = Peng(k)/(eta*LHV);
    end
end

% ---- Results with SOC correction ----
fuel_L  = sum(mdot)*dt/rho_fuel;
dist_km = trapz(t,v)/1000;
Pavg = mean(Peng(Peng>0));
eta_chain = interp1(loadFrac, eta_eng, Pavg/Pe_max)*eta_g*eta_inv;
dE = (SOC0 - SOC(end))*Qbat*mean(vocTab);
fuel_corr_L = fuel_L + dE/(eta_chain*LHV)/rho_fuel;
L100 = fuel_corr_L/dist_km*100;

load(fullfile('results','series_thermostat.mat'));
fprintf('Final SOC: %.3f\n', SOC(end));
fprintf('Power-follower: %.2f L/100km (SOC-corrected)\n', L100);
fprintf('Thermostat:     %.2f L/100km\n', series.L100);
fprintf('ICE baseline:   %.2f L/100km\n', ice.L100);
fprintf('PF improvement vs ICE: %.1f %%\n', (1-L100/ice.L100)*100);

pf.L100 = L100; pf.SOC = SOC; pf.t = t;
save(fullfile('results','series_powerfollower.mat'),'pf');

figure
subplot(3,1,1); plot(t,SOC); yline(SOClow,'--'); yline(SOChigh,'--'); grid on; ylabel('SOC')
subplot(3,1,2); plot(t,Peng/1000); grid on; ylabel('Engine power (kW)')
subplot(3,1,3); plot(t,Pbat/1000); grid on; ylabel('Battery power (kW)'); xlabel('Time (s)')