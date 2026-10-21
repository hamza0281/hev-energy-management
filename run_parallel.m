% run_parallel.m - parallel HEV, rule-based mode control (backward-facing)
clear; clc; close all
addpath(genpath(pwd));
params; load('cycle.mat'); load('ice_baseline.mat');
tEnd = cycle(end,1);
out = sim('vehicle_base');
t = out.v.Time; v = out.v.Data; Pw = out.Pwheel.Data;
dt = mean(diff(t)); N = numel(t);

% ---- Parameters ----
eta_drv = 0.90; eta_m = 0.92; eta_inv = 0.95;
Paux = 500; Pregen_max = 20e3; Pchg_max = 15e3;
Pe_max = 70e3;
LHV = 42.5e6; rho_fuel = 0.745;
loadFrac = [0.00 0.05 0.10 0.20 0.40 0.60 0.80 1.00];
eta_eng  = [0.05 0.14 0.20 0.27 0.33 0.34 0.33 0.31];
Qbat = 6.5*3600; Rint = 0.15;
socTab = [0 0.2 0.4 0.6 0.8 1];
vocTab = [170 185 195 202 210 220];
SOC0 = 0.60; SOClow = 0.50; SOChigh = 0.70;

% ---- Rule thresholds ----
Pev     = 8e3;     % below this shaft power, drive electric only (W)
Passist = 30e3;    % above this, engine capped here and motor assists (W)
Pchg    = 8e3;     % extra engine power in charge mode (W)

SOC = zeros(N,1); SOC(1) = SOC0;
Pe = zeros(N,1); Pbat = zeros(N,1); mdot = zeros(N,1); mode = zeros(N,1);
chgMode = false;
for k = 1:N
    % hysteresis for charge mode
    if SOC(k) < SOClow, chgMode = true;
    elseif SOC(k) > SOClow + 0.05, chgMode = false; end

    if Pw(k) < 0
        % ---- Regenerative braking: engine off, friction brakes take the rest ----
        mode(k) = 5;  Pe(k) = 0;
        Pelec = max(Pw(k)*eta_drv*eta_m, -Pregen_max);
    else
        Pshaft = Pw(k)/eta_drv;                  % shaft power needed at the wheels
        if chgMode
            mode(k) = 4;                         % charge while driving
            Pe(k) = min(Pshaft + Pchg, Pe_max);
            Pelec = (Pshaft - Pe(k))*eta_m;      % motor acts as generator
        elseif Pshaft < Pev && SOC(k) > SOClow
            mode(k) = 1;                         % EV only
            Pe(k) = 0;
            Pelec = Pshaft/eta_m;
        elseif Pshaft > Passist && SOC(k) > SOClow
            mode(k) = 3;                         % hybrid assist
            Pe(k) = Passist;
            Pelec = (Pshaft - Pe(k))/eta_m;
        else
            mode(k) = 2;                         % engine only
            Pe(k) = min(Pshaft, Pe_max);
            Pelec = 0;
        end
    end

    % ---- Battery (Rint) ----
    Pb = Pelec + Paux;
    Pb = max(Pb, -Pchg_max);
    Voc = interp1(socTab, vocTab, SOC(k), 'linear', 'extrap');
    Pb = min(Pb, 0.99*Voc^2/(4*Rint));
    Pbat(k) = Pb;
    I = (Voc - sqrt(Voc^2 - 4*Rint*Pb))/(2*Rint);
    if k < N, SOC(k+1) = SOC(k) - I*dt/Qbat; end

    % ---- Fuel ----
    if Pe(k) > 0
        eta = interp1(loadFrac, eta_eng, Pe(k)/Pe_max, 'linear', 'extrap');
        mdot(k) = Pe(k)/(eta*LHV);
    end
end

% ---- Results with SOC correction ----
fuel_L  = sum(mdot)*dt/rho_fuel;
dist_km = trapz(t,v)/1000;
Pavg = mean(Pe(Pe>0));
eta_chain = interp1(loadFrac, eta_eng, Pavg/Pe_max)*eta_m*eta_inv;
dE = (SOC0 - SOC(end))*Qbat*mean(vocTab);
fuel_corr_L = fuel_L + dE/(eta_chain*LHV)/rho_fuel;
L100 = fuel_corr_L/dist_km*100;

load(fullfile('results','series_thermostat.mat'));
load(fullfile('results','series_powerfollower.mat'));
fprintf('Final SOC: %.3f\n', SOC(end));
fprintf('Parallel HEV:         %.2f L/100km (SOC-corrected)\n', L100);
fprintf('Series thermostat:    %.2f L/100km\n', series.L100);
fprintf('Series power-follower:%.2f L/100km\n', pf.L100);
fprintf('ICE baseline:         %.2f L/100km\n', ice.L100);
fprintf('Parallel improvement vs ICE: %.1f %%\n', (1-L100/ice.L100)*100);
names = {'EV','Engine only','Assist','Charge','Regen'};
for m = 1:5
    fprintf('  %-12s %.1f %% of time\n', names{m}, 100*mean(mode==m));
end

par.L100 = L100; par.SOC = SOC; par.t = t;
save(fullfile('results','parallel.mat'),'par');

figure
subplot(4,1,1); plot(t,SOC); yline(SOClow,'--'); yline(SOChigh,'--'); grid on; ylabel('SOC')
subplot(4,1,2); plot(t,Pe/1000); grid on; ylabel('Engine (kW)')
subplot(4,1,3); plot(t,Pbat/1000); grid on; ylabel('Battery (kW)')
subplot(4,1,4); stairs(t,mode); grid on; ylabel('Mode'); xlabel('Time (s)')
yticks(1:5); yticklabels(names)