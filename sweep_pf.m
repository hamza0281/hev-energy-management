% sweep_pf.m - sensitivity study of the power-follower parameters
clear; clc; close all
addpath(genpath(pwd));
params; load('cycle.mat'); load('ice_baseline.mat');
tEnd = cycle(end,1);
out = sim('vehicle_base');
t = out.v.Time; v = out.v.Data; Pw = out.Pwheel.Data;

Ksh_list = [0.6 0.8 1.0];
Pon_list = [8e3 12e3 16e3 20e3];
L100 = zeros(numel(Ksh_list), numel(Pon_list));
SOCend = L100;
for i = 1:numel(Ksh_list)
    for j = 1:numel(Pon_list)
        [L100(i,j), SOCend(i,j)] = pf_model(t, v, Pw, Ksh_list(i), Pon_list(j));
    end
end

fprintf('\nL/100km (rows: Kshare, columns: Pon in kW)\n');
fprintf('Kshare \\ Pon  %6.0f %6.0f %6.0f\n', Pon_list/1e3);
for i = 1:numel(Ksh_list)
    fprintf('   %.1f        %6.2f %6.2f %6.2f\n', Ksh_list(i), L100(i,:));
end
fprintf('\nFinal SOC\n');
for i = 1:numel(Ksh_list)
    fprintf('   %.1f        %6.3f %6.3f %6.3f\n', Ksh_list(i), SOCend(i,:));
end

T = table(repelem(Ksh_list',numel(Pon_list)), repmat(Pon_list'/1e3,numel(Ksh_list),1), ...
    L100(:), SOCend(:), 'VariableNames', {'Kshare','Pon_kW','L_per_100km','SOC_end'});
writetable(T, fullfile('results','sensitivity.csv'));

figure
heatmap(string(Pon_list/1e3), string(Ksh_list), L100);
xlabel('Pon (kW)'); ylabel('Kshare'); title('Power-follower fuel use (L/100 km)')
saveas(gcf, fullfile('results','sensitivity.png'));

% ------------------------------------------------------------------
function [L100, SOCfinal] = pf_model(t, v, Pw, Kshare, Pon)
N = numel(t); dt = mean(diff(t));
eta_m = 0.92; eta_g = 0.92; eta_inv = 0.95;
Paux = 500; Pregen_max = 20e3; Pchg_max = 15e3;
Pe_max = 70e3; Pe_min = 5e3;
LHV = 42.5e6; rho_fuel = 0.745;
loadFrac = [0.00 0.05 0.10 0.20 0.40 0.60 0.80 1.00];
eta_eng  = [0.05 0.14 0.20 0.27 0.33 0.34 0.33 0.31];
Qbat = 6.5*3600; Rint = 0.15;
socTab = [0 0.2 0.4 0.6 0.8 1];
vocTab = [170 185 195 202 210 220];
SOC0 = 0.60; SOClow = 0.50; SOChigh = 0.70; Pcharge = 6e3;

SOC = zeros(N,1); SOC(1) = SOC0;
Peng = zeros(N,1); mdot = zeros(N,1); chgMode = false;
for k = 1:N
    if Pw(k) >= 0, Pmot = Pw(k)/eta_m;
    else,          Pmot = max(Pw(k)*eta_m, -Pregen_max); end
    Pbus = Pmot + Paux;
    if SOC(k) < SOClow, chgMode = true;
    elseif SOC(k) > SOClow + 0.05, chgMode = false; end

    if Pw(k) < 0 || SOC(k) > SOChigh,  Pe_cmd = 0;
    elseif chgMode,                    Pe_cmd = max(Pbus,0)/eta_g + Pcharge;
    elseif Pbus > Pon,                 Pe_cmd = Kshare*Pbus/eta_g;
    else,                              Pe_cmd = 0; end
    if Pe_cmd < Pe_min, Pe_cmd = 0; end
    Peng(k) = min(Pe_cmd, Pe_max);

    Pb = Pbus - Peng(k)*eta_g;
    Pb = max(Pb, -Pchg_max);
    Voc = interp1(socTab, vocTab, SOC(k), 'linear', 'extrap');
    Pb = min(Pb, 0.99*Voc^2/(4*Rint));
    I = (Voc - sqrt(Voc^2 - 4*Rint*Pb))/(2*Rint);
    if k < N, SOC(k+1) = SOC(k) - I*dt/Qbat; end

    if Peng(k) > 0
        eta = interp1(loadFrac, eta_eng, Peng(k)/Pe_max, 'linear', 'extrap');
        mdot(k) = Peng(k)/(eta*LHV);
    end
end
fuel_L = sum(mdot)*dt/rho_fuel;
dist_km = trapz(t,v)/1000;
if any(Peng>0), Pavg = mean(Peng(Peng>0)); else, Pavg = 20e3; end
eta_chain = interp1(loadFrac, eta_eng, Pavg/Pe_max)*eta_g*eta_inv;
dE = (SOC0 - SOC(end))*Qbat*mean(vocTab);
L100 = (fuel_L + dE/(eta_chain*LHV)/rho_fuel)/dist_km*100;
SOCfinal = SOC(end);
end