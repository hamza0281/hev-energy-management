% run_all.m - regenerates every result and figure from scratch
addpath(genpath(pwd));
import_cycle;                                           % UDDS -> data/cycle.mat
run_baseline;   saveas(gcf,'results/baseline_tracking.png');
run_ice;        saveas(gcf,'results/ice_power.png');
run_series;     saveas(gcf,'results/series_thermostat.png');
run_series_sf;  saveas(gcf,'results/series_stateflow.png');
run_series_pf;  saveas(gcf,'results/series_powerfollower.png');
run_parallel;   saveas(gcf,'results/parallel_modes.png');
run_compare;                                            % saves comparison.png and .csv
sweep_pf;                                               % saves sensitivity.png and .csv
disp('All results regenerated in /results');