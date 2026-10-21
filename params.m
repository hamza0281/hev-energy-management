% params.m - all vehicle parameters live here
veh_m   = 1400;    % vehicle mass, kg
veh_Cd  = 0.30;    % drag coefficient
veh_A   = 2.2;     % frontal area, m^2
veh_Crr = 0.010;   % rolling resistance coefficient
veh_rw  = 0.30;    % wheel radius, m
rho     = 1.2;     % air density, kg/m^3
g       = 9.81;    % gravity, m/s^2

% Driver PI controller
Kp   = 3000;       % N per (m/s) of speed error
Ki   = 300;
Fmax = 4200;       % max tractive/brake force, N