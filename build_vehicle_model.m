function build_vehicle_model()
mdl = 'vehicle_base';
if bdIsLoaded(mdl), close_system(mdl,0); end
new_system(mdl); open_system(mdl);

% ---- Blocks ----
add_block('simulink/Sources/From Workspace',[mdl '/Cycle'], ...
    'VariableName','cycle','Position',[30 100 110 130]);
add_block('simulink/Math Operations/Sum',[mdl '/SpeedErr'], ...
    'Inputs','+-','Position',[150 100 180 130]);
add_block('simulink/Math Operations/Gain',[mdl '/Kp'], ...
    'Gain','Kp','Position',[230 60 280 90]);
add_block('simulink/Math Operations/Gain',[mdl '/Ki'], ...
    'Gain','Ki','Position',[230 140 280 170]);
add_block('simulink/Continuous/Integrator',[mdl '/IntErr'], ...
    'LimitOutput','on','UpperSaturationLimit','Fmax', ...
    'LowerSaturationLimit','-Fmax','Position',[310 140 350 170]);
add_block('simulink/Math Operations/Sum',[mdl '/PI_Sum'], ...
    'Inputs','++','Position',[390 90 420 120]);
add_block('simulink/Discontinuities/Saturation',[mdl '/ForceLimit'], ...
    'UpperLimit','Fmax','LowerLimit','-Fmax','Position',[450 90 490 120]);

add_block('simulink/User-Defined Functions/Fcn',[mdl '/Faero'], ...
    'Expr','0.5*rho*veh_Cd*veh_A*u*abs(u)','Position',[600 220 700 250]);
add_block('simulink/User-Defined Functions/Fcn',[mdl '/Froll'], ...
    'Expr','veh_m*g*veh_Crr*tanh(u/0.1)','Position',[600 280 700 310]);
add_block('simulink/Math Operations/Sum',[mdl '/ForceBalance'], ...
    'Inputs','+--','Position',[560 100 590 140]);
add_block('simulink/Math Operations/Gain',[mdl '/InvMass'], ...
    'Gain','1/veh_m','Position',[630 105 680 135]);
add_block('simulink/Continuous/Integrator',[mdl '/Velocity'], ...
    'Position',[710 105 750 135]);
add_block('simulink/Math Operations/Product',[mdl '/WheelPower'], ...
    'Position',[560 380 590 420]);

% ---- Logging blocks ----
logs = {'ref','v','Ftrac','Pwheel'};
for i = 1:numel(logs)
    add_block('simulink/Sinks/To Workspace',[mdl '/log_' logs{i}], ...
        'VariableName',logs{i},'SaveFormat','Timeseries', ...
        'Position',[820 40+60*i 880 70+60*i]);
end

% ---- Wires ----
add_line(mdl,'Cycle/1','SpeedErr/1','autorouting','on');
add_line(mdl,'Velocity/1','SpeedErr/2','autorouting','on');
add_line(mdl,'SpeedErr/1','Kp/1','autorouting','on');
add_line(mdl,'SpeedErr/1','Ki/1','autorouting','on');
add_line(mdl,'Ki/1','IntErr/1','autorouting','on');
add_line(mdl,'Kp/1','PI_Sum/1','autorouting','on');
add_line(mdl,'IntErr/1','PI_Sum/2','autorouting','on');
add_line(mdl,'PI_Sum/1','ForceLimit/1','autorouting','on');
add_line(mdl,'ForceLimit/1','ForceBalance/1','autorouting','on');
add_line(mdl,'Velocity/1','Faero/1','autorouting','on');
add_line(mdl,'Velocity/1','Froll/1','autorouting','on');
add_line(mdl,'Faero/1','ForceBalance/2','autorouting','on');
add_line(mdl,'Froll/1','ForceBalance/3','autorouting','on');
add_line(mdl,'ForceBalance/1','InvMass/1','autorouting','on');
add_line(mdl,'InvMass/1','Velocity/1','autorouting','on');
add_line(mdl,'ForceLimit/1','WheelPower/1','autorouting','on');
add_line(mdl,'Velocity/1','WheelPower/2','autorouting','on');
% logging
add_line(mdl,'Cycle/1','log_ref/1','autorouting','on');
add_line(mdl,'Velocity/1','log_v/1','autorouting','on');
add_line(mdl,'ForceLimit/1','log_Ftrac/1','autorouting','on');
add_line(mdl,'WheelPower/1','log_Pwheel/1','autorouting','on');

% ---- Solver settings ----
set_param(mdl,'SolverType','Fixed-step','Solver','ode3', ...
    'FixedStep','0.1','StopTime','tEnd');

save_system(mdl, fullfile('models',mdl));
disp('Model built and saved in /models');
end