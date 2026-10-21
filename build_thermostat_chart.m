function build_thermostat_chart()
mdl = 'thermostat_sf';
if bdIsLoaded(mdl), close_system(mdl,0); end
new_system(mdl); open_system(mdl);

% Thresholds live in the base workspace (also keep them in params.m)
assignin('base','SOClow',0.45);
assignin('base','SOChigh',0.70);

% ---- Test input: SOC sweeping 0.40 -> 0.75 -> 0.40 every 200 s ----
add_block('simulink/Sources/Sine Wave',[mdl '/SOC_test'], ...
    'Amplitude','0.175','Bias','0.575','Frequency','2*pi/200', ...
    'Position',[30 60 80 90]);

% ---- Stateflow chart ----
add_block('sflib/Chart',[mdl '/Thermostat'],'Position',[170 40 300 110]);
rt = sfroot;
ch = rt.find('-isa','Stateflow.Chart','Path',[mdl '/Thermostat']);
ch.ChartUpdate = 'DISCRETE';
ch.SampleTime  = '0.1';

d = Stateflow.Data(ch); d.Name = 'SOC';     d.Scope = 'Input';
d = Stateflow.Data(ch); d.Name = 'engOn';   d.Scope = 'Output';
d = Stateflow.Data(ch); d.Name = 'SOClow';  d.Scope = 'Parameter';
d = Stateflow.Data(ch); d.Name = 'SOChigh'; d.Scope = 'Parameter';

sOff = Stateflow.State(ch);
sOff.Position    = [40 100 140 60];
sOff.LabelString = sprintf('Engine_OFF\nentry: engOn = 0;');

sOn = Stateflow.State(ch);
sOn.Position     = [300 100 140 60];
sOn.LabelString  = sprintf('Engine_ON\nentry: engOn = 1;');

t0 = Stateflow.Transition(ch);                 % default transition
t0.Destination = sOff; t0.DestinationOClock = 0;
t0.SourceEndPoint = t0.DestinationEndPoint - [0 30];

t1 = Stateflow.Transition(ch);
t1.Source = sOff; t1.Destination = sOn;
t1.LabelString = '[SOC < SOClow]';

t2 = Stateflow.Transition(ch);
t2.Source = sOn; t2.Destination = sOff;
t2.LabelString = '[SOC > SOChigh]';

% ---- Scope and wiring ----
add_block('simulink/Sinks/Scope',[mdl '/Scope'], ...
    'NumInputPorts','2','Position',[380 50 420 110]);
add_line(mdl,'SOC_test/1','Thermostat/1','autorouting','on');
add_line(mdl,'SOC_test/1','Scope/1','autorouting','on');
add_line(mdl,'Thermostat/1','Scope/2','autorouting','on');

set_param(mdl,'SolverType','Fixed-step','Solver','ode3', ...
    'FixedStep','0.1','StopTime','400');
save_system(mdl, fullfile('models',mdl));
disp('Model built: press Run in Simulink, then open the Scope.');
end