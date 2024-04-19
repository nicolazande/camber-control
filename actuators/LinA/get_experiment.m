% select parameters
prefix = '';
load = 'pp';
source_type = 'sine-sweep';
amplitude = '10mm';
frequency = '0.5Hz';

% table data
%Table = readtable(['data/', source_type, '-', amplitude, '-', frequency, '.txt']);
%Table.Properties.VariableNames = ["time", "target-position", "position-demand", "actual-position", "actual-speed", "current", "torque"];
Table = readtable(['data/', load, '-', source_type, '-', amplitude, '-', frequency, '.txt']);
Table.Properties.VariableNames = ["time", "target-position", "position-demand", "actual-position", "actual-speed", "current"];

% Create plots
setpoint = 'target-position';
feedback = 'position-demand';
plot(Table.("time"), Table.(setpoint));
hold on;
plot(Table.("time"), Table.(feedback));
title([source_type, ' - ', amplitude, ' - ', frequency]);
legend('reference', 'feedback');

% iddata for as simulation input
idddata_input = iddata(0.001*Table.("target-position"), [], 0.01);