% select parameters
prefix = 'Km';

% table data
Table = readtable([prefix, '.txt']);
Table.Properties.VariableNames = ["time", "target-position", "position-demand", "actual-position", "velocity", "Km"];

% plot
plot(Table.("time"), Table.("target-position"));
hold on;
plot(Table.("time"), Table.("position-demand"));
hold on;
plot(Table.("time"), Table.("actual-position"));
hold on;
plot(Table.("time"), Table.("Km"));
legend('target-position', 'position-deman', "actual-position", "Km");


function x = deg2mm_f(x)
%
% EccA-EccB angular [deg] to linear [mm] motion map for front axis.
%
    c5 = 0.0000000000703;
    c4 = 0.0000000031969;
    c3 = -0.000004402643;
    c2 = -0.0001583845823;
    c1 = 0.083336057797;
    c0 = 0.0046001004738;
    x = c5*x^5 + c4*x^4 + c3*x^3 + c2*x^2 + c1*x + c0;
end

function x = deg2mm_r(x)
%
% EccA-EccB angular [deg] to linear [mm] motion map for rear axis.
%
    c5 = 0.0000000000624;
    c4 = -0.000000000045;
    c3 = -0.0000037025199;
    c2 = -0.0001050951019;
    c1 = 0.0716813513211;
    c0 = 0.0041785649983;
    x = c5*x.^5 + c4*x.^4 + c3*x.^3 + c2*x.^2 + c1*x + c0;
end
