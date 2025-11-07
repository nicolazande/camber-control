%% CONFIG
xlsx_file = 'data/20251107_992GT3_Toe_New_PID_Tests.xlsx';
sheet = 3;

%% READ & AUTO-MAP COLUMNS
T = readtable(xlsx_file, 'Sheet', sheet);
T.Properties.VariableNames
time = T.("Var1") / 1000;
setpoint = T.("Var4");
feedback = T.("Var9");
current = T.("Var19");

%% PLOT POSITION
figure; hold on; grid on;
plot(time, setpoint, 'LineWidth', 1.5);
plot(time, feedback, 'LineWidth', 1.5);
legend('Reference','Feedback');
title("Position Tracking (deg)");
xlabel("Time [s]");
ylabel("Angle [deg]");

%% PLOT CURRENT IF PRESENT
if ~isempty(current)
    figure; plot(time, current, 'LineWidth', 1.3); grid on;
    title("Motor Current"); xlabel("Time [s]");
    ylabel("Current [A]");
end

