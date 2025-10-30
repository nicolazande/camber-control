%% === CONFIG ===
xlsx_file = 'data/20251024_992GT3_Camber_PID_Tests.xlsx';
sheet = 4;

%% === READ & AUTO-MAP COLUMNS ===
T = readtable(xlsx_file, 'Sheet', sheet);

time = T.("Time_ms_") / 1000;
setpoint = T.("PosTar_deg_");
feedback = T.("PosAct_deg_");
current = T.("Current_A_");

%% === PLOT POSITION ===
figure; hold on; grid on;
plot(time, setpoint, 'LineWidth', 1.5);
plot(time, feedback, 'LineWidth', 1.5);
legend('Reference','Feedback');
title("Position Tracking (deg)");
xlabel("Time [s]");
ylabel("Angle [deg]");

%% === PLOT CURRENT IF PRESENT ===
if ~isempty(current)
    figure; plot(time, current, 'LineWidth', 1.3); grid on;
    title("Motor Current"); xlabel("Time [s]");
    ylabel("Current [A]");
end