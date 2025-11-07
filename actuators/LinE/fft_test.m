%% === CONFIG ===
xlsx_file = 'data/20251106_992GT3_Toe_New_PID_Tests.xlsx';
sheet = 2;

%% === READ & AUTO-MAP COLUMNS ===
T = readtable(xlsx_file, 'Sheet', sheet);

time     = T.("Var1") / 1000;   % [s]
setpoint = T.("Var4");          % position target
feedback = T.("Var9");          % position actual
current  = T.("Var19");         % current command or measured

%% === SAMPLING ===
dt = diff(time);
dt = dt(~isnan(dt) & isfinite(dt));
Fs = 1 / mean(dt);              % sampling freq [Hz]
fprintf('Sampling freq: %.1f Hz\n', Fs);

%% === RAW FFT ON FEEDBACK (overview of whole run) ===
fb = fillmissing(feedback,'linear');
L  = length(fb);
Y  = fft(fb - mean(fb));
P2 = abs(Y/L);
P1 = P2(1:L/2+1);
P1(2:end-1) = 2*P1(2:end-1);
f  = Fs*(0:(L/2))/L;

figure('Name','FFT - Position Actual');
plot(f, P1, 'LineWidth', 1.3); grid on;
xlim([0 min(Fs/2,200)]);
title('FFT globale di Position Actual');
xlabel('Frequenza [Hz]'); ylabel('|Ampiezza|');
set(gca,'YScale','log');

%% === PLATEAU ANALYSIS (ripple/noise) ===
% 1) Find flat setpoint
dsp  = [0; diff(setpoint)];
flat = abs(dsp) < 1e-6;
runs = []; st = NaN;
for i = 1:numel(flat)
    if flat(i) && isnan(st), st = i; end
    if (~flat(i) || i==numel(flat)) && ~isnan(st)
        runs(end+1,:) = [st, i]; 
        st = NaN;
    end
end
if isempty(runs)
    idx = round(numel(time)/3) : round(2*numel(time)/3);
else
    [~,k] = max(runs(:,2)-runs(:,1));
    s = runs(k,1); e = runs(k,2);
    guard = round(0.05*Fs); % remove 50ms from border
    s = s+guard; e = e-guard;
    s = max(s,1); e = min(e,numel(time));
    idx = s:e;
end

% 2) Error in plateau
e = fillmissing(setpoint(idx) - feedback(idx),'linear');
e = detrend(e);
t_plateau = time(idx)-time(idx(1));

% 3) Welch PSD
nperseg = min(length(e), round(0.6*Fs));
[pxx, fwel] = pwelch(e, hann(nperseg), floor(nperseg/2), [], Fs, 'onesided');

% 4) Peak and band
mask_dither = fwel >= 5 & fwel <= min(200, Fs/2);
[~,i_pk] = max(pxx(mask_dither));
f_pk = fwel(mask_dither); f_pk = f_pk(i_pk);

% Energy per band
band_low = fwel>=0.1 & fwel<5;       % slow (control)
band_mid = fwel>=5   & fwel<30;      % closed loop
band_high= fwel>=30  & fwel<200;     % noise or quantization
E_low  = trapz(fwel(band_low),  pxx(band_low));
E_mid  = trapz(fwel(band_mid),  pxx(band_mid));
E_high = trapz(fwel(band_high), pxx(band_high));

fprintf('--- Plateau analysis ---\n');
fprintf('Peak freq (5–200Hz): %.2f Hz\n', f_pk);
fprintf('Energy ratio  Low[0–5Hz]=%.2g | Mid[5–30Hz]=%.2g | High[30–200Hz]=%.2g\n',...
        E_low, E_mid, E_high);

%% === PLOT ===
figure('Name','Error in plateau and PSD');
subplot(2,1,1);
plot(t_plateau, e, 'LineWidth', 1); grid on;
xlabel('Time [s]'); ylabel('Error [unit]');
title('Error in plateau (detrend)');

subplot(2,1,2);
semilogy(fwel, pxx, 'LineWidth', 1.2); grid on; hold on;
plot([f_pk f_pk], ylim, '--r', 'DisplayName', sprintf('Peak %.1fHz', f_pk));
xlim([0 200]);
xlabel('Frequency [Hz]'); ylabel('PSD');
title('PSD (Welch) error');
legend show;
