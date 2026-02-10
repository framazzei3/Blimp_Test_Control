%% ============================
%        INIT
% ============================
clc;
close all;
clear;

addpath Data\

%% ============================
%        LOAD DATA
% ============================
csv_file = "20260205_163719.csv";
T = readtable(csv_file);

% Filter only ALTITUDE HOLD mode
T_alt = T(strcmp(T.mode, "ALTITUDE_HOLD"), :);
T_alt = T_alt(3000:end,:);

%% ============================
%        TIME VECTOR
% ============================
% Relative time in seconds
t = (T_alt.millis - T_alt.millis(1)) / 1000; % [s]
dt = mean(diff(t)); % average time step
fs = 1/dt; % sampling frequency

fprintf('===== ACQUISITION PARAMETERS =====\n');
fprintf('Sampling frequency: %.2f Hz\n', fs);
fprintf('Sampling period: %.4f s\n', dt);
fprintf('Total duration: %.2f s\n', t(end));
fprintf('Number of samples: %d\n', length(t));

%% ============================
%   VELOCITY COMPUTATION (DERIVATIVE)
% ============================
z = T_alt.z_meas; % height measurement [m]

% Numerical derivative (central difference)
v_raw = gradient(z, t); % [m/s]

% Show raw signal
figure('Name', 'Raw signal');
subplot(2,1,1)
plot(t, z, 'LineWidth', 1.5)
xlabel('Time (s)')
ylabel('Height z (m)')
title('Height measurement')
grid on

subplot(2,1,2)
plot(t, v_raw, 'Color', [0.7 0.7 0.7], 'LineWidth', 1)
xlabel('Time (s)')
ylabel('Velocity v_{raw} (m/s)')
title('Vertical velocity (raw derivative)')
grid on

%% ============================
%   FREQUENCY ANALYSIS
% ============================
% FFT of velocity signal
N = length(v_raw);
Y = fft(v_raw);
P2 = abs(Y/N);
P1 = P2(1:N/2+1);
P1(2:end-1) = 2*P1(2:end-1);
f = fs*(0:(N/2))/N;

% PSD (Power Spectral Density)
[pxx, f_psd] = pwelch(v_raw, [], [], [], fs);

% Show spectrum
figure('Name', 'Frequency analysis');
subplot(2,1,1)
plot(f, P1, 'LineWidth', 1.5)
xlabel('Frequency (Hz)')
ylabel('|Amplitude|')
title('Amplitude spectrum')
grid on
xlim([0 min(5, fs/2)]) % limit to 5 Hz or Nyquist

subplot(2,1,2)
plot(f_psd, 10*log10(pxx), 'LineWidth', 1.5)
xlabel('Frequency (Hz)')
ylabel('PSD (dB/Hz)')
title('Power spectral density')
grid on
xlim([0 min(5, fs/2)])

%% ============================
%   CUTOFF FREQUENCY ESTIMATION
% ============================
% Cumulative power
potenza_cum = cumsum(pxx) / sum(pxx);

% Frequency containing 95% of energy
threshold = 0.95;
idx_95 = find(potenza_cum >= threshold, 1, 'first');
fc_95 = f_psd(idx_95);

% Frequency containing 99% of energy
threshold = 0.99;
idx_99 = find(potenza_cum >= threshold, 1, 'first');
fc_99 = f_psd(idx_99);

fprintf('\n===== ESTIMATED CUTOFF FREQUENCY =====\n');
fprintf('fc (95%% energy): %.2f Hz\n', fc_95);
fprintf('fc (99%% energy): %.2f Hz\n', fc_99);

% Choose cutoff frequency (use 95% or adjust manually)
fc = 0.4; % <-- MANUALLY ADJUST HERE (try 0.3, 0.5, 1.0, 1.5 Hz)
fprintf('\n--> MANUAL cutoff frequency: %.2f Hz\n', fc);

%% ============================
%   ALPHA COEFFICIENT COMPUTATION
% ============================
% For Arduino (using the same dt as the data)
dt_arduino = dt;

alpha_v = exp(-2 * pi * fc * dt_arduino);

fprintf('\n===== FILTER PARAMETERS FOR ARDUINO =====\n');
fprintf('Alpha to use: %.6f\n', alpha_v);
fprintf('(for dt = %.4f s, fc = %.2f Hz)\n', dt_arduino, fc);

% Check effective cutoff frequency
fc_effective = -log(alpha_v) / (2 * pi * dt_arduino);
fprintf('Effective cutoff frequency: %.2f Hz\n', fc_effective);

%% ============================
%   LOW-PASS FILTER DESIGN
% ============================
% Butterworth filter
order = 2;
fn = fc / (fs/2); % normalized frequency
[b, a] = butter(order, fn, 'low');

% Show frequency response
figure('Name', 'Filter frequency response');
freqz(b, a, 1024, fs)
sgtitle(sprintf('Butterworth filter order %d, fc = %.2f Hz', order, fc))

%% ============================
%   APPLYING FILTERS
% ============================
% Butterworth filter (zero-phase)
v_filt_butter = filtfilt(b, a, v_raw);

% Exponential filter (Arduino-like)
v_filt_exp = zeros(size(v_raw));
for k = 2:length(v_raw)
    v_filt_exp(k) = alpha_v * v_filt_exp(k-1) + (1 - alpha_v) * v_raw(k);
end

%% ============================
%   FILTER COMPARISON
% ============================
figure('Name', 'Filter comparison');
subplot(3,1,1)
plot(t, z, 'LineWidth', 1.5)
xlabel('Time (s)')
ylabel('Height (m)')
title('Original signal: height z')
grid on

subplot(3,1,2)
plot(t, v_raw, 'LineWidth', 1.5, 'DisplayName', 'v_{raw}')
hold on
plot(t, v_filt_butter, 'LineWidth', 1.5, 'DisplayName', 'Butterworth')
plot(t, v_filt_exp', 'LineWidth', 1.5, 'DisplayName', 'Exponential (Arduino)')
plot(t, T_alt.v_hat, 'g:', 'LineWidth', 1.5, 'DisplayName', 'v_{hat} Arduino (Moving Average)')
xlabel('Time (s)')
ylabel('Velocity (m/s)')
title('Filtered vertical velocity')
legend('Location', 'best')
grid on

subplot(3,1,3)
noise = v_raw - v_filt_exp;
plot(t, noise)
xlabel('Time (s)')
ylabel('Removed noise (m/s)')
title('Noise component (raw - filtered)')
grid on

%% ============================
%   SNR
% ============================
snr_raw = snr(v_raw);
snr_butter = snr(v_filt_butter);
snr_exp = snr(v_filt_exp);

fprintf('\n===== SIGNAL-TO-NOISE RATIO =====\n');
fprintf('SNR raw:          %.2f dB\n', snr_raw);
fprintf('SNR Butterworth:  %.2f dB\n', snr_butter);
fprintf('SNR Exponential:  %.2f dB\n', snr_exp);

%% ============================
%   ARDUINO CODE
% ============================
fprintf('\n===== ARDUINO CODE =====\n');
fprintf('const float ALPHA_V = %.6ff;\n\n', alpha_v);
fprintf('// Inside loop:\n');
fprintf('float v_raw = (z_meas - z_prev) / dt;\n');
fprintf('z_prev = z_meas;\n');
fprintf('v_hat = ALPHA_V * v_hat + (1.0f - ALPHA_V) * v_raw;\n');
