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
% Legge il CSV
% --- Nome file ---
csv_file = "20260209_163822.csv"; % log only data from 09 Feb 2026
T = readtable(csv_file);

% Filtra solo ALTITUDE HOLD
T_alt = T(strcmp(T.mode, "ALTITUDE_HOLD"), :);

%% ============================
%        TIME VECTOR
% ============================
% time   = hh:mm:ss (NTP)
% millis = Relative time Arduino

csv_file = "20260205_114534.csv";

% --- Extract the data ---
date_str = extractBefore(csv_file, "_");   % "20260205"

% --- Converts in datetime ---
file_date = datetime(date_str, ...
                     'InputFormat', 'yyyyMMdd', ...
                     'TimeZone', 'Europe/Zurich');

% --- Converts in string ---
ntp_time_str = string(T_alt.time);

% --- Datetime NTP (seconds) ---
t_ntp = datetime( ...
    datestr(file_date, 'yyyy-mm-dd') + " " + ntp_time_str, ...
    'InputFormat', 'yyyy-MM-dd HH:mm:ss', ...
    'TimeZone', 'Europe/Zurich');

% --- Calcola millisecondi relativi ---
millis_rel = T_alt.millis - T_alt.millis(1);

% --- Aggiunge millisecondi al datetime ---
% t_plot = t_ntp + milliseconds(millis_rel);
t_plot = millis_rel/1000;

% % --- Opzionale: forza il formato con millisecondi per la visualizzazione ---
% t_plot.Format = 'yyyy-MM-dd HH:mm:ss.SSS';


%% ============================
%   ACCELERATION
% ============================

figure;
plot(t_plot, T_alt.de_v_dt,       'LineWidth', 2, 'DisplayName', 'v derivative');
hold on;
plot(t_plot, T_alt.de_v_dt_filt,  'LineWidth', 2, 'DisplayName', 'v derivative (filtered)');
xlabel('Time (s)');
ylabel('Vertical acceleration (m/s^2)');
title('Acceleration');
grid on;
legend;

figure;
subplot(2,1,1)
plot(t_plot, T_alt.bat_voltage,'LineWidth', 2, 'DisplayName', 'v derivative');
xlabel('Time (s)');
ylabel('(V)');
grid on;
legend;

subplot(2,1,2)
plot(t_plot, T_alt.bat_percent,'LineWidth', 2, 'DisplayName', 'v derivative');
xlabel('Time (s)');
ylabel('%');
grid on;
legend;

% ---------- VELOCITY ERROR ----------
subplot(2,1,1);
plot(t_plot, T_alt.e_v, 'LineWidth', 2, 'DisplayName', 'e_v = v_{ref} - v_{hat}');
hold on;
ylabel('(m/s)');
title('Velocity Tracking Error');
grid on;
legend;

% ---------- INTEGRATED VELOCITY ERROR ----------
subplot(2,1,2);
plot(t_plot, T_alt.int_ev, 'LineWidth', 2, 'DisplayName', sprintf('I_{ev} = \\Sigma e_v \\Delta t = %.3f m', T_alt.int_ev(end)));
xlabel('Time (s)');
ylabel('(m/s)');
title('Integrated Velocity Error (I_{ev} = \int e_v dt)');
grid on;
legend;

%% ============================
%   ALTITUDE & VELOCITY TRACKING
% ============================

figure;

% ---------- ALTITUDE ----------
subplot(3,1,1);
plot(t_plot, T_alt.z_meas, 'LineWidth', 2, 'DisplayName', 'z_{meas}');
hold on;
plot(t_plot, T_alt.z_ref,  'LineWidth', 2, 'DisplayName', 'z_{ref}');
ylabel('Altitude (m)');
title('Altitude Tracking');
grid on;
legend;

% ---------- VELOCITY ----------
subplot(3,1,2);
plot(t_plot, T_alt.v_hat_butter, 'LineWidth', 2, 'DisplayName', 'v_{hat}');
hold on;
plot(t_plot, T_alt.v_ref,        'LineWidth', 2, 'DisplayName', 'v_{ref}');
xlabel('Time (s)');
ylabel('Velocity (m/s)');
title('Velocity Tracking');
grid on;
legend;

% ---------- CONTROL INPUT ----------
subplot(3,1,3);
plot(t_plot, T_alt.uz_pid, 'LineWidth', 2, 'DisplayName', 'u_z');
xlabel('Time (s)');
ylabel('Control');
title('Control Input');
grid on;
legend;

sgtitle(sprintf('Gains: Kp_v=%.2f, Kd_v=%.2f, Ki_v=%.2f, Kp_z=%.2f', ...
    T_alt.Kp_up(1,1), T_alt.Kd_up(1,1), T_alt.Ki_up(1,1), T_alt.Kp_z(1,1)));

