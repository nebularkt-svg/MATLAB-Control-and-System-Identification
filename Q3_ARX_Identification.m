%% ELECTENG 704 - Assignment #1 - Question 3
% PRBS excitation, ARX model estimation, cross-validation,
% and frequency response comparison
%
% Requires: System Identification Toolbox (idinput, iddata, arx, compare)
% Requires: Control System Toolbox (tf, c2d, freqresp)

clear; clc; close all;

%% -------------------- (a) PRBS excitation + ARX estimation --------------------

% --- 1. Define the true (original) system ---
num = [1 0.3 4 0.6 3];
den = [1 0.3 6 1.2 10 1.2 4];
sys_c = tf(num, den);              % continuous-time original system

Ts = 0.1;                           % sampling time (s)
N  = 2000;                          % number of samples

sys_d = c2d(sys_c, Ts, 'zoh');      % discretise for simulation/ID purposes

% --- 2. Generate PRBS input (2000 samples) ---
u = idinput(N, 'prbs');             % PRBS input, amplitude +/-1 by default
t = (0:N-1)'*Ts;                    % time vector

% --- 3. Simulate the discretised system to obtain the output ---
y = lsim(sys_d, u, t);

% Optional: add a small amount of measurement noise to make identification
% realistic (comment out if you want noise-free data).
rng(1);                             % for reproducibility
noise_std = 0.01 * std(y);
y = y + noise_std*randn(size(y));

% --- 3b. Plot the PRBS input and the resulting system output ---
% NOTE: over the full 200 s / 2000-sample record, the PRBS switches too
% fast for individual transitions to be visible (it just looks like a
% solid block). So we show BOTH a full-range overview AND a zoomed-in
% window so the actual switching pattern can be seen and graded.

t_zoom = 20;   % seconds shown in the zoomed-in subplot (adjust if needed)

figure('Name','PRBS Input and System Output');

% -- (1) PRBS input, zoomed in so switching is actually visible --
subplot(3,1,1);
stairs(t, u, 'b', 'LineWidth', 1);
xlabel('Time (s)');
ylabel('u(t)');
title(sprintf('(a) PRBS Input Signal - Zoomed In (First %d s)', t_zoom));
grid on;
ylim([-1.5 1.5]);
xlim([0 t_zoom]);

% -- (2) PRBS input, full record (overview only, switching not resolvable) --
subplot(3,1,2);
stairs(t, u, 'b', 'LineWidth', 0.5);
xlabel('Time (s)');
ylabel('u(t)');
title('(b) PRBS Input Signal - Full Record (2000 Samples, T_s = 0.1 s)');
grid on;
ylim([-1.5 1.5]);

% -- (3) System output, full record --
subplot(3,1,3);
plot(t, y, 'r', 'LineWidth', 1);
xlabel('Time (s)');
ylabel('y(t)');
title('(c) System Output Response to PRBS Input');
grid on;

sgtitle('Figure 1: PRBS Excitation and System Response');

% --- 4. Build iddata object and split into estimation / validation sets ---
data      = iddata(y, u, Ts);
data_est  = data(1:1000);           % samples 1-1000: used to FIT the model
data_val  = data(1001:2000);        % samples 1001-2000: used to VALIDATE

% --- 5. Estimate ARX models of different orders ---
% ARX structure: A(q) y(t) = B(q) u(t-nk) + e(t)
% orders = [na nb nk]
orders = { [2 2 1], [4 4 1], [6 6 1], [8 8 1] };
model_names = {'ARX[2 2 1]','ARX[4 4 1]','ARX[6 6 1]','ARX[8 8 1]'};

models = cell(1, numel(orders));
fit_pct = zeros(1, numel(orders));

for k = 1:numel(orders)
    models{k} = arx(data_est, orders{k});
    % Cross-validate on the held-out (validation) data
    [~, fitval] = compare(data_val, models{k});
    fit_pct(k) = fitval;
    fprintf('%s : fit to validation data = %.2f%%\n', model_names{k}, fitval);
end

% --- 6. Plot cross-validation comparison (measured vs simulated) ---
figure('Name','ARX Cross-Validation');
compare(data_val, models{:});
title('Figure 2: ARX Model Cross-Validation - Measured vs. Model Output (Validation Set)');
legend('Location','best');

%% -------------------- (b) Frequency response comparison (LINEAR scale) --------------------

% Choose a common frequency vector (rad/s), covering the system's bandwidth
w = linspace(0.01, pi/Ts, 1000);    % up to Nyquist frequency for fair comparison

% --- Original continuous system frequency response ---
H_true = squeeze(freqresp(sys_c, w));
mag_true = abs(H_true);
phase_true = unwrap(angle(H_true)) * 180/pi;

figure('Name','Frequency Response Comparison (Linear Scale)');

subplot(2,1,1); hold on; grid on;
plot(w, mag_true, 'k', 'LineWidth', 2, 'DisplayName', 'True system');

colors = lines(numel(models));
for k = 1:numel(models)
    H_est = squeeze(freqresp(models{k}, w));
    mag_est = abs(H_est);
    plot(w, mag_est, 'Color', colors(k,:), 'LineWidth', 1.2, ...
        'DisplayName', model_names{k});
end
xlabel('Frequency (rad/s)');
ylabel('Magnitude (linear)');
title('(a) Magnitude Response Comparison (Linear Scale, Not dB)');
legend('Location','best');

subplot(2,1,2); hold on; grid on;
plot(w, phase_true, 'k', 'LineWidth', 2, 'DisplayName', 'True system');
for k = 1:numel(models)
    H_est = squeeze(freqresp(models{k}, w));
    phase_est = unwrap(angle(H_est)) * 180/pi;
    plot(w, phase_est, 'Color', colors(k,:), 'LineWidth', 1.2, ...
        'DisplayName', model_names{k});
end
xlabel('Frequency (rad/s)');
ylabel('Phase (degrees)');
title('(b) Phase Response Comparison (Linear Scale)');
legend('Location','best');

sgtitle('Figure 3: Frequency Response Comparison');

%% -------------------- Summary table --------------------
fprintf('\n--- Summary of ARX model validation fit ---\n');
for k = 1:numel(orders)
    fprintf('%-12s : %.2f%% fit (validation)\n', model_names{k}, fit_pct(k));
end
