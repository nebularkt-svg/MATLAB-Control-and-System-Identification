%% ELECTENG 704 - Assignment #1 - Question 2
% Sliding-Mode Control of a DC Motor Position System
% Parts (c) Boundary-layer (chattering vs accuracy) and
%       (d) Robustness stress test
%
% Model:      theta_ddot = -a1*theta_dot + a2*u + d(t)
% Sliding surface: s = e_dot + lambda*e,  e = theta - theta_d
% Control:    u = u_eq - K*sat(s/phi)
%
% No special toolbox required - plain MATLAB (ODE integrated manually
% with fixed-step RK4 so that the exact control history u(t), s(t) can
% be logged alongside the states).

clear; clc; close all;

%% -------------------- Fixed problem parameters --------------------
a1_nom = 20;         % nominal a1 (used by the controller's u_eq, ALWAYS)
a2_nom = 8;           % nominal a2 (used by the controller's u_eq, ALWAYS)

D      = 0.6;         % disturbance bound
eta    = 0.1;         % reaching-law design margin (from part b)
K      = 0.1;         % switching gain, K >= (D+eta)/a2_nom = 0.0875 -> chose 0.1

lambda = 5;           % sliding surface slope (design choice, >0)

% Reference trajectory: theta_d(t) = 0.5 sin(1.5 t)
theta_d      = @(t) 0.5*sin(1.5*t);
theta_d_dot  = @(t) 0.75*cos(1.5*t);
theta_d_ddot = @(t) -1.125*sin(1.5*t);

% Matched disturbance
d_fun = @(t) 0.5*sin(5*t);

% Simulation time
Tend = 10;
dt   = 0.001;
tvec = 0:dt:Tend;
Nt   = numel(tvec);

%% -------------------- Controller definition --------------------
% u_eq always uses the NOMINAL parameters a1_nom, a2_nom (the controller
% never "knows" the true plant parameters - that's the point of part d).
u_eq_fun = @(th_dot,t) (1/a2_nom)*( theta_d_ddot(t) + a1_nom*th_dot ...
                                     - lambda*(th_dot - theta_d_dot(t)) );

sat_fun = @(x) max(min(x,1),-1);   % standard saturation function

%% ==================== PART (c): Chattering vs Tracking Accuracy ====================
% Three-way comparison: pure sign(s) (no boundary layer, no saturation -
% this is the TRUE discontinuous sliding-mode control and will show real
% high-frequency chattering) vs. two boundary-layer widths phi=0.02, 0.1.
% Plant = nominal (matched case) throughout, so boundary-layer width /
% switching law is the only variable being compared.

phi_list = [0.02, 0.1];
results_c = struct();

% --- Case 0: pure sign(s), NO boundary layer, NO saturation on u ---
[s_sign, u_sign, e_sign, ~] = simulate_smc(tvec, dt, a1_nom, a2_nom, ...
    u_eq_fun, K, NaN, lambda, theta_d, theta_d_dot, d_fun, sat_fun, 'sign');

% --- Cases 1-2: boundary-layer sat(s/phi) ---
for kk = 1:numel(phi_list)
    phi = phi_list(kk);
    [s_h, u_h, e_h, th_h] = simulate_smc(tvec, dt, a1_nom, a2_nom, ...
        u_eq_fun, K, phi, lambda, theta_d, theta_d_dot, d_fun, sat_fun, 'sat');
    results_c(kk).phi = phi;
    results_c(kk).s = s_h;
    results_c(kk).u = u_h;
    results_c(kk).e = e_h;
end

% --- Plot: s(t), u(t), e(t) - sign(s) vs both boundary-layer widths ---
% NOTE: over the full 10 s record, the sign(s) switching is too fast to
% see individual transitions (it looks like a solid/fuzzy band). So we
% show a zoomed-in window for u(t) specifically, where the chattering is
% the point of interest, alongside the full-record view for s(t)/e(t).

t_zoom_lo = 1;   % seconds - zoom window for the chattering close-up
t_zoom_hi = 2;

figure('Name','Q2c Chattering Comparison');

subplot(4,1,1); hold on; grid on;
plot(tvec, s_sign, 'k', 'LineWidth', 0.6);
plot(tvec, results_c(1).s, 'b', 'LineWidth', 1);
plot(tvec, results_c(2).s, 'r', 'LineWidth', 1);
ylabel('s(t)');
legend('sign(s), no boundary layer','\phi = 0.02','\phi = 0.1','Location','best');
title('(a) Sliding Surface s(t) - Full Record');

subplot(4,1,2); hold on; grid on;
plot(tvec, u_sign, 'k', 'LineWidth', 0.4);
plot(tvec, results_c(1).u, 'b', 'LineWidth', 0.8);
plot(tvec, results_c(2).u, 'r', 'LineWidth', 0.8);
ylabel('u(t)');
legend('sign(s), no boundary layer','\phi = 0.02','\phi = 0.1','Location','best');
title('(b) Control Input u(t) - Full Record (chattering not resolvable at this scale)');

subplot(4,1,3); hold on; grid on;
plot(tvec, u_sign, 'k', 'LineWidth', 0.6);
plot(tvec, results_c(1).u, 'b', 'LineWidth', 1.2);
plot(tvec, results_c(2).u, 'r', 'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('u(t)');
xlim([t_zoom_lo t_zoom_hi]);
legend('sign(s), no boundary layer','\phi = 0.02','\phi = 0.1','Location','best');
title(sprintf('(c) Control Input u(t) - Zoomed In (%d-%d s) to Show Chattering', t_zoom_lo, t_zoom_hi));

subplot(4,1,4); hold on; grid on;
plot(tvec, e_sign, 'k', 'LineWidth', 0.6);
plot(tvec, results_c(1).e, 'b', 'LineWidth', 1);
plot(tvec, results_c(2).e, 'r', 'LineWidth', 1);
xlabel('Time (s)'); ylabel('e(t)');
legend('sign(s), no boundary layer','\phi = 0.02','\phi = 0.1','Location','best');
title('(d) Tracking Error e(t) - Full Record');

sgtitle('Q2 Figure 1: Chattering vs. Tracking Accuracy - sign(s) vs. Boundary-Layer Control');

% --- Quantify chattering and accuracy for the discussion ---
idx_ss = round(0.8*Nt):Nt;

du_sign = diff(u_sign)/dt;
chat_sign = mean(abs(du_sign));
zc_sign = sum(diff(sign(s_sign)) ~= 0);
rms_sign = rms(e_sign(idx_ss));
fprintf('sign(s), no BL : chattering index = %.3f, zero-crossings of s = %d, steady-state RMS error = %.5f\n', ...
    chat_sign, zc_sign, rms_sign);

for kk = 1:numel(phi_list)
    u_h = results_c(kk).u;
    e_h = results_c(kk).e;
    s_h = results_c(kk).s;
    du = diff(u_h)/dt;
    chattering_index = mean(abs(du));
    zc = sum(diff(sign(s_h)) ~= 0);
    e_rms_ss = rms(e_h(idx_ss));
    fprintf('phi = %.2f     : chattering index = %.3f, zero-crossings of s = %d, steady-state RMS error = %.5f\n', ...
        phi_list(kk), chattering_index, zc, e_rms_ss);
end

%% ==================== PART (d): Robustness Stress Test ====================
% Perturb the ACTUAL plant, while u_eq still uses the NOMINAL parameters
a1_true = 24;
a2_true = 6.5;
phi_d   = 0.05;   % representative boundary layer for this test

[s_nom, u_nom, e_nom, ~] = simulate_smc(tvec, dt, a1_nom, a2_nom, ...
    u_eq_fun, K, phi_d, lambda, theta_d, theta_d_dot, d_fun, sat_fun, 'sat');

[s_pert, u_pert, e_pert, ~] = simulate_smc(tvec, dt, a1_true, a2_true, ...
    u_eq_fun, K, phi_d, lambda, theta_d, theta_d_dot, d_fun, sat_fun, 'sat');

figure('Name','Q2d Robustness Stress Test');

subplot(3,1,1); hold on; grid on;
plot(tvec, s_nom, 'b', 'LineWidth', 1);
plot(tvec, s_pert, 'r', 'LineWidth', 1);
ylabel('s(t)');
legend('Nominal plant (a_1=20, a_2=8)','Perturbed plant (a_1=24, a_2=6.5)','Location','best');
title('(a) Sliding Surface s(t)');

subplot(3,1,2); hold on; grid on;
plot(tvec, u_nom, 'b', 'LineWidth', 0.8);
plot(tvec, u_pert, 'r', 'LineWidth', 0.8);
ylabel('u(t)');
legend('Nominal plant','Perturbed plant','Location','best');
title('(b) Control Input u(t)');

subplot(3,1,3); hold on; grid on;
plot(tvec, e_nom, 'b', 'LineWidth', 1);
plot(tvec, e_pert, 'r', 'LineWidth', 1);
xlabel('Time (s)'); ylabel('e(t)');
legend('Nominal plant','Perturbed plant','Location','best');
title('(c) Tracking Error e(t)');

sgtitle('Q2 Figure 2: Robustness Stress Test - Parameter Mismatch (u_{eq} uses nominal values)');

idx_ss = round(0.8*Nt):Nt;
fprintf('\n--- Robustness stress test (phi = %.2f) ---\n', phi_d);
fprintf('Nominal plant     : steady-state RMS error = %.5f\n', rms(e_nom(idx_ss)));
fprintf('Perturbed plant   : steady-state RMS error = %.5f\n', rms(e_pert(idx_ss)));

%% ==================== Local function: SMC simulation (fixed-step RK4) ====================
function [s_h, u_h, e_h, th_h] = simulate_smc(tvec, dt, a1_true, a2_true, ...
    u_eq_fun, K, phi, lambda, theta_d, theta_d_dot, d_fun, sat_fun, mode)

    Nt = numel(tvec);
    s_h  = zeros(1,Nt);
    u_h  = zeros(1,Nt);
    e_h  = zeros(1,Nt);
    th_h = zeros(1,Nt);

    % initial conditions: start with a small tracking error
    theta     = 0;
    theta_dot = 0;

    % dynamics function: xdot = f(t, x, u)
    f = @(t,x,u) [x(2);
                  -a1_true*x(2) + a2_true*u + d_fun(t)];

    for i = 1:Nt
        t = tvec(i);
        e     = theta - theta_d(t);
        e_dot = theta_dot - theta_d_dot(t);
        s     = e_dot + lambda*e;

        if strcmp(mode, 'sign')
            % TRUE discontinuous sliding-mode control: no boundary layer,
            % no saturation - this is the theoretical ideal that produces
            % real high-frequency chattering (bang-bang switching).
            u = u_eq_fun(theta_dot, t) - K*sign(s);
        else
            % Boundary-layer (continuous) approximation
            u = u_eq_fun(theta_dot, t) - K*sat_fun(s/phi);
        end

        % log
        s_h(i)  = s;
        u_h(i)  = u;
        e_h(i)  = e;
        th_h(i) = theta;

        if i < Nt
            % RK4 step (holding u constant over the step - standard ZOH assumption)
            x  = [theta; theta_dot];
            k1 = f(t,        x,            u);
            k2 = f(t+dt/2,   x+dt/2*k1,    u);
            k3 = f(t+dt/2,   x+dt/2*k2,    u);
            k4 = f(t+dt,     x+dt*k3,      u);
            x  = x + (dt/6)*(k1+2*k2+2*k3+k4);
            theta     = x(1);
            theta_dot = x(2);
        end
    end
end
