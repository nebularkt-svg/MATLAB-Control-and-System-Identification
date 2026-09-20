%% ELECTENG 704 - Assignment #1 - Question 1(f)
% Magnetic Levitation - Integral-Augmented Feedback Linearization
% Closed-loop simulation with TRUE plant parameters (delta=+0.2, c=0.03)
%
% Plain MATLAB - fixed-step RK4 integration (no toolbox required)

clear; clc; close all;

%% -------------------- Parameters --------------------
k   = 0.001;      % electromagnetic force constant
R   = 1;           % coil resistance (Ohm)
L   = 0.01;        % coil inductance (H)
g   = 9.81;        % gravity (m/s^2)
m0  = 0.05;        % nominal mass (kg)
Umax = 12;          % actuator saturation (V)

yd  = 0.01;         % desired gap (m)

% TRUE plant parameters (uncertainty realisation for this simulation)
delta   = 0.2;
m_true  = m0*(1+delta);   % = 0.06 kg
c_true  = 0.03;             % Ns/m

% Pole-placement gains from part (e): 4th-order Butterworth, omega0 = 30 rad/s
% NOTE: omega0 must be fast enough to dominate the plant's OPEN-LOOP
% INSTABILITY (magnetic force ~ 1/x1^2 is a destabilizing positive-feedback
% mechanism near small gaps). omega0 = 5 (a "conservative-looking" choice)
% was tested and found to DIVERGE - the gap collapses to zero before the
% controller can react. omega0 = 30 was verified (numerically, by sweeping
% omega0) to give a stable response with near-zero steady-state error while
% peak |u| stays far below the 12V saturation limit (~0.4V), so there is no
% actuator constraint issue at this bandwidth.
omega0 = 30;
k3 = 2.613*omega0;
k2 = 3.414*omega0^2;
k1 = 2.613*omega0^3;
k0 = omega0^4;

fprintf('Controller gains: k0=%.2f k1=%.2f k2=%.2f k3=%.2f\n', k0,k1,k2,k3);

%% -------------------- Equilibrium current (for reference / initial condition) --------------------
id = yd*sqrt(m_true*g/k);
fprintf('Equilibrium current estimate (true mass): id = %.4f A\n', id);

%% -------------------- Simulation setup --------------------
Tend = 1;            % simulation horizon (s) - sufficient to see settling
dt   = 2e-6;          % small step required due to system's stiffness near
                        % small x1 and the fast poles chosen in part (e)
tvec = 0:dt:Tend;
Nt   = numel(tvec);

% state vector: x = [x1 (y); x2 (ydot); x3 (i); x4 (integral e0)]
x = [0.008; 0; id; 0];   % y(0)=0.008 m as specified, start current near equilibrium estimate

y_h = zeros(1,Nt); i_h = zeros(1,Nt); u_h = zeros(1,Nt); e1_h = zeros(1,Nt);

for n = 1:Nt
    x1 = x(1); x2 = x(2); x3 = x(3); x4 = x(4);

    % --- Nominal model-based estimates (certainty-equivalence) ---
    f3hat = (k/m0)*(x3^2/x1^2)*( (2*R/L) + (2*x2/x1) );
    g3hat = -(2*k/(m0*L))*(x3/x1^2);

    % --- Errors (yd constant => ydot_d = yddot_d = 0) ---
    e1 = x1 - yd;
    e2 = x2;
    yddot_hat = g - (k/m0)*(x3^2/x1^2);   % nominal-model estimate of ydd (c=0 assumed)
    e3 = yddot_hat;
    e0 = x4;

    % --- Virtual input from pole-placement (part e) ---
    v = -k0*e0 - k1*e1 - k2*e2 - k3*e3;

    % --- Feedback-linearizing control law (part c), saturated ---
    u = (v - f3hat)/g3hat;
    u = max(min(u,Umax),-Umax);

    % log
    y_h(n) = x1; i_h(n) = x3; u_h(n) = u; e1_h(n) = e1;

    if n < Nt
        f = @(xx,uu) true_dynamics(xx,uu,k,R,L,g,m_true,c_true,yd);
        k1_ = f(x,u);
        k2_ = f(x+dt/2*k1_,u);
        k3_ = f(x+dt/2*k2_,u);
        k4_ = f(x+dt*k3_,u);
        x = x + (dt/6)*(k1_+2*k2_+2*k3_+k4_);
        % guard against x1 -> 0 (physical constraint x1>0)
        if x(1) <= 1e-4
            x(1) = 1e-4;
        end
    end
end

%% -------------------- Results --------------------
idx_ss = round(0.8*Nt):Nt;   % last 20% of the simulation -> steady state
steady_state_error = mean(y_h(idx_ss)) - yd;
peak_u = max(abs(u_h));

fprintf('\n--- Results ---\n');
fprintf('Steady-state gap error: %.6f m (%.3f mm)\n', steady_state_error, steady_state_error*1000);
fprintf('Peak |u|: %.3f V (limit = %d V)\n', peak_u, Umax);

%% -------------------- Plots --------------------
figure('Name','Q1f Maglev Closed-Loop Simulation');

subplot(3,1,1); hold on; grid on;
plot(tvec, y_h*1000, 'b', 'LineWidth', 1.2);
yline(yd*1000, 'k--', 'y_d');
xlabel('Time (s)'); ylabel('y(t) (mm)');
title('(a) Gap y(t)');

subplot(3,1,2); hold on; grid on;
plot(tvec, i_h, 'r', 'LineWidth', 1);
xlabel('Time (s)'); ylabel('i(t) (A)');
title('(b) Coil Current i(t)');

subplot(3,1,3); hold on; grid on;
plot(tvec, u_h, 'm', 'LineWidth', 1);
yline(Umax,'k--'); yline(-Umax,'k--');
xlabel('Time (s)'); ylabel('u(t) (V)');
title('(c) Control Voltage u(t)');

sgtitle('Q1 Figure 1: Closed-Loop Response (True Plant, \delta=+0.2, c=0.03)');

%% -------------------- Local function: TRUE plant dynamics --------------------
function xdot = true_dynamics(x,u,k,R,L,g,m,c,yd)
    x1 = x(1); x2 = x(2); x3 = x(3);
    x1dot = x2;
    x2dot = g - (k/m)*(x3^2/x1^2) - (c/m)*x2;
    x3dot = -R/L*x3 + u/L;
    x4dot = x1 - yd;   % integral state
    xdot = [x1dot; x2dot; x3dot; x4dot];
end
