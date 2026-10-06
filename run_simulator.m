% =========================================================================
% PURE MATLAB SIMULATION: LQR + OBSERVER (NO SIMULINK, NO TOOLBOXES)
% =========================================================================

% --- 1. Physical and Model Parameters ---
p.m = 1.0; 
p.l = 0.5; 
p.g = 9.81; 
p.b = 0.1; 
p.J = p.m * p.l^2; 

m.m = p.m; 
m.l = p.l;
m.g = p.g;
m.b = p.b;
m.J = m.m * m.l^2;

Ts = 0.01; % Sampling time

% --- 2. Linearization & Discretization (Toolbox-Free) ---
Ac = [0, 1; (m.g / m.l), -m.b / m.J];
Bc = [0; 1 / m.J];

% Matrix Exponential for Zero-Order Hold (ZOH)
M = expm([Ac, Bc; 0, 0, 0] * Ts);
Ad = M(1:2, 1:2);
Bd = M(1:2, 3);
C = [1, 0]; 

% --- 3. Discrete-Time LQR Design (Toolbox-Free) ---
Q = [100, 0; 0, 10]; 
R = 1;               

% Solve Discrete Algebraic Riccati Equation iteratively
P = Q;
for iter = 1:1000
    P_next = Ad' * P * Ad - (Ad' * P * Bd) * ((R + Bd' * P * Bd) \ (Bd' * P * Ad)) + Q;
    P = P_next;
end
K = (R + Bd' * P * Bd) \ (Bd' * P * Ad);

% --- 4. Observer Design (Toolbox-Free) ---
lqr_poles = eig(Ad - Bd * K);
observer_poles = lqr_poles * 0.3; % Observer poles faster than controller

p1 = observer_poles(1);
p2 = observer_poles(2);
Delta_Ad = Ad^2 - (p1 + p2) * Ad + (p1 * p2) * eye(2);
Obs_mat = [C; C * Ad];
L = Delta_Ad * (Obs_mat \ [0; 1]);

% --- 5. Actuation Limits ---
umin = -2.3;
umax = 2.3;

% =========================================================================
% SIMULATION LOOP
% =========================================================================
T_end = 5; % Simulate for 5 seconds
N = round(T_end / Ts);

% Data logging arrays
t_log = zeros(1, N);
x_log = zeros(2, N);
xhat_log = zeros(2, N);
u_log = zeros(1, N);

% Initial Conditions
% We use 30 degrees (pi/6 rad) as an initial condition based on your evaluation tests
x = [pi/6; 0]; 
x_hat = [0; 0]; % Incorrect initial estimate to test observer convergence

for k = 1:N
    t_k = (k-1) * Ts;
    
    % 1. Measure output (Angle only)
    y = C * x; 
    
    % 2. Controller (with saturation)
    u_raw = -K * x_hat;
    u = min(max(u_raw, umin), umax);
    
    % 3. Observer Update (Compute next estimate)
    y_hat = C * x_hat;
    err = y - y_hat;
    x_hat_next = Ad * x_hat + Bd * u + L * err;
    
    % 4. Log data for plotting
    t_log(k) = t_k;
    x_log(:, k) = x;
    xhat_log(:, k) = x_hat;
    u_log(k) = u;
    
    % 5. Physical Plant Dynamics (Nonlinear, Continuous)
    % Solved using ode45 as requested in Section 1.1
    % Equation: J * theta_ddot = m*g*l*sin(theta) - b*theta_dot + u
    odefun = @(t, x_cont) [x_cont(2); ...
                           (p.m * p.g * p.l * sin(x_cont(1)) - p.b * x_cont(2) + u) / p.J];
                       
    [~, x_ode] = ode45(odefun, [t_k, t_k + Ts], x);
    
    % Extract the state at the end of the sampling period
    x = x_ode(end, :)';
    
    % Cycle observer estimate for the next loop iteration (acting as Unit Delay)
    x_hat = x_hat_next;
end

% =========================================================================
% PLOTTING RESULTS
% =========================================================================
figure('Name', 'LQR + Observer Control', 'Position', [100, 100, 800, 600]);

% Plot 1: Angle Tracking
subplot(2,1,1);
plot(t_log, x_log(1,:), 'b', 'LineWidth', 1.5); hold on;
plot(t_log, xhat_log(1,:), 'r--', 'LineWidth', 1.5);
yline(0, 'k:');
xlabel('Time [s]');
ylabel('Angle \theta [rad]');
legend('True Angle', 'Estimated Angle');
title('Pendulum Stabilization (30^\circ Initial Angle)');
grid on;

% Plot 2: Control Effort
subplot(2,1,2);
plot(t_log, u_log, 'g', 'LineWidth', 1.5); hold on;
yline(umax, 'r--', 'Upper Limit');
yline(umin, 'r--', 'Lower Limit');
xlabel('Time [s]');
ylabel('Control Torque u [Nm]');
legend('Applied Torque', 'Actuator Limits');
title('Actuator Effort under Saturation');
grid on;