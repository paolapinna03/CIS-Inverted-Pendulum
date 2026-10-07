
% Simulation parameters
T_sim = T + 2; % Simulate for the swing-up horizon (T) plus 2 extra seconds for stabilization
N_sim = round(T_sim / Ts);

% Data logging arrays
t_log = (0:N_sim-1) * Ts;
x_log = zeros(4, N_sim);
u_log = zeros(1, N_sim);

% Initial State 
x = x0; 

% Capture zone thresholds 
theta_thresh = 0.35; % ~20 degrees
omega_thresh = 2.0;  % rad/s

N_traj = numel(Unom); % Length of the nominal iLQR trajectory

disp('Running pure MATLAB simulation...');

for k = 1:N_sim
    % 1. Log current state
    x_log(:, k) = x;
    
    % 2. Angle Wrapping
    % Wrap the current angle theta to be within [-pi, pi]
    x_wrap = x;
    x_wrap(3) = atan2(sin(x(3)), cos(x(3))); 
    
    % 3. Check Capture Zone
    in_capture_zone = abs(x_wrap(3)) < theta_thresh && abs(x_wrap(4)) < omega_thresh;
    
    % 4. Switching Logic & Controller
    if in_capture_zone
        % LQR Stabilization Mode
        u_raw = -K_lqr * x_wrap;
        
    elseif k <= N_traj
        % iLQR Trajectory Tracking Mode
        x_nom_k = Xnom(:, k);
        error_x = x_wrap - x_nom_k;
        
        % Wrap the angular error
        error_x(3) = atan2(sin(error_x(3)), cos(error_x(3)));
        
        % Tracking control law 
        u_raw = Unom(k) - Knom(:, :, k) * error_x;
        
    else
        % Failure / Drift Mode
        u_raw = 0; 
    end
    
    % 5. Actuator Saturation
    u = min(max(u_raw, -Fmax), Fmax);
    u_log(k) = u;
    
    % 6. Physical Plant Dynamics
    % Propagates the system using the provided 4th-order Runge-Kutta integrator
    x = discrete_step(x, u, model, Ts);
end

disp('Simulation complete! Plotting results...');

% Plotting results
figure('Name', 'Cart-Pole Swing-Up and Stabilization', 'Position', [100, 100, 900, 600]);

% Plot 1: Pendulum Angle
subplot(3,1,1);
plot(t_log, x_log(3,:), 'b', 'LineWidth', 1.5); hold on;
yline(0, 'k--'); % Upright equilibrium
yline(pi, 'r:'); % Downward equilibrium
yline(-pi, 'r:'); 
ylabel('Angle \theta [rad]');
title('Pendulum Angle');
grid on;

% Plot 2: Cart Position
subplot(3,1,2);
plot(t_log, x_log(1,:), 'm', 'LineWidth', 1.5); hold on;
yline(xmax, 'r--', 'Limit');
yline(-xmax, 'r--', 'Limit');
ylabel('Position p [m]');
title('Cart Position');
grid on;

% Plot 3: Control Force
subplot(3,1,3);
plot(t_log, u_log, 'g', 'LineWidth', 1.5); hold on;
yline(Fmax, 'r--', 'Max Force');
yline(-Fmax, 'r--', 'Min Force');
xlabel('Time [s]');
ylabel('Force u [N]');
title('Actuator Effort');
grid on;