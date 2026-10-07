%% Physical parameters used by Simscape
%%%%% You can vary the parameters %%%%%%
param.m  = 1.0;     % Pendulum mass [kg]
param.M  = 1.0;     % Cart mass [kg]
param.L  = 0.5;     % Full pole length [m]
param.bc = 0.1;     % Cart damping [N*s/m]
param.bp = 0.05;    % Pivot damping [N*m*s/rad]
g=9.81;

%% Initial conditions: hanging down, stationary
%%%%% Vary the initial state %%%%%%
param.p0     = 0;
param.v0     = 0;
param.theta0 = 180; % Simscape joint target is in degrees (0 upright, 180 down)
param.omega0 = 0;

x0 = [param.p0; param.v0;
      deg2rad(param.theta0);param.omega0];

%% Parameters for nonlinear dynamics 
model.M   = param.M;
model.m   = param.m;
model.ell = param.L/2;     % Pivot-to-point-mass distance [m]
model.bc  = param.bc;
model.bp  = param.bp;
model.g   = g;

%%%%% You can vary these parameters %%%%%%
Ts   = 0.01;               % Sample time for discrete-time dynamics [s]
Fmax = 20;                 % Force saturation [N]
xmax = 0.5;                % Cart-travel limit [m] (can be arbitrarily large)

%% Continuous-time linearization at upright
% Upright = 0; positive angles are counterclockwise from 0
%%% You do not need to change this part %%%

M   = model.M;
m   = model.m;
ell = model.ell;

J = m*ell^2;
Delta = (M+m)*J - (m*ell)^2;

A = [0, 1, 0, 0;
     0, -J*model.bc/Delta, ...
         m^2*g*ell^2/Delta, -m*ell*model.bp/Delta;
     0, 0, 0, 1;
     0, -m*ell*model.bc/Delta, ...
         (M+m)*m*g*ell/Delta, -(M+m)*model.bp/Delta];

B = [0;
     J/Delta;
     0;
     m*ell/Delta];

%% Discrete-time LQR
%%% Obtain a discrete-time model of the continuous time system
sys_c = ss(A,B,eye(4),zeros(4,1));
sys_d = c2d(sys_c,Ts,'zoh');

Ad = sys_d.A;
Bd = sys_d.B;

%%% Compute the discrete LQR gain, select and justify weight matrices
Q_lqr = diag([10 1 100 10]);
R_lqr = 1;

K_lqr = dlqr(Ad,Bd,Q_lqr,R_lqr);


%% iLQR setup

T = 8.0;
N = round(T/Ts);
T = N*Ts;

t_ilqr = (0:N-1)*Ts;

% Stronger initial energy-pumping guess
U0 = 15*sin(2*pi*0.45*t_ilqr);

% Small running-state penalties
Q = diag([0.5, 0.02, 0.2, 0.02]);

R = 0.01;

% Strong terminal objective
Qf = diag([300, 50, 6000, 1500]);

%% Optimize swing-up

[Xnom,Unom,Knom,costHistory] = ilqr(x0,U0,model,Ts,Q,R,Qf,Fmax);

%% Check nominal trajectory

x_final = Xnom(:,end);

fprintf('\nFinal state:\n');
fprintf('p     = %.4f m\n',x_final(1));
fprintf('v     = %.4f m/s\n',x_final(2));
fprintf('theta = %.4f rad (%.2f deg)\n', ...
    x_final(3),rad2deg(x_final(3)));
fprintf('omega = %.4f rad/s\n',x_final(4));

max_force = max(abs(Unom));
max_cart_position = max(abs(Xnom(1,:)));

fprintf('\nTrajectory limits:\n');
fprintf('Maximum force = %.4f N\n',max_force);
fprintf('Maximum cart displacement = %.4f m\n', ...
    max_cart_position);

fprintf('Force limit satisfied: %d\n', ...
    max_force <= Fmax);

fprintf('Cart limit satisfied: %d\n', ...
    max_cart_position <= xmax);

fprintf('\nTerminal state:\n');
fprintf('|p|     = %.4f m\n',abs(x_final(1)));
fprintf('|v|     = %.4f m/s\n',abs(x_final(2)));
fprintf('|theta| = %.2f deg\n',abs(rad2deg(x_final(3))));
fprintf('|omega| = %.4f rad/s\n',abs(x_final(4)));

%% Plot results

t_state = (0:N)*Ts;
t_control = (0:N-1)*Ts;

figure

subplot(4,1,1)
plot(t_state,Xnom(1,:),'LineWidth',1.4)
ylabel('p [m]')
grid on

subplot(4,1,2)
plot(t_state,Xnom(2,:),'LineWidth',1.4)
ylabel('v [m/s]')
grid on

subplot(4,1,3)
plot(t_state,rad2deg(Xnom(3,:)),'LineWidth',1.4)
ylabel('\theta [deg]')
grid on

subplot(4,1,4)
plot(t_state,Xnom(4,:),'LineWidth',1.4)
ylabel('\omega [rad/s]')
xlabel('Time [s]')
grid on

figure
plot(t_control,Unom,'LineWidth',1.4)
yline(Fmax,'--')
yline(-Fmax,'--')
xlabel('Time [s]')
ylabel('Force [N]')
title('Optimized Swing-up Control')
grid on

figure
plot(0:length(costHistory)-1,costHistory,'-o','LineWidth',1.4)
xlabel('Accepted iLQR iteration')
ylabel('Cost')
title('iLQR Cost Convergence')
grid on

%% Configure and run Simulink simulation
% The helper configures the MATLAB Function block so that the iLQR nominal
% trajectory is tracked for the first T seconds. Afterwards the upright LQR
% takes over. The helper also fixes the MATLAB Function data dimensions to
% avoid variable-size/code-generation errors.
configure_cartpole_controller;
out = sim("cartpole");
