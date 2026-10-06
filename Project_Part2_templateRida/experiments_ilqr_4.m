clear;
clc;

run("setup_cartpole.m");

%% Experiment 1 - Horizon length

T_values = [4 6 8 10];

results_horizon = zeros(length(T_values),9);

for i = 1:length(T_values)

    % Horizon
    T_test = T_values(i);
    N_test = round(T_test/Ts);
    T_test = N_test*Ts;

    t_test = (0:N_test-1)*Ts;

    % Same initial-guess structure for every horizon
    U0_test = 15*sin(2*pi*0.45*t_test);

    % Run iLQR
    [X_test,U_test,~,cost_test] = ...
        ilqr(x0,U0_test,model,Ts,Q,R,Qf,Fmax);

    % Terminal state
    x_end = X_test(:,end);

    % Save results
    results_horizon(i,1) = T_test;
    results_horizon(i,2) = abs(x_end(1));
    results_horizon(i,3) = abs(x_end(2));
    results_horizon(i,4) = abs(rad2deg(x_end(3)));
    results_horizon(i,5) = abs(x_end(4));

    % Constraints
    results_horizon(i,6) = max(abs(X_test(1,:)));
    results_horizon(i,7) = max(abs(U_test));

    % Optimization performance
    results_horizon(i,8) = cost_test(end);
    results_horizon(i,9) = length(cost_test)-1;

end

horizon_table = array2table(results_horizon, ...
    'VariableNames', ...
    {'T_s', ...
    'p_final_m', ...
    'v_final_mps', ...
    'theta_final_deg', ...
    'omega_final_radps', ...
    'max_cart_m', ...
    'max_force_N', ...
    'final_cost', ...
    'accepted_iterations'});

disp(horizon_table)

%% Experiment 2 - Weight matrices

T_test = 8;
N_test = round(T_test/Ts);
t_test = (0:N_test-1)*Ts;

U0_test = 15*sin(2*pi*0.45*t_test);

% Different weight configurations
Q_tests = {
    diag([0.2, 0.02, 0.2, 0.02])
    diag([0.5, 0.02, 0.2, 0.02])
    diag([1.0, 0.02, 0.2, 0.02])
    diag([0.5, 0.02, 0.2, 0.02])
    };

R_tests = {
    0.01
    0.01
    0.01
    0.05
    };

Qf_tests = {
    diag([200, 50, 5000, 800])
    diag([300, 50, 6000, 1500])
    diag([500, 50, 6000, 1500])
    diag([300, 50, 6000, 1500])
    };

labels = {
    'Lower terminal penalty'
    'Baseline'
    'Higher cart penalty'
    'Higher control penalty'
    };

n_tests = length(labels);

results_weights = zeros(n_tests,8);

for i = 1:n_tests

    Q_test = Q_tests{i};
    R_test = R_tests{i};
    Qf_test = Qf_tests{i};

    [X_test,U_test,~,cost_test] = ...
        ilqr(x0,U0_test,model,Ts, ...
        Q_test,R_test,Qf_test,Fmax);

    x_end = X_test(:,end);

    results_weights(i,1) = abs(x_end(1));
    results_weights(i,2) = abs(x_end(2));
    results_weights(i,3) = abs(rad2deg(x_end(3)));
    results_weights(i,4) = abs(x_end(4));

    results_weights(i,5) = max(abs(X_test(1,:)));
    results_weights(i,6) = max(abs(U_test));

    results_weights(i,7) = cost_test(end);
    results_weights(i,8) = length(cost_test)-1;

end

weight_table = array2table(results_weights, ...
    'VariableNames', ...
    {'p_final_m', ...
    'v_final_mps', ...
    'theta_final_deg', ...
    'omega_final_radps', ...
    'max_cart_m', ...
    'max_force_N', ...
    'final_cost', ...
    'accepted_iterations'});

weight_table.Configuration = labels;

weight_table = movevars(weight_table, ...
    'Configuration','Before',1);

disp(weight_table)

%% Experiment 3 - Initial guess sensitivity

T_test = 8;
N_test = round(T_test/Ts);
t_test = (0:N_test-1)*Ts;

% Keep the same weights
Q_test  = diag([0.5, 0.02, 0.2, 0.02]);
R_test  = 0.01;
Qf_test = diag([300, 50, 6000, 1500]);

% Different initial control guesses
U0_tests = {
    zeros(1,N_test)
    15*sin(2*pi*0.45*t_test)
    8*sin(2*pi*0.45*t_test)
    -15*sin(2*pi*0.45*t_test)
    };

labels_guess = {
    'Zero guess'
    'Baseline sinusoid'
    'Lower amplitude sinusoid'
    'Opposite-phase sinusoid'
    };

n_tests = length(labels_guess);

results_guess = zeros(n_tests,8);

for i = 1:n_tests

    U0_test = U0_tests{i};

    [X_test,U_test,~,cost_test] = ...
        ilqr(x0,U0_test,model,Ts, ...
        Q_test,R_test,Qf_test,Fmax);

    x_end = X_test(:,end);

    results_guess(i,1) = abs(x_end(1));
    results_guess(i,2) = abs(x_end(2));
    results_guess(i,3) = abs(rad2deg(x_end(3)));
    results_guess(i,4) = abs(x_end(4));

    results_guess(i,5) = max(abs(X_test(1,:)));
    results_guess(i,6) = max(abs(U_test));

    results_guess(i,7) = cost_test(end);
    results_guess(i,8) = length(cost_test)-1;

end

guess_table = array2table(results_guess, ...
    'VariableNames', ...
    {'p_final_m', ...
    'v_final_mps', ...
    'theta_final_deg', ...
    'omega_final_radps', ...
    'max_cart_m', ...
    'max_force_N', ...
    'final_cost', ...
    'accepted_iterations'});

guess_table.InitialGuess = labels_guess;

guess_table = movevars(guess_table, ...
    'InitialGuess','Before',1);

disp(guess_table)

%% Experiment 4 - Problem description: force and cart limits

T_test = 8;
N_test = round(T_test/Ts);
t_test = (0:N_test-1)*Ts;

% Best initial guess found in Experiment 3
U0_test = -15*sin(2*pi*0.45*t_test);

Q_test  = diag([0.5, 0.02, 0.2, 0.02]);
R_test  = 0.01;
Qf_test = diag([300, 50, 6000, 1500]);

Fmax_values = [10 15 20 25];

results_force = zeros(length(Fmax_values),8);

for i = 1:length(Fmax_values)

    Fmax_test = Fmax_values(i);

    [X_test,U_test,~,cost_test] = ...
        ilqr(x0,U0_test,model,Ts, ...
        Q_test,R_test,Qf_test,Fmax_test);

    x_end = X_test(:,end);

    results_force(i,1) = Fmax_test;
    results_force(i,2) = abs(x_end(1));
    results_force(i,3) = abs(x_end(2));
    results_force(i,4) = abs(rad2deg(x_end(3)));
    results_force(i,5) = abs(x_end(4));
    results_force(i,6) = max(abs(X_test(1,:)));
    results_force(i,7) = max(abs(U_test));
    results_force(i,8) = cost_test(end);

end

force_table = array2table(results_force, ...
    'VariableNames', ...
    {'Fmax_N', ...
    'p_final_m', ...
    'v_final_mps', ...
    'theta_final_deg', ...
    'omega_final_radps', ...
    'max_cart_m', ...
    'max_force_N', ...
    'final_cost'});

disp(force_table)

%% Cart-displacement feasibility

xmax_values = [0.3 0.5 0.7];

cart_feasible = false(height(force_table),length(xmax_values));

for i = 1:height(force_table)
    for j = 1:length(xmax_values)
        cart_feasible(i,j) = ...
            force_table.max_cart_m(i) <= xmax_values(j);
    end
end

cart_feasibility_table = array2table(cart_feasible, ...
    'VariableNames',{'xmax_03','xmax_05','xmax_07'});

cart_feasibility_table.Fmax_N = force_table.Fmax_N;

cart_feasibility_table = movevars( ...
    cart_feasibility_table,'Fmax_N','Before',1);

disp(cart_feasibility_table)

%% Experiment 5 - Handling optimization failure

Fmax_fail = 10;

[X_fail,U_fail,~,cost_fail] = ...
    ilqr(x0,U0_test,model,Ts, ...
    Q_test,R_test,Qf_test,Fmax_fail);

x_fail = X_fail(:,end);

fprintf('\n--- Failure analysis ---\n');
fprintf('Fmax = %.1f N\n',Fmax_fail);
fprintf('|p_final|     = %.4f m\n',abs(x_fail(1)));
fprintf('|v_final|     = %.4f m/s\n',abs(x_fail(2)));
fprintf('|theta_final| = %.2f deg\n', ...
    abs(rad2deg(x_fail(3))));
fprintf('|omega_final| = %.4f rad/s\n', ...
    abs(x_fail(4)));
fprintf('Final cost    = %.2f\n',cost_fail(end));

t_fail_state = (0:size(X_fail,2)-1)*Ts;
t_fail_ctrl  = (0:length(U_fail)-1)*Ts;

figure
plot(t_fail_state,rad2deg(X_fail(3,:)), ...
    'LineWidth',1.4)
xlabel('Time [s]')
ylabel('\theta [deg]')
title('Failed Swing-up: F_{max} = 10 N')
grid on

figure
plot(t_fail_ctrl,U_fail,'LineWidth',1.4)
yline(Fmax_fail,'--')
yline(-Fmax_fail,'--')
xlabel('Time [s]')
ylabel('Force [N]')
title('Control Input During Failed Swing-up')
grid on

figure
plot(0:length(cost_fail)-1,cost_fail,'-o', ...
    'LineWidth',1.4)
xlabel('Accepted iLQR iteration')
ylabel('Cost')
title('Cost Evolution for Failed Swing-up')
grid on