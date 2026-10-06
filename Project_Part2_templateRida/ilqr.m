function [X,U,K,costHistory] = ilqr(x0,U0,model,Ts,Q,R,Qf,Fmax)

x0 = x0(:);

% Saturate initial control guess
U0 = min(Fmax,max(-Fmax,U0(:)'));

N = numel(U0);

% Initial trajectory generated with nonlinear dynamics
[X,U] = forward_pass( ...
    x0, ...
    zeros(4,N+1), ...
    U0, ...
    zeros(1,N), ...
    zeros(1,4,N), ...
    0, ...
    model,Ts,Fmax);

% Initial cost
J = trajectory_cost(X,U,Q,R,Qf);

costHistory = J;

% iLQR parameters
maxIter = 100;
tol = 1e-6;

% Line-search values
alphas = [1 0.5 0.25 0.1 0.05 0.01];

for iter = 1:maxIter

    % Linearize nonlinear dynamics along current trajectory
    [A,B] = linearize_trajectory(X,U,model,Ts);

    % Backward pass
    [d,K] = backward_lqr(X,U,A,B,Q,R,Qf);

    accepted = false;

    % Line search
    for alpha = alphas

        % Generate candidate trajectory
        [Xnew,Unew] = forward_pass( ...
            x0,X,U,d,K,alpha,model,Ts,Fmax);

        % Candidate cost
        Jnew = trajectory_cost(Xnew,Unew,Q,R,Qf);

        % Accept only if cost decreases
        if Jnew < J

            improvement = J - Jnew;

            X = Xnew;
            U = Unew;
            J = Jnew;

            costHistory(end+1) = J;

            accepted = true;

            break
        end

    end

    % Stop if no alpha produced an improvement
    if ~accepted
        break
    end

    % Convergence criterion
    if improvement < tol
        break
    end

end

% Final feedback gains along accepted nominal trajectory
[A,B] = linearize_trajectory(X,U,model,Ts);
[~,K] = backward_lqr(X,U,A,B,Q,R,Qf);

end