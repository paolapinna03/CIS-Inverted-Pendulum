function [Xnew,Unew] = forward_pass(x0,X,U,d,K,alpha,model,Ts,Fmax)

% X, U: nominal trajectory
% d: feedforward corrections
% K: feedback gains
% alpha: line-search step size

N = numel(U);

% Initialization
Xnew = zeros(4,N+1);
Unew = zeros(1,N);

Xnew(:,1) = x0;

for k = 1:N

    % Feedback gain at time k
    Kk = reshape(K(:,:,k),1,4);

    % New control input
    Unew(k) = U(k) ...
        + alpha*d(k) ...
        - Kk*(Xnew(:,k) - X(:,k));

    % Force saturation
    Unew(k) = min(Fmax,max(-Fmax,Unew(k)));

    % Propagate nonlinear dynamics
    Xnew(:,k+1) = discrete_step( ...
        Xnew(:,k),Unew(k),model,Ts);

end

end