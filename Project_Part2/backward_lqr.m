function [d,K] = backward_lqr(X,U,A,B,Q,R,Qf)

% A is of size (4,4,N)
% B is of size (4,1,N)

N = numel(U);

% Feedforward corrections
d = zeros(1,N);

% State-feedback gains
K = zeros(1,4,N);

% Terminal conditions
S = Qf;
s = Qf*X(:,N+1);

% Backward recursion
for k = N:-1:1

    Ak = A(:,:,k);
    Bk = B(:,:,k);

    xbar = X(:,k);
    ubar = U(k);

    % Keep S_{k+1} and s_{k+1}
    Snext = S;
    snext = s;

    % R + B' S B
    G = R + Bk'*Snext*Bk;

    % Feedback gain
    K(:,:,k) = G \ (Bk'*Snext*Ak);

    % Feedforward term
    d(k) = -G \ (R*ubar + Bk'*snext);

    % Cost-to-go recursion
    S = Q + Ak'*Snext*(Ak - Bk*K(:,:,k));

    s = Q*xbar ...
        + Ak'*snext ...
        + Ak'*Snext*Bk*d(k);

end

end