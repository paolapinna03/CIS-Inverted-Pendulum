function J = trajectory_cost(X,U,Q,R,Qf)

N = numel(U);

J = 0;

% Running cost
for k = 1:N
    xk = X(:,k);
    uk = U(k);

    J = J + 0.5*(xk'*Q*xk + uk'*R*uk);
end

% Terminal cost
xN = X(:,N+1);
J = J + 0.5*xN'*Qf*xN;

end