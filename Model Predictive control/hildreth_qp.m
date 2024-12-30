function eta = hildreth_qp(H, f, A_cons, b)

    [n_constraints, n_vars] = size(A_cons);

    % Initial unconstrained solution
    eta = -H \ f;

    % Check if initial solution satisfies constraints
    violated = 0;
    for i = 1:n_constraints
        if A_cons(i, :) * eta > b(i)
            violated = violated + 1;
        end
    end
    if violated == 0
        return; % Return if no constraints are violated
    end

    % Compute intermediate terms for the iterative solver
    P = A_cons * (H \ A_cons'); % Projected Hessian
    d = A_cons * (H \ f) + b;   % Adjusted constraints
    lambda = zeros(n_constraints, 1); % Initialize Lagrange multipliers

    % Iterative optimization
    max_iter = 500;    % Maximum number of iterations
    tolerance = 1e-8; % Convergence tolerance
    for km = 1:max_iter
        lambda_prev = lambda; % Store previous lambda values

        % Update each Lagrange multiplier iteratively
        for i = 1:n_constraints
            w = P(i, :) * lambda - P(i, i) * lambda(i);
            w = w + d(i);
            lambda(i) = max(0, -w / P(i, i)); % Ensure non-negativity
        end

        % Check for convergence
        al = (lambda - lambda_prev)' * (lambda - lambda_prev);
        if al < tolerance
            break;
        end
    end

    % Compute the constrained solution
    eta = -H \ f - H \ A_cons' * lambda;
end
