function [t, X, u, failed, reason] = dp_simulate_numeric(K, initial_state, simulation_time, P)
%DP_SIMULATE_NUMERIC Fixed-step RK4 simulation of the nonlinear plant.

if nargin < 4 || isempty(P)
    P = dp_config();
end
if nargin < 3 || isempty(simulation_time)
    simulation_time = P.optimize.sim_time;
end

dt = P.optimize.dt;
n = ceil(simulation_time / dt);
t = (0:n)' * dt;
t(end) = simulation_time;
X = zeros(n+1,6);
u = zeros(n+1,1);
X(1,:) = initial_state(:)';
failed = false;
reason = '';
last = n + 1;

for k = 1:n
    h = t(k+1) - t(k);
    xk = X(k,:)';
    [k1, uk] = rhs(xk, K, P);
    k2 = rhs(xk + 0.5*h*k1, K, P);
    k3 = rhs(xk + 0.5*h*k2, K, P);
    k4 = rhs(xk + h*k3, K, P);
    xn = xk + h*(k1 + 2*k2 + 2*k3 + k4)/6;
    X(k+1,:) = xn';
    u(k) = uk;

    if any(~isfinite(xn))
        failed = true; reason = 'nonfinite_state'; last = k + 1; break;
    end
    if max(abs(xn)) > P.optimize.max_abs_state || ...
            max(abs(xn([5 6]))) > P.optimize.max_abs_angular_rate
        failed = true; reason = 'state_diverged'; last = k + 1; break;
    end
    if abs(xn(1)) >= P.x_limit
        failed = true; reason = 'track_limit'; last = k + 1; break;
    end
end

[~, u(last)] = rhs(X(last,:)', K, P);
t = t(1:last);
X = X(1:last,:);
u = u(1:last);
end

function [dx, u] = rhs(X, K, P)
u = -K * X;
u = min(max(u, -P.u_max), P.u_max);
[xdd, t1dd, t2dd] = dp_dynamics(u, X(2), X(5), X(3), X(6), P);
dx = [X(4); X(5); X(6); xdd; t1dd; t2dd];
end
