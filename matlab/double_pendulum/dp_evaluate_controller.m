function result = dp_evaluate_controller(K, P)
%DP_EVALUATE_CONTROLLER Score one gain across all configured nonlinear cases.

if nargin < 2 || isempty(P)
    P = dp_config();
end
states = P.optimize.test_initial_states;
ncase = size(states,1);
cases = repmat(empty_case(), ncase, 1);
total_cost = 0;
for i = 1:ncase
    [t, X, u, failed, reason] = dp_simulate_numeric( ...
        K, states(i,:), P.optimize.sim_time, P);
    cases(i) = score_case(t, X, u, failed, reason, P);
    if ~isfinite(cases(i).cost)
        total_cost = Inf;
    elseif isfinite(total_cost)
        total_cost = total_cost + cases(i).cost;
    end
end

result.cost = total_cost / ncase;
result.success_count = sum([cases.stable]);
result.success_rate = result.success_count / ncase;
result.worst_max_abs_x = max([cases.max_abs_x]);
result.max_abs_u = max([cases.max_abs_u]);
result.rms_u = sqrt(mean([cases.rms_u].^2));
result.control_energy = sum([cases.control_energy]);
result.settling_time = max([cases.settling_time]);
result.stable = result.success_count == ncase;
result.cases = cases;
end

function c = score_case(t, X, u, failed, reason, P)
c = empty_case();
c.failed = failed;
c.reason = reason;
c.t = t;
c.X = X;
c.u = u;
if isempty(t) || isempty(X) || any(~isfinite(X(:))) || any(~isfinite(u(:)))
    c.cost = Inf;
    return;
end

c.max_abs_x = max(abs(X(:,1)));
c.max_abs_u = max(abs(u));
c.rms_u = sqrt(mean(u.^2));
c.control_energy = trapz(t, u.^2);

window = t >= max(0, t(end) - P.optimize.settle_window);
within = abs(X(window,2)) < P.optimize.theta_tol ...
    & abs(X(window,3)) < P.optimize.theta_tol ...
    & abs(X(window,5)) < P.optimize.theta_dot_tol ...
    & abs(X(window,6)) < P.optimize.theta_dot_tol ...
    & abs(X(window,4)) < P.optimize.xdot_tol;
c.stable = ~failed && t(end) >= P.optimize.sim_time - eps && all(within);
c.settling_time = settling_time(t, X, P);

w = P.optimize.weights;
c.cost = w.theta1*trapz(t,X(:,2).^2) + w.theta2*trapz(t,X(:,3).^2) ...
    + w.x*trapz(t,X(:,1).^2) + w.theta1dot*trapz(t,X(:,5).^2) ...
    + w.theta2dot*trapz(t,X(:,6).^2) + w.u*c.control_energy ...
    + w.settling_time*c.settling_time;
c.cost = c.cost + w.max_x*c.max_abs_x^2;

if c.max_abs_x > 0.20
    c.cost = c.cost + 1e4*((c.max_abs_x - 0.20)/0.07)^2;
end
if c.max_abs_x > 0.27
    c.cost = c.cost + 1e6*((c.max_abs_x - 0.27)/0.03)^2;
end
sat_fraction = mean(abs(u) >= 0.99*P.u_max);
c.cost = c.cost + w.saturation*sat_fraction;
if ~c.stable
    c.cost = c.cost + P.optimize.penalty_unstable;
end
if c.max_abs_x >= P.x_limit || strcmp(reason,'track_limit')
    c.cost = c.cost + P.optimize.penalty_track;
end
if failed && ~strcmp(reason,'track_limit')
    c.cost = Inf;
end
end

function ts = settling_time(t, X, P)
good = abs(X(:,2)) < P.optimize.theta_tol ...
    & abs(X(:,3)) < P.optimize.theta_tol ...
    & abs(X(:,5)) < P.optimize.theta_dot_tol ...
    & abs(X(:,6)) < P.optimize.theta_dot_tol ...
    & abs(X(:,4)) < P.optimize.xdot_tol;
bad_after = flipud(cummax(flipud(~good)));
idx = find(~bad_after, 1, 'first');
if isempty(idx)
    ts = P.optimize.sim_time;
else
    ts = t(idx);
end
end

function c = empty_case()
c = struct('cost',Inf,'stable',false,'failed',true,'reason','not_run', ...
    'max_abs_x',Inf,'max_abs_u',Inf,'rms_u',Inf,'control_energy',Inf, ...
    'settling_time',Inf,'t',[],'X',[],'u',[]);
end
