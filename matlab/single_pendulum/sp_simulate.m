function result = sp_simulate(scenario,P)
%SP_SIMULATE Sample/hold controller with nonlinear RK4 plant integration.
if nargin < 1, scenario = 'swingup'; end
if nargin < 2, P = sp_config(); end
scenario = validatestring(scenario,{'swingup','balance'});
balanceOnly = strcmp(scenario,'balance');
if balanceOnly, initial = P.balance_initial; else, initial = P.swing_initial; end
t = (0:ceil(P.stopTime/P.dt))'*P.dt; t(end) = P.stopTime;
X = zeros(numel(t),4); X(1,:) = initial(:)';
u = zeros(size(t)); stage = u; last = numel(t);
for k = 1:numel(t)-1
    x = X(k,:)'; h = t(k+1)-t(k);
    [u(k),stage(k)] = sp_control(x,P,balanceOnly);
    k1 = sp_dynamics(x,u(k),P);
    k2 = sp_dynamics(x+h*k1/2,u(k),P);
    k3 = sp_dynamics(x+h*k2/2,u(k),P);
    k4 = sp_dynamics(x+h*k3,u(k),P);
    X(k+1,:) = (x+h*(k1+2*k2+2*k3+k4)/6)';
    if any(~isfinite(X(k+1,:))) || abs(X(k+1,1)) >= P.x_limit
        last = k+1; break;
    end
end
[u(last),stage(last)] = sp_control(X(last,:)',P,balanceOnly);
result = sp_metrics(t(1:last),X(1:last,:),u(1:last),stage(1:last),P,scenario);
end
