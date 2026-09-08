function dp_plot_optimization(candidates, best_result)
%DP_PLOT_OPTIMIZATION Create separate optimization and response figures.
costs = [candidates.cost]; plot_costs = costs;
plot_costs(~isfinite(plot_costs)) = NaN;
best_cost = cummin(costs); best_cost(~isfinite(best_cost)) = NaN;
figure('Name','LQR Best Cost');
semilogy(1:numel(best_cost),best_cost,'LineWidth',1.5); grid on;
xlabel('Trial'); ylabel('Best cost'); title('Trial vs best cost');
figure('Name','LQR Current Cost');
semilogy(1:numel(plot_costs),plot_costs,'.-'); grid on;
xlabel('Trial'); ylabel('Current cost'); title('Trial vs current cost');
if isempty(best_result.cases), return; end
[~, i] = max([best_result.cases.cost]); c = best_result.cases(i);
names = {'theta1','theta2','x','u'};
y = {c.X(:,2),c.X(:,3),c.X(:,1),c.u};
units = {'rad','rad','m','m/s^2'};
for k = 1:4
    figure('Name',['Best LQR ' names{k}]);
    plot(c.t,y{k},'LineWidth',1.3); grid on;
    xlabel('Time (s)'); ylabel([names{k} ' (' units{k} ')']);
    title(['Best controller ' names{k} '(t), worst test case']);
end
end
