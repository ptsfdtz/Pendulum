function r = sp_metrics(t,X,u,stage,P,scenario)
r = struct('t',t,'X',X,'u',u,'stage',stage,'scenario',scenario,'config',P);
r.finite = all(isfinite([X(:);u(:)]));
r.max_abs_x = max(abs(X(:,1))); r.max_abs_u = max(abs(u));
r.track_ok = r.max_abs_x < P.x_limit;
r.stages_visited = unique(stage(:))';
a = atan2(sin(X(:,2)),cos(X(:,2))); tail = t >= t(end)-2;
r.settled = max(abs(a(tail))) < 0.05 && max(abs(X(tail,3:4)),[],'all') < 0.25;
r.passed = r.finite && r.track_ok && r.settled && ...
    abs(t(end)-P.stopTime) < 1e-8 && r.max_abs_u <= P.u_max+1e-9;
if strcmp(scenario,'swingup'), r.passed = r.passed && all(ismember([1 2],r.stages_visited)); end
end
