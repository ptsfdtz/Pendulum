function L = dp_lqr_design(Q, R, verbose)
%DP_LQR_DESIGN Design an LQR from the numerically linearized nonlinear model.
P = dp_config();
if nargin < 1 || isempty(Q), Q = P.paper.Q; end
if nargin < 2 || isempty(R), R = P.paper.R; end
if nargin < 3, verbose = true; end
[A, B, linearization] = dp_linearize(P, verbose);
[K, S, poles] = lqr(A, B, Q, R);
L = struct('A',A,'B',B,'Q',Q,'R',R,'K',K,'S',S, ...
    'poles',poles,'linearization',linearization);
if verbose
    fprintf('LQR gain (u = -K*X):\n'); disp(K);
    fprintf('Closed-loop poles:\n'); disp(poles);
end
end
