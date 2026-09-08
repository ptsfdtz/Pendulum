function [A, B, info] = dp_linearize(P, verbose)
%DP_LINEARIZE Numerically linearize dp_dynamics at X=0, u=0.

if nargin < 1 || isempty(P)
    P = dp_config();
end
if nargin < 2
    verbose = true;
end

x0 = zeros(6,1);
u0 = 0;
hx = 1e-6 * ones(6,1);
hu = 1e-6;
A = zeros(6,6);
for k = 1:6
    dx = zeros(6,1);
    dx(k) = hx(k);
    A(:,k) = (state_rhs(x0 + dx, u0, P) - state_rhs(x0 - dx, u0, P)) / (2*hx(k));
end
B = (state_rhs(x0, u0 + hu, P) - state_rhs(x0, u0 - hu, P)) / (2*hu);

info.controllability_rank = rank(ctrb(A,B));
info.A_reference = P.paper.A;
info.B_reference = P.paper.B;
info.A_max_abs_error = max(abs(A(:) - P.paper.A(:)));
info.B_max_abs_error = max(abs(B(:) - P.paper.B(:)));

if verbose
    fprintf('\nNumerical upright linearization:\nA =\n');
    disp(A);
    fprintf('B =\n');
    disp(B);
    fprintf('Controllability rank: %d / 6\n', info.controllability_rank);
    fprintf('Paper comparison: max|dA|=%.4g, max|dB|=%.4g\n', ...
        info.A_max_abs_error, info.B_max_abs_error);
end
if info.controllability_rank ~= 6
    error('dp:UncontrollableLinearization', ...
        'Numerical linearization is not controllable (rank %d, expected 6).', ...
        info.controllability_rank);
end
end

function dx = state_rhs(X, u, P)
[xdd, t1dd, t2dd] = dp_dynamics(u, X(2), X(5), X(3), X(6), P);
dx = [X(4); X(5); X(6); xdd; t1dd; t2dd];
end
