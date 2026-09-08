function [u,stage] = dp_controller(t, x, xdot, theta1, theta1dot, theta2, theta2dot, controlMode)
%DP_CONTROLLER Code-generation-friendly runtime controller.
P = dp_config();
if nargin >= 8, P.control_mode = controlMode; end
persistent K
persistent swingStage
if isempty(K) || isempty(swingStage) || t <= 0
    K = zeros(1,6);
    swingStage = 1;
    if P.control_mode == 2
        L = dp_best_lqr();
        K = L.K;
    end
end

% Paper state order: [x theta1 theta2 xdot theta1dot theta2dot].
X = [x; theta1; theta2; xdot; theta1dot; theta2dot];
stage = 0;
switch P.control_mode
    case 0
        u = 0;
    case 1
        u = P.test_accel*cos(2*pi*P.test_frequency*t);
    case 2
        u = -K*X;
        stage = 3;
    case 4
        [u,swingStage] = dp_swingup_control(X,P,swingStage);
        stage = swingStage;
    otherwise
        u = 0;
end
u = min(max(u,-P.u_max),P.u_max);
end
