function [u, stage, terms] = dp_swingup_control(X, P, previousStage, allowTransition)
%DP_SWINGUP_CONTROL Paper-derived three-stage double-pendulum swing-up.
% X = [x theta1 theta2 xdot theta1dot theta2dot]'.

if nargin < 2 || isempty(P)
    P = dp_config();
end
if nargin < 3 || isempty(previousStage)
    previousStage = 1;
end
if nargin < 4
    allowTransition = true;
end

x = X(1);
theta1 = atan2(sin(X(2)),cos(X(2)));
theta2 = atan2(sin(X(3)),cos(X(3)));
xdot = X(4);
theta1dot = X(5);
theta2dot = X(6);

u1 = 0;
u21 = 0;
u22 = 0;
E1 = 0.5*P.J1*theta1dot^2 + ...
    P.m1*P.g*P.l1*(cos(theta1)-1);
E2 = 0.5*P.J2*theta2dot^2 + ...
    P.m2*P.g*P.l2*(cos(theta2)-1);

stage = previousStage;
if allowTransition && previousStage == 1 && ...
        abs(theta1) <= P.swingup.epsilon1 && ...
        abs(theta1dot) <= P.swingup.stage1_capture_rate
    stage = 2;
elseif allowTransition && previousStage == 2 && ...
        abs(theta1) > P.swingup.stage1_reentry_angle
    stage = 1;
elseif allowTransition && previousStage == 2 && ...
        abs(theta1) <= P.swingup.capture_angle1 && ...
        abs(theta2) <= P.swingup.capture_angle2 && ...
        abs(xdot) <= P.swingup.capture_cart_speed && ...
        x*xdot <= 0 && ...
        x*theta1 <= 0 && x*theta2 >= 0 && ...
        abs(theta1dot) <= P.swingup.capture_rate1 && ...
        abs(theta2dot) <= P.swingup.capture_rate2
    stage = 3;
end

if stage == 1
    sigma = pi - abs(theta1);
    if sigma <= P.swingup.sigma1
        beta1 = P.swingup.beta11;
    elseif sigma <= P.swingup.sigma2
        beta1 = P.swingup.beta12;
    else
        beta1 = P.swingup.beta13;
    end
    switching = (E1-P.swingup.energy1_target)*theta1dot*cos(theta1);
    requestedU1 = P.swingup.energy1_input_sign*beta1* ...
        localEnergySwitch(switching,P.swingup.energy_switch_smoothing);
    % Equation (26) assumes an ideal switch exactly at vmax. With a
    % sampled acceleration-input plant, retain any command that brakes the
    % cart and suppress only a command that would increase overspeed.
    if abs(xdot) < P.swingup.velocity_limit || requestedU1*xdot <= 0
        u1 = requestedU1;
    end
    u = u1;
elseif stage == 2
    singleState = [x;theta1;xdot;theta1dot];
    u21 = -P.swingup.Ks*singleState;
    if abs(theta2) > P.swingup.alpha
        u22 = P.swingup.second_input_sign* ...
            (-P.swingup.beta21*sign(theta2dot));
    else
        switching = (E2-P.swingup.energy2_target)*theta2dot*cos(theta2);
        u22 = P.swingup.second_input_sign* ...
            (-P.swingup.beta22*localEnergySwitch(switching, ...
            P.swingup.energy_switch_smoothing));
    end
    u = u21 + u22;
    % The paper uses a variable-gain LQR near the hand-off to enlarge the
    % capture region.  Use the validated full-state gain as a pre-capture
    % braking law, then latch stage 3 only after angular speed is low.
    if abs(theta1) <= P.swingup.capture_assist_angle1 && ...
            abs(theta2) <= P.swingup.capture_assist_angle2 && ...
            abs(theta1dot) <= P.swingup.capture_assist_rate1 && ...
            abs(theta2dot) <= P.swingup.capture_assist_rate2
        L = dp_best_lqr();
        assistState = [x;theta1;theta2;xdot;theta1dot;theta2dot];
        u = -L.K*assistState;
    end
else
    L = dp_best_lqr();
    balanceState = [x;theta1;theta2;xdot;theta1dot;theta2dot];
    u = -L.K*balanceState;
end

% Restricted-rail command conditioning. The paper limits velocity to keep
% cart travel bounded; the ideal acceleration plant has no friction, so a
% sampled simulation also needs an explicit stopping-distance guard.
if stage ~= 3 && abs(x) >= P.swingup.soft_track_limit && x*xdot > 0
    remaining = P.swingup.track_limit-P.swingup.track_brake_margin-abs(x);
    remaining = max(remaining,0.005);
    requiredBrake = xdot^2/(2*remaining);
    brakeMagnitude = max(P.swingup.minimum_brake_accel,requiredBrake);
    u = -sign(x)*min(brakeMagnitude,P.u_max);
end

u = min(max(u,-P.u_max),P.u_max);
terms = [u1;u21;u22;E1;E2];
end

function value = localEnergySwitch(switching,boundaryLayer)
if boundaryLayer > 0
    value = tanh(switching/boundaryLayer);
else
    value = sign(switching);
end
end
