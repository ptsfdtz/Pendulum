function result = dp_simulate_swingup(P)
%DP_SIMULATE_SWINGUP Fast fixed-step nonlinear test of the swing-up logic.

if nargin < 1 || isempty(P)
    P = dp_config();
end
dt = P.swingup.numeric_dt;
n = ceil(P.stopTime/dt);
t = (0:n)'*dt;
X = zeros(n+1,6);
u = zeros(n+1,1);
stage = zeros(n+1,1);
X(1,:) = [P.x0 P.theta1_0 P.theta2_0 P.xdot0 ...
    P.theta1dot_0 P.theta2dot_0];
failed = false;
reason = '';
last = n+1;
currentStage = P.swingup.initial_stage;

for k = 1:n
    xk = X(k,:)';
    [uk,nextStage] = dp_swingup_control(xk,P,currentStage,true);
    % Match the digital Simulink implementation: the control command is
    % sampled once per controller tick and held throughout the RK4 step.
    k1 = localPlantRhs(xk,uk,P);
    k2 = localPlantRhs(xk+0.5*dt*k1,uk,P);
    k3 = localPlantRhs(xk+0.5*dt*k2,uk,P);
    k4 = localPlantRhs(xk+dt*k3,uk,P);
    xn = xk + dt*(k1+2*k2+2*k3+k4)/6;
    X(k+1,:) = xn';
    u(k) = uk;
    currentStage = nextStage;
    stage(k) = currentStage;
    if any(~isfinite(xn))
        failed = true; reason = 'nonfinite_state'; last = k+1; break;
    end
    if abs(xn(1)) >= P.swingup.hard_track_limit
        failed = true; reason = 'track_limit'; last = k+1; break;
    end
end
[u(last),stage(last)] = dp_swingup_control(X(last,:)',P,currentStage,true);
t = t(1:last); X = X(1:last,:); u = u(1:last); stage = stage(1:last);
wrappedAngles = atan2(sin(X(:,2:3)),cos(X(:,2:3)));
tailStart = max(1,size(X,1)-round(2/dt));
settled = all(max(abs(wrappedAngles(tailStart:end,:)),[],1) < 0.05) && ...
    max(abs(X(tailStart:end,4:6)),[],'all') < 0.25;
result = struct('t',t,'X',X,'u',u,'stage',stage,'failed',failed, ...
    'reason',reason,'settled',settled,'max_abs_x',max(abs(X(:,1))), ...
    'max_abs_u',max(abs(u)),'final_wrapped_angles',wrappedAngles(end,:), ...
    'stages_visited',unique(stage(:))');
end

function dx = localPlantRhs(X,u,P)
[xdd,t1dd,t2dd] = dp_dynamics(u,X(2),X(5),X(3),X(6),P);
dx = [X(4);X(5);X(6);xdd;t1dd;t2dd];
end
