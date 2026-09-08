function result = dp_optimize_stage2_robust()
%DP_OPTIMIZE_STAGE2_ROBUST Deterministic nonlinear stage-2 robustness search.
% No hardware logs are used. Cases span symmetric initial states and three
% bounded actuator/joint-friction profiles.

% Rows: [x theta1 theta2 xdot theta1dot theta2dot].
initialStates = [ ...
     0.00  0.10  pi-0.10  0.00  0.00  0.00;
     0.00 -0.10 -pi+0.10  0.00  0.00  0.00;
     0.10  0.20  2.50    -0.05  0.50 -2.00;
    -0.10 -0.20 -2.50     0.05 -0.50  2.00;
     0.05  0.05  1.50     0.00  0.00  3.00;
    -0.05 -0.05 -1.50     0.00  0.00 -3.00];

% [actuator gain, acceleration deadzone, b1, b2, coulomb1, coulomb2].
nonidealities = [ ...
    1.00 0.00 0.0000 0.0000 0.0000 0.0000;
    0.80 0.10 0.0005 0.0002 0.0002 0.0001;
    0.65 0.20 0.0010 0.0005 0.0004 0.0002];

% [beta21, beta22, capture-assist rate2, E2 target].
candidates = [ ...
    1.10 3.30  4.0 0.010;
    1.10 4.00  6.0 0.010;
    1.10 5.00  9.0 0.010;
    1.10 6.00  9.0 0.005;
    0.80 5.00  9.0 0.005;
    1.40 5.00  9.0 0.005;
    0.80 6.00 12.0 0.000;
    1.10 6.00 12.0 0.000;
    1.40 6.00 12.0 0.000;
    1.10 8.00 12.0 0.000;
    1.40 8.00 12.0 0.005;
    0.80 8.00  9.0 0.000];

nCandidate = size(candidates,1);
nCase = size(initialStates,1)*size(nonidealities,1);
captures = zeros(nCandidate,1);
settled = zeros(nCandidate,1);
trackFailures = zeros(nCandidate,1);
captureTimeSum = zeros(nCandidate,1);
meanMaxX = zeros(nCandidate,1);
worstMaxX = zeros(nCandidate,1);
meanMaxU = zeros(nCandidate,1);

for candidateIndex = 1:nCandidate
    for profileIndex = 1:size(nonidealities,1)
        for stateIndex = 1:size(initialStates,1)
            P = dp_config();
            P.stopTime = 25;
            P.swingup.numeric_dt = 0.002;
            P.swingup.initial_stage = 2;
            P.x0 = initialStates(stateIndex,1);
            P.theta1_0 = initialStates(stateIndex,2);
            P.theta2_0 = initialStates(stateIndex,3);
            P.xdot0 = initialStates(stateIndex,4);
            P.theta1dot_0 = initialStates(stateIndex,5);
            P.theta2dot_0 = initialStates(stateIndex,6);
            P.simulation.actuator_gain = nonidealities(profileIndex,1);
            P.simulation.acceleration_deadzone = nonidealities(profileIndex,2);
            P.simulation.joint1_viscous = nonidealities(profileIndex,3);
            P.simulation.joint2_viscous = nonidealities(profileIndex,4);
            P.simulation.joint1_coulomb = nonidealities(profileIndex,5);
            P.simulation.joint2_coulomb = nonidealities(profileIndex,6);
            P.swingup.beta21 = candidates(candidateIndex,1);
            P.swingup.beta22 = candidates(candidateIndex,2);
            P.swingup.capture_assist_rate2 = candidates(candidateIndex,3);
            P.swingup.energy2_target = candidates(candidateIndex,4);
            one = dp_simulate_swingup(P);
            captureIndex = find(one.stage == 3,1,'first');
            if ~isempty(captureIndex)
                captures(candidateIndex) = captures(candidateIndex)+1;
                captureTimeSum(candidateIndex) = captureTimeSum(candidateIndex)+ ...
                    one.t(captureIndex);
            end
            settled(candidateIndex) = settled(candidateIndex)+double(one.settled);
            trackFailures(candidateIndex) = trackFailures(candidateIndex)+ ...
                double(one.failed && strcmp(one.reason,'track_limit'));
            meanMaxX(candidateIndex) = meanMaxX(candidateIndex)+one.max_abs_x/nCase;
            worstMaxX(candidateIndex) = max(worstMaxX(candidateIndex),one.max_abs_x);
            meanMaxU(candidateIndex) = meanMaxU(candidateIndex)+one.max_abs_u/nCase;
        end
    end
    fprintf('Candidate %d/%d: captures=%d/%d, track failures=%d\n', ...
        candidateIndex,nCandidate,captures(candidateIndex),nCase, ...
        trackFailures(candidateIndex));
end

meanCaptureTime = inf(nCandidate,1);
hasCapture = captures > 0;
meanCaptureTime(hasCapture) = captureTimeSum(hasCapture)./captures(hasCapture);
ranking = table((1:nCandidate)',candidates(:,1),candidates(:,2), ...
    candidates(:,3),candidates(:,4),captures,settled,trackFailures, ...
    meanCaptureTime,meanMaxX,worstMaxX,meanMaxU, ...
    'VariableNames',{'candidate','far_gain','near_gain','assist_rate2', ...
    'energy2_target','captures','settled','track_failures', ...
    'mean_capture_time','mean_max_x','worst_max_x','mean_max_u'});
ranking = sortrows(ranking,{'captures','settled','track_failures', ...
    'mean_capture_time','worst_max_x'}, ...
    {'descend','descend','ascend','ascend','ascend'});
result = struct('initial_states',initialStates, ...
    'nonidealities',nonidealities,'ranking',ranking,'best',ranking(1,:));
disp(ranking);
end
