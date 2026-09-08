function summary = dp_evaluate_stage2_candidate(candidateId,parameters,stopTime,outputFile,stateSelection,profileSelection,scenarioKind)
%DP_EVALUATE_STAGE2_CANDIDATE Long-horizon, model-only robustness test.
% parameters = [beta21 beta22 capture_assist_rate2 energy2_target] and,
% optionally, [stage1_gain_scale stage1_velocity_limit numeric_dt
% energy_switch_smoothing].

if nargin < 3 || isempty(stopTime), stopTime = 50; end
if nargin < 4 || isempty(outputFile)
    outputFile = fullfile(dp_output_dir(),sprintf('stage2_candidate_%d.csv',candidateId));
end
if nargin < 5 || isempty(stateSelection), stateSelection = 1:6; end
if nargin < 6 || isempty(profileSelection), profileSelection = 1:3; end
if nargin < 7 || isempty(scenarioKind), scenarioKind = 'stage2'; end

initialStates = [ ...
     0.00  0.10  pi-0.10  0.00  0.00  0.00;
     0.00 -0.10 -pi+0.10  0.00  0.00  0.00;
     0.10  0.20  2.50    -0.05  0.50 -2.00;
    -0.10 -0.20 -2.50     0.05 -0.50  2.00;
     0.05  0.05  1.50     0.00  0.00  3.00;
    -0.05 -0.05 -1.50     0.00  0.00 -3.00];
initialStage = 2;
if strcmp(scenarioKind,'full')
    initialStates = [ ...
         0.00  pi-0.02  pi-0.04  0.00  0.00  0.00;
         0.00 -pi+0.02 -pi+0.04  0.00  0.00  0.00;
         0.08  pi-0.05  pi-0.02 -0.03  0.00  0.00;
        -0.08 -pi+0.05 -pi+0.02  0.03  0.00  0.00;
         0.00  pi-0.08 -pi+0.06  0.00  0.15 -0.10;
         0.00 -pi+0.08  pi-0.06  0.00 -0.15  0.10];
    initialStage = 1;
end

% [actuator gain, acceleration deadzone, b1, b2, coulomb1, coulomb2].
nonidealities = [ ...
    1.00 0.00 0.0000 0.0000 0.0000 0.0000;
    0.80 0.10 0.0005 0.0002 0.0002 0.0001;
    0.65 0.20 0.0010 0.0005 0.0004 0.0002];

nCase = numel(stateSelection)*numel(profileSelection);
profile = zeros(nCase,1); initial_state = zeros(nCase,1);
captured = false(nCase,1); settled = false(nCase,1);
failed = false(nCase,1); track_failure = false(nCase,1);
capture_time = nan(nCase,1); max_abs_x = zeros(nCase,1);
max_abs_u = zeros(nCase,1); final_angle1 = zeros(nCase,1);
final_angle2 = zeros(nCase,1); stages = strings(nCase,1);

row = 0;
for profileIndex = profileSelection
    for stateIndex = stateSelection
        row = row+1;
        P = dp_config();
        P.stopTime = stopTime;
        P.swingup.numeric_dt = 0.002;
        P.swingup.initial_stage = initialStage;
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
        P.swingup.beta21 = parameters(1);
        P.swingup.beta22 = parameters(2);
        P.swingup.capture_assist_rate2 = parameters(3);
        P.swingup.energy2_target = parameters(4);
        if numel(parameters) >= 5
            P.swingup.beta11 = P.swingup.beta11*parameters(5);
            P.swingup.beta12 = P.swingup.beta12*parameters(5);
            P.swingup.beta13 = P.swingup.beta13*parameters(5);
        end
        if numel(parameters) >= 6
            P.swingup.velocity_limit = parameters(6);
        end
        if numel(parameters) >= 7
            P.swingup.numeric_dt = parameters(7);
        end
        if numel(parameters) >= 8
            P.swingup.energy_switch_smoothing = parameters(8);
        end

        one = dp_simulate_swingup(P);
        captureIndex = find(one.stage == 3,1,'first');
        profile(row) = profileIndex;
        initial_state(row) = stateIndex;
        captured(row) = ~isempty(captureIndex);
        if ~isempty(captureIndex), capture_time(row) = one.t(captureIndex); end
        settled(row) = one.settled;
        failed(row) = one.failed;
        track_failure(row) = one.failed && strcmp(one.reason,'track_limit');
        max_abs_x(row) = one.max_abs_x;
        max_abs_u(row) = one.max_abs_u;
        final_angle1(row) = one.final_wrapped_angles(1);
        final_angle2(row) = one.final_wrapped_angles(2);
        stages(row) = mat2str(one.stages_visited);

        detail = table(profile,initial_state,captured,settled,failed, ...
            track_failure,capture_time,max_abs_x,max_abs_u,final_angle1, ...
            final_angle2,stages);
        writetable(detail(1:row,:),outputFile);
    end
end

summary = struct('candidate',candidateId,'parameters',parameters, ...
    'captures',sum(captured),'settled',sum(settled), ...
    'track_failures',sum(track_failure), ...
    'mean_capture_time',mean(capture_time,'omitnan'), ...
    'worst_max_x',max(max_abs_x),'detail',detail);
fprintf(['Candidate %d long test: captured=%d/%d settled=%d/%d ' ...
    'track_failures=%d mean_capture_time=%.3f worst_x=%.4f\n'], ...
    candidateId,summary.captures,nCase,summary.settled,nCase, ...
    summary.track_failures,summary.mean_capture_time,summary.worst_max_x);
end
