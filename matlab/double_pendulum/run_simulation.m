function result = run_simulation(options)
%RUN_SIMULATION Build and run the final Simulink validation model.
if nargin < 1, options = struct(); end
if ~isfield(options,'show_ui'), options.show_ui = true; end
if ~isfield(options,'show_animation'), options.show_animation = options.show_ui; end
if ~isfield(options,'stop_time'), options.stop_time = []; end

addpath(fileparts(mfilename('fullpath')));
previousConfig = Simulink.fileGenControl('getConfig');
restoreConfig = onCleanup(@() Simulink.fileGenControl('setConfig','config',previousConfig));
Simulink.fileGenControl('set','CacheFolder',fullfile(dp_output_dir(),'cache'), ...
    'CodeGenFolder',fullfile(dp_output_dir(),'codegen'),'createDir',true);
if ~isfield(options,'scenario'), options.scenario = 'swingup'; end
P = dp_scenario_config(options.scenario);
if P.control_mode == 2 && exist('dp_best_lqr.m','file') ~= 2
    fprintf('No saved best LQR found; starting optimization.\n');
    dp_lqr_optimize();
end
clear dp_animation dp_controller dp_dynamics dp_config dp_best_lqr dp_swingup_control
P = dp_scenario_config(options.scenario);
run_stop_time = P.stopTime;
if ~isempty(options.stop_time), run_stop_time = options.stop_time; end

fprintf('\n========================================\n');
fprintf('Double inverted pendulum simulation\n');
display_track_limit = P.x_limit;
if P.control_mode == 4
    display_track_limit = P.swingup.hard_track_limit;
end
fprintf('Duration %.2f s | track +/-%.2f m | mode %d\n', ...
    run_stop_time,display_track_limit,P.control_mode);
fprintf('Initial theta1 %.3f rad | theta2 %.3f rad\n', ...
    P.theta1_0,P.theta2_0);
fprintf('========================================\n');

build_options = struct('show_ui',options.show_ui, ...
    'show_animation',options.show_animation,'scenario',options.scenario);
MODEL = build_model(build_options);
if ~isempty(options.stop_time), set_param(MODEL,'StopTime',num2str(run_stop_time)); end
set_param(MODEL,'SimulationCommand','update');
if options.show_ui
    open_system([MODEL '/Scope_State']);
    open_system([MODEL '/Scope_Velocity']);
    open_system([MODEL '/Scope_Control']);
    open_system([MODEL '/Scope_Stage']);
end
fprintf('Running Simulink validation...\n');
simOut = sim(MODEL,'ReturnWorkspaceOutputs','on');

state_log = simOut.get('dp_state_log');
velocity_log = simOut.get('dp_velocity_log');
control_log = simOut.get('dp_control_log');
stage_log = simOut.get('dp_stage_log');
state_values = state_log.signals.values;
velocity_values = velocity_log.signals.values;
control_values = control_log.signals.values;
stage_values = stage_log.signals.values;
% Continuous states and sampled commands need not have identical time grids.
all_values = [state_values(:); velocity_values(:); control_values(:)];
result = struct();
result.model = MODEL;
result.simulation_output = simOut;
result.finite = all(isfinite(all_values(:)));
result.max_abs_x = max(abs(state_values(:,1)));
result.max_abs_theta1 = max(abs(state_values(:,2)));
result.max_abs_theta2 = max(abs(state_values(:,3)));
result.max_abs_u = max(abs(control_values(:)));
result.u_within_limit = result.max_abs_u <= P.u_max + 1e-9;
result.x_changed = (max(state_values(:,1))-min(state_values(:,1))) > 1e-8;
result.final_state = [state_values(end,:) velocity_values(end,:)];
wrapped_angles = atan2(sin(state_values(:,2:3)),cos(state_values(:,2:3)));
result.final_wrapped_angles = wrapped_angles(end,:);
result.stages_visited = unique(stage_values(:))';
capture_index = find(stage_values(:) == 3,1,'first');
if isempty(capture_index)
    result.capture_time = NaN;
else
    result.capture_time = stage_log.time(capture_index);
end
tail_time = state_log.time >= max(state_log.time(end)-2,0);
velocity_tail = velocity_log.time >= max(velocity_log.time(end)-2,0);
result.settled = all(max(abs(wrapped_angles(tail_time,:)),[],1) < 0.05) && ...
    max(abs(velocity_values(velocity_tail,:)),[],'all') < 0.25;
result.track_ok = result.max_abs_x < P.swingup.hard_track_limit;
if P.control_mode == 4
    result.passed = result.finite && result.u_within_limit && ...
        result.x_changed && result.track_ok && result.settled && ...
        all(ismember([1 2 3],result.stages_visited));
else
    result.passed = result.finite && result.u_within_limit && result.x_changed && ...
        result.track_ok && result.settled;
end
fprintf(['Finished: finite=%d, max|x|=%.6f, max|u|=%.6f, ' ...
    'settled=%d, stages='],result.finite,result.max_abs_x, ...
    result.max_abs_u,result.settled);
fprintf('%g ',result.stages_visited);
fprintf(', passed=%d\n',result.passed);
end
