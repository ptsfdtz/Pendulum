function result = run_pendulum(order,scenario,options)
%RUN_PENDULUM Four explicit offline scenarios, numerical or Simulink.
% run_pendulum('single','swingup') / ('single','balance')
% run_pendulum('double','swingup') / ('double','balance')
if nargin < 3, options = struct(); end
order = validatestring(order,{'single','double'});
scenario = validatestring(scenario,{'swingup','balance'});
if ~isfield(options,'engine'), options.engine = 'numeric'; end
if ~isfield(options,'show_ui'), options.show_ui = true; end
options.engine = validatestring(options.engine,{'numeric','simulink'});
root = setup_pendulum();
folder = fullfile(root,'output',[order '_pendulum']);
if ~isfolder(folder), mkdir(folder); end
if strcmp(options.engine,'numeric')
    if strcmp(order,'single')
        result = sp_simulate(scenario);
    else
        P = dp_scenario_config(scenario);
        if strcmp(scenario,'swingup')
            result = dp_simulate_swingup(P);
        else
            L = dp_best_lqr();
            initial = [P.x0 P.theta1_0 P.theta2_0 P.xdot0 P.theta1dot_0 P.theta2dot_0];
            [t,X,u,failed,reason] = dp_simulate_numeric(L.K,initial,P.stopTime,P);
            result = struct('t',t,'X',X,'u',u,'failed',failed,'reason',reason);
            result.stage = 3*ones(size(t)); result.stages_visited = 3;
            tail = t >= t(end)-2;
            result.settled = max(abs(X(tail,2:3)),[],'all') < 0.05 && ...
                max(abs(X(tail,4:6)),[],'all') < 0.25;
            result.max_abs_x = max(abs(X(:,1))); result.max_abs_u = max(abs(u));
        end
        result.passed = ~result.failed && result.settled && ...
            result.max_abs_x < P.x_limit && result.max_abs_u <= P.u_max+1e-9 && ...
            abs(result.t(end)-P.stopTime) < 1e-8;
        if strcmp(scenario,'swingup')
            result.passed = result.passed && all(ismember([1 2 3],result.stages_visited));
        end
        result.config = P;
    end
elseif strcmp(order,'double')
    options.scenario = scenario;
    result = run_simulation(options);
    result.config = dp_scenario_config(scenario);
else
    previous = Simulink.fileGenControl('getConfig');
    restore = onCleanup(@() Simulink.fileGenControl('setConfig','config',previous));
    Simulink.fileGenControl('set','CacheFolder',fullfile(folder,'cache'), ...
        'CodeGenFolder',fullfile(folder,'codegen'),'createDir',true);
    model = sp_build_model(scenario,options.show_ui);
    simOut = sim(model,'ReturnWorkspaceOutputs','on');
    states = simOut.get('sp_state_log'); control = simOut.get('sp_control_log');
    stages = simOut.get('sp_stage_log');
    % Controller is held between sample hits; align sampled logs to state time.
    u = interp1(control.time,control.signals.values,states.time,'previous','extrap');
    stage = interp1(stages.time,stages.signals.values,states.time,'previous','extrap');
    result = sp_metrics(states.time,states.signals.values,u,stage,sp_config(),scenario);
    result.model = model;
end
result.order = order; result.scenario = scenario; result.engine = options.engine;
result.matlab_release = version('-release');
% Each run keeps its own parameters, signals, and result for reproduction.
runFolder = tempname(folder); mkdir(runFolder);
result.output_file = fullfile(runFolder,[scenario '_' options.engine '.mat']);
save(result.output_file,'result');
fprintf('%s %s (%s): passed=%d, settled=%d, max|x|=%.6f m\n', ...
    order,scenario,options.engine,result.passed,result.settled,result.max_abs_x);
end
