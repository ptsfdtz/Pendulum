function result=start_pendulum_simulink(order,mode,duration)
%START_PENDULUM_SIMULINK Home, zero, then native Simulink swing-up and balance.
% start_pendulum_simulink(1) or (2): physical operation, StopTime remains inf.
% start_pendulum_simulink(2,'readonly',5): inputs + native graph, servo unopened.
% Optional finite duration is a test watchdog, not the saved model StopTime.
if nargin<1, order=1; end
if nargin<2, mode='hardware'; end
if nargin<3, duration=inf; end
validateattributes(order,{'numeric'},{'scalar','integer','>=',1,'<=',2});
validateattributes(duration,{'numeric'},{'scalar','positive','nonnan'});
mode=validatestring(mode,{'hardware','readonly'});
root=fileparts(mfilename('fullpath')); addpath(fullfile(root,'matlab','native_simulink'),fullfile(root,'matlab','hardware'));
model=sprintf('Pendulum_Native_%d',order);
if ~isfile(fullfile(root,'matlab','native_simulink',[model '.slx'])), pn_build(); end
load_system(model); set_param(model,'StopTime','inf');
if strcmp(mode,'hardware')
    % Exercise the actual native controller and input scheduling before motion.
    % Run long enough to warm the complete S-function and native graph in
    % this MATLAB process before a separate hardware session can arm. A
    % timing-only failure may be retried while outputs remain unopened.
    for attempt=1:3
        try
            pre=execute('readonly',5); break;
        catch err
            timing=contains(getReport(err),'pendulum:Timing') || any( ...
                contains(err.message,{'control computation exceeded','control deadline','sample timeout'},'IgnoreCase',true));
            if ~timing || attempt==3, rethrow(err); end
            fprintf('Read-only preflight timing retry %d/3; outputs remain disabled.\n',attempt+1);
        end
    end
    assert(strcmp(pre.status,'completed'),'pendulum:Preflight','Native read-only preflight failed.');
end
result=execute(mode,duration);
    function result=execute(which,seconds)
        ctx=PnHardwareSession(order,which,seconds); pn_context('set',ctx);
        cleanup=onCleanup(@() finish(ctx));
        try, sim(model); catch err, ctx.fault(err); rethrow(err); end
        ctx.finish(); result=ctx.Result; result.folder=ctx.Folder;
    end
    function finish(ctx), ctx.finish(); pn_context('clear'); end
end
