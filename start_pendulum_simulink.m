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
    pre=execute('readonly',2);
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
