function report=pn_verify_startup()
% Reproduce Start-to-Outputs and first-evaluation delays without physical I/O.
here=fileparts(mfilename('fullpath')); addpath(here,fullfile(here,'tests'),fullfile(here,'..','hardware'));
saved=load(fullfile(here,'..','output','native_simulink','tp2afa237e_9111_4e1f_b2d1_a645c2d3cde7','run.mat'));
assert(saved.result.samples==0 && contains(saved.result.error,'Control sample timeout'));
% Exercise the actual S-function + native graph after a delayed Start callback.
model='Pendulum_Native_1'; load_system(model);
sync=[model '/Real-Time Synchronization']; set_param(sync,'Commented','on');
ctx=PnStartupTestSession(0.095,0.12);
trace=ctx.Trace; pn_context('set',ctx); guard=onCleanup(@()closeSession(ctx));
sim(model); ctx.finish();
assert(ctx.Count==10 && strcmp(ctx.Result.status,'completed'));
assert(trace('ever_enabled') && ~trace('enabled') && trace('voltage')==0);
assert(strcmp(get_param(model,'StopTime'),'inf')); clear guard;
% Delay both first callbacks beyond the unchanged 50 ms runtime timeout.
ctx=PnStartupTestSession(inf,0.12); trace=ctx.Trace; guard=onCleanup(@()closeSession(ctx)); ctx.start();
[counts,~,~]=ctx.read(0); assert(~trace('enabled'));
[o,~]=ph_control(counts,[],ctx.C,1); packet=[o.voltage;0;o.state';o.acceleration;o.voltage;o.stage;o.velocityReference;0];
pause(0.12); ctx.write(packet,0);
assert(trace('enabled') && ctx.Count==1 && ctx.Result.startup_compute_seconds>0.05);
% Once enabled, the same 50 ms gap must still stop outputs.
pause(0.08); caught=false;
try, ctx.read(0.01); catch err, caught=strcmp(err.identifier,'pendulum:Timing'); end
assert(caught && ctx.Finished && ~trace('enabled') && trace('voltage')==0); clear guard;
% Movement or an active limit during the first calculation must inhibit enable.
for which=1:3
    ctx=PnStartupTestSession(inf,0); trace=ctx.Trace; guard=onCleanup(@()closeSession(ctx)); ctx.start(); ctx.read(0);
    if which==1, ctx.IO.counts(2)=100;
    elseif which==2, ctx.IO.limits=[true false];
    else, ctx.IO.counts(1)=100; end
    caught=false;
    try, ctx.write(packet,0); catch err, caught=ismember(err.identifier,{'pendulum:Zero','pendulum:Limits','pendulum:Center'}); end
    assert(caught && ~trace('ever_enabled') && trace('voltage')==0); clear guard;
end
% The 10 ms computation limit remains active after startup.
ctx=PnStartupTestSession(inf,0); trace=ctx.Trace; guard=onCleanup(@()closeSession(ctx)); ctx.start(); ctx.read(0); ctx.write(packet,0);
ctx.Tick=tic; pause(0.03); caught=false;
try, ctx.write(packet,0.01); catch err, caught=strcmp(err.identifier,'pendulum:Timing'); end
assert(caught && ~trace('enabled') && trace('voltage')==0); clear guard;
for order=1:2
    model=sprintf('Pendulum_Native_%d',order); load_system(model); set_param(model,'SimulationCommand','update');
end
set_param('Pendulum_Native_1/Real-Time Synchronization','Commented','off');
report=struct('passed',true,'delayed_start_and_first_evaluation',true,'fresh_input_checks',true, ...
    'runtime_timeout_preserved',true,'compute_deadline_preserved',true,'actual_sfun_simulated',true,'physical_io_opened',false);
save(fullfile(here,'..','output','native_simulink','startup_verification.mat'),'report'); disp(report);
bdclose('all');
end
function closeSession(ctx), ctx.finish(); pn_context('clear'); end
