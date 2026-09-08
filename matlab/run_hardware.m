function result=run_hardware(order,options)
%RUN_HARDWARE MATLAB-only physical controller. Explicit start enables motion.
% run_hardware(1) or run_hardware(2): home, downward zero, swing-up, balance.
% Ctrl+C unwinds onCleanup: AO zero, servo off, dispose tasks, save log.
if nargin<2, options=struct(); end
validateattributes(order,{'numeric'},{'scalar','integer','>=',1,'<=',2});
root=fileparts(mfilename('fullpath')); addpath(fullfile(root,'hardware'));
C=ph_config();
if isfield(options,'duration'), C.duration=options.duration; end
validateattributes(C.duration,{'numeric'},{'scalar','finite','positive','<=',600});
folder=fullfile(root,'output','hardware'); if ~isfolder(folder), mkdir(folder); end
folder=tempname(folder); mkdir(folder);
record=PhRunRecord(order,C,folder);
cleanup=onCleanup(@() record.finish());
try
    io=PhIO(C,order); record.IO=io;
    record.Result.preflight=ph_preflight(io,C,order);
    assert(record.Result.preflight.passed,'pendulum:Timing', ...
        'Warmed input/control loop exceeded %.1f ms in %d/200 samples. See saved preflight metrics.', ...
        C.dt(order)*1000,record.Result.preflight.overruns);
    io.openOutputs(); H=ph_home(io,C); zero=ph_zero(io,C,order);
    reference=[H.center zero]; reference(2)=reference(2)+C.countsPerRev(1)/2;
    record.Result.home=H; record.Result.references=reference;
    s=io.read(); assert(~any(s.limits),'pendulum:Limits','Active limit at start.');
    assert(abs(s.counts(1)-H.center)<0.05*H.travel,'pendulum:Center','Cart moved from center.');
    assert(all(abs(s.counts(2:end)-zero)<=C.zeroSpan(order,1:order)), ...
        'pendulum:Zero','Pendulum moved before start.');
    S=[]; dt=C.dt(order); start=tic; next=0; previous=0;
    record.Result.status='running'; fprintf('Order %d: swing-up + balance. Ctrl+C stops.\n',order);
    io.write(0); io.servo(true);
    while toc(start)<C.duration
        while toc(start)<next, end
        now=toc(start); tick=tic;
        assert(now-previous<C.timeout,'pendulum:Timing','Control sample timeout.');
        previous=now; s=io.read();
        assert(~any(s.limits),'pendulum:Limits','Physical limit triggered.');
        relative=s.counts-reference;
        position=-relative(1)*C.cartScale(order);
        assert(abs(position)<C.positionStop(order),'pendulum:Track','Software stop reached.');
        [out,S]=ph_control(relative,S,C,order); voltage=out.voltage;
        warningLimit=C.positionWarning(order);
        if order==1, warningLimit=min(warningLimit,0.85*H.travel/2*C.cartScale(1)); end
        if abs(position)>=warningLimit && position*voltage>0
            if order==1, voltage=0; else, voltage=-sign(position)*0.03; end
            S.vref=0; S.integral=0; S.vfree=false; S.ifree=false;
        end
        assert(toc(tick)<dt,'pendulum:Timing','Computation exceeded control period.');
        io.write(voltage);
        record.Count=record.Count+1; record.Log(record.Count,:)=[now out.state out.acceleration voltage out.stage toc(tick) now-next];
        next=next+dt;
        assert(toc(start)-next<dt,'pendulum:Timing','Missed control deadline.');
    end
    record.Result.status='completed'; record.finish();
catch err
    if ~isempty(record.IO), record.IO.stop(); end
    record.Result.status='fault'; record.Result.error=getReport(err,'extended','hyperlinks','off');
    rethrow(err);
end
result=record.Result;
end
