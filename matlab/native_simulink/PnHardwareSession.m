classdef PnHardwareSession < handle
    % Device lifecycle, reference acquisition, pacing and streaming evidence.
    % All swing-up, stabilization, state estimation, PI and travel logic is native.
    properties
        C; Order; Mode; IO=[]; Reference; Warning; Home=[];
        StartClock; ClockBase=0; Tick; Previous=0; LastTime=-inf; Cached; Duration;
        Folder; Count=0; Finished=false; InputPreflight=false; Result;
        Log; Timing; Capacity; LastInterval=0; LastReadSeconds=0;
        Armed=false; PreArmCount=0; ArmAfterSamples=200;
    end
    methods
        function obj=PnHardwareSession(order,mode,duration)
            obj.C=ph_config(); obj.Order=order; obj.Mode=mode; obj.Duration=duration;
            base=fullfile(fileparts(mfilename('fullpath')),'..','output','native_simulink');
            if ~isfolder(base), mkdir(base); end
            obj.Folder=tempname(base); mkdir(obj.Folder);
            obj.Reference=zeros(1,order+1); obj.Warning=obj.C.positionWarning(order);
            obj.Result=struct('order',order,'mode',mode,'status','initializing','config',obj.C,'hardware_swingup_verified',false);
            % Keep all deterministic-loop telemetry in RAM.  Ten minutes is
            % bounded and only about 27 MB at the 200 Hz double-pendulum rate.
            seconds=duration; if ~isfinite(seconds), seconds=600; end
            obj.Capacity=ceil(seconds/obj.C.dt(order))+2;
            obj.Log=zeros(obj.Capacity,19); obj.Timing=zeros(obj.Capacity,5);
        end
        function start(obj)
            assert(~obj.Finished,'pendulum:Session','Create a new session before restarting.');
            try
                obj.IO=PhIO(obj.C,obj.Order);
                % Warm only the existing driver paths; no MATLAB controller call.
                for k=1:100, obj.IO.read(); end
                if strcmp(obj.Mode,'hardware')
                    if obj.InputPreflight
                        obj.Result.preflight=ph_preflight(obj.IO,obj.C,obj.Order);
                        assert(obj.Result.preflight.passed,'pendulum:Timing', ...
                            'Input preflight exceeded the original control period.');
                    end
                    obj.IO.openOutputs();
                    % Warm both vendor output paths at zero volts while the
                    % servo remains disabled.  Their first .NET call can
                    % otherwise block for ~200 ms after motion has begun.
                    for k=1:100
                        obj.IO.read(); obj.IO.write(0); obj.IO.servo(false);
                    end
                    for v=[-obj.C.home.search obj.C.home.search -obj.C.home.fine obj.C.home.fine 0]
                        obj.IO.servo(false); obj.IO.write(v);
                    end
                    obj.IO.write(0); obj.IO.servo(true); obj.IO.write(0); obj.IO.servo(false);
                    s=obj.IO.read();
                    assert(~any(s.limits),'pendulum:Limits','Active limit after zero-command output warm-up.');
                    obj.Home=ph_home(obj.IO,obj.C);
                    zero=ph_zero(obj.IO,obj.C,obj.Order);
                    obj.Reference=[obj.Home.center zero]; obj.Reference(2)=obj.Reference(2)+obj.C.countsPerRev(1)/2;
                    s=obj.IO.read();
                    assert(~any(s.limits),'pendulum:Limits','Active limit at start.');
                    assert(abs(s.counts(1)-obj.Home.center)<0.05*obj.Home.travel,'pendulum:Center','Cart moved from center.');
                    assert(all(abs(s.counts(2:end)-zero)<=obj.C.zeroSpan(obj.Order,1:obj.Order)), 'pendulum:Zero','Pendulum moved before start.');
                    if obj.Order==1, obj.Warning=min(obj.Warning,0.85*obj.Home.travel/2*obj.C.cartScale(1)); end
                end
                obj.Result.home=obj.Home; obj.Result.references=obj.Reference;
                % Simulink may still initialize other blocks after Start returns.
                % Keep the servo disabled until the first valid command is ready.
                if strcmp(obj.Mode,'hardware'), obj.IO.write(0); end
                obj.Result.status='running'; obj.StartClock=tic; obj.Previous=0;
            catch err
                obj.fault(err); rethrow(err);
            end
        end
        function [counts,limits,warningLimit]=read(obj,t)
            try
                if t~=obj.LastTime
                    if isinf(obj.LastTime)
                        % The first Outputs call establishes simulation pacing;
                        % Start-to-Outputs initialization is not a sample interval.
                        obj.StartClock=tic; obj.ClockBase=t; obj.Previous=t;
                    end
                    % Model pacing is owned by Simulink Desktop Real-Time's
                    % Real-Time Synchronization block.  Do not busy-wait in
                    % this MATLAB S-function and compete with its scheduler.
                    now=obj.ClockBase+toc(obj.StartClock); obj.Tick=tic;
                    obj.LastInterval=now-obj.Previous;
                    if (strcmp(obj.Mode,'hardware') && obj.Armed) || ...
                            (strcmp(obj.Mode,'readonly') && obj.Count>=100)
                        assert(obj.LastInterval<obj.C.timeout,'pendulum:Timing', ...
                            'Control sample timeout (interval %.3f ms).',1000*obj.LastInterval);
                    end
                    obj.Previous=now;
                    readTick=tic; s=obj.IO.read(); obj.LastReadSeconds=toc(readTick);
                    obj.Cached=s; obj.LastTime=t;
                else, s=obj.Cached;
                end
                counts=s.counts-obj.Reference; limits=s.limits; warningLimit=obj.Warning;
            catch err, obj.fault(err); rethrow(err); end
        end
        function write(obj,packet,t)
            try
                dt=obj.C.dt(obj.Order); controlSeconds=toc(obj.Tick); late=obj.Previous-t;
                assert(~logical(packet(2)),'pendulum:NativeFault','Native controller limit/nonfinite fault.');
                preArm=strcmp(obj.Mode,'hardware') && ~obj.Armed;
                if preArm
                    if obj.PreArmCount==0, obj.Result.startup_compute_seconds=controlSeconds; end
                    obj.PreArmCount=obj.PreArmCount+1;
                    obj.IO.write(0);
                    if obj.PreArmCount<obj.ArmAfterSamples
                        return;
                    end
                    % The complete Simulink graph and Desktop UI have now
                    % run at real-time rate with the servo disabled. Reject
                    % movement before arming and restart the timing epoch.
                    check=tic; fresh=obj.IO.read();
                    assert(toc(check)<obj.C.timeout,'pendulum:Timing','Startup input timeout.');
                    assert(~any(fresh.limits),'pendulum:Limits','Active limit before first command.');
                    assert(abs(fresh.counts(1)-obj.Home.center)<0.05*obj.Home.travel, ...
                        'pendulum:Center','Cart moved during model initialization.');
                    assert(abs(fresh.counts(1)-obj.Cached.counts(1))<=max(10,round(obj.Home.travel*0.0005)), ...
                        'pendulum:Center','Cart moved during first controller evaluation.');
                    assert(all(abs(fresh.counts(2:end)-obj.Cached.counts(2:end))<= ...
                        obj.C.zeroSpan(obj.Order,1:obj.Order)), ...
                        'pendulum:Zero','Pendulum moved during first controller evaluation.');
                    obj.IO.servo(true);
                    obj.Armed=true;
                    obj.StartClock=tic; obj.ClockBase=t; obj.Previous=t; obj.Tick=tic;
                    late=0; controlSeconds=0;
                end
                warm=strcmp(obj.Mode,'readonly') && obj.Count<100;
                if ~warm
                    controlBudget=min(obj.C.timeout,2*dt);
                    assert(controlSeconds<controlBudget,'pendulum:Timing', ...
                        'Native input/control computation exceeded %.1f ms (%.3f ms).', ...
                        1000*controlBudget,1000*controlSeconds);
                end
                writeSeconds=0;
                if strcmp(obj.Mode,'hardware'), writeTick=tic; obj.IO.write(packet(1)); writeSeconds=toc(writeTick); end
                elapsed=toc(obj.Tick);
                assert(obj.Count<obj.Capacity,'pendulum:Recording', ...
                    'In-memory recording capacity reached; outputs stopped safely.');
                obj.Count=obj.Count+1; raw=obj.Cached.counts-obj.Reference; raw(end+1:3)=0;
                v=packet(3:end); % six states, u, unsupervised voltage, stage, vref, reset
                row=[t v(1:7)' packet(1) v(9) elapsed late v(8) v(10:11)' packet(2) raw];
                obj.Log(obj.Count,:)=row;
                obj.Timing(obj.Count,:)=[obj.LastInterval obj.LastReadSeconds controlSeconds writeSeconds late];
                if ~warm
                    assert(obj.ClockBase+toc(obj.StartClock)-(t+dt)<dt,'pendulum:Timing','Missed native control deadline.');
                end
                if t+dt>=obj.Duration
                    obj.Result.status='completed';
                    set_param(sprintf('Pendulum_Native_%d',obj.Order),'SimulationCommand','stop');
                end
            catch err, obj.fault(err); rethrow(err); end
        end
        function fault(obj,err)
            obj.Result.last_interval_seconds=obj.LastInterval;
            obj.Result.last_read_seconds=obj.LastReadSeconds;
            obj.Result.status='fault'; obj.Result.error=getReport(err,'extended','hyperlinks','off'); obj.finish();
        end
        function finish(obj)
            if obj.Finished, return; end
            obj.Finished=true;
            if ~isempty(obj.IO), obj.IO.stop(); delete(obj.IO); obj.IO=[]; end
            if strcmp(obj.Result.status,'running'), obj.Result.status='stopped'; end
            obj.Result.samples=obj.Count;
            obj.Result.prearm_samples=obj.PreArmCount;
            if obj.Count>0
                data=obj.Log(1:obj.Count,:); timing=obj.Timing(1:obj.Count,:);
                path=fullfile(obj.Folder,'samples.csv'); file=fopen(path,'w'); assert(file>=0);
                fprintf(file,'t,x,theta1,theta2,xdot,omega1,omega2,acceleration,voltage,stage,compute_seconds,lateness_seconds,raw_voltage,vref,soft_reset,fault,count_x,count_a,count_b\n');
                fprintf(file,[repmat('%.17g,',1,18) '%.17g\n'],data'); fclose(file);
                obj.Result.stages_visited=unique(data(:,10))';
                tail=data(:,1)>=data(end,1)-2;
                obj.Result.settled=data(end,1)>=2 && all(abs(data(tail,3:2+obj.Order))<0.05,'all') && ...
                    all(abs(data(tail,5:5+obj.Order))<0.25,'all');
                measured=data;
                if strcmp(obj.Mode,'readonly') && size(data,1)>100, measured=data(101:end,:); end
                obj.Result.warmup_samples=strcmp(obj.Mode,'readonly')*min(100,size(data,1));
                obj.Result.max_compute_seconds=max(measured(:,11)); obj.Result.max_lateness_seconds=max(measured(:,12));
                measuredTiming=timing; if strcmp(obj.Mode,'readonly') && size(timing,1)>100, measuredTiming=timing(101:end,:); end
                names={'interval_seconds','read_seconds','control_seconds','write_seconds','lateness_seconds'};
                for k=1:numel(names)
                    values=sort(measuredTiming(:,k)); n=numel(values);
                    obj.Result.timing.(names{k})=struct('mean',mean(values),'max',values(end), ...
                        'p95',values(max(1,ceil(0.95*n))),'p99',values(max(1,ceil(0.99*n))));
                end
                timing_columns=names; save(fullfile(obj.Folder,'timing.mat'),'timing','timing_columns');
                obj.Result.hardware_swingup_verified=strcmp(obj.Mode,'hardware') && strcmp(obj.Result.status,'completed') && ...
                    obj.Result.settled && all(ismember(1:obj.Order+1,obj.Result.stages_visited));
            end
            result=obj.Result; save(fullfile(obj.Folder,'run.mat'),'result');
            fprintf('Native session %s; %d samples; outputs released. %s\n',result.status,obj.Count,obj.Folder);
        end
    end
end
