classdef PnHardwareSession < handle
    % Device lifecycle, reference acquisition, pacing and streaming evidence.
    % All swing-up, stabilization, state estimation, PI and travel logic is native.
    properties
        C; Order; Mode; IO=[]; Reference; Warning; Home=[];
        StartClock; ClockBase=0; Tick; Previous=0; LastTime=-inf; Cached; Duration;
        Folder; File=-1; Count=0; Finished=false; InputPreflight=false; Result;
    end
    methods
        function obj=PnHardwareSession(order,mode,duration)
            obj.C=ph_config(); obj.Order=order; obj.Mode=mode; obj.Duration=duration;
            base=fullfile(fileparts(mfilename('fullpath')),'..','output','native_simulink');
            if ~isfolder(base), mkdir(base); end
            obj.Folder=tempname(base); mkdir(obj.Folder);
            obj.Reference=zeros(1,order+1); obj.Warning=obj.C.positionWarning(order);
            obj.Result=struct('order',order,'mode',mode,'status','initializing','config',obj.C,'hardware_swingup_verified',false);
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
                    obj.IO.openOutputs(); obj.Home=ph_home(obj.IO,obj.C);
                    zero=ph_zero(obj.IO,obj.C,obj.Order);
                    obj.Reference=[obj.Home.center zero]; obj.Reference(2)=obj.Reference(2)+obj.C.countsPerRev(1)/2;
                    s=obj.IO.read();
                    assert(~any(s.limits),'pendulum:Limits','Active limit at start.');
                    assert(abs(s.counts(1)-obj.Home.center)<0.05*obj.Home.travel,'pendulum:Center','Cart moved from center.');
                    assert(all(abs(s.counts(2:end)-zero)<=obj.C.zeroSpan(obj.Order,1:obj.Order)), 'pendulum:Zero','Pendulum moved before start.');
                    if obj.Order==1, obj.Warning=min(obj.Warning,0.85*obj.Home.travel/2*obj.C.cartScale(1)); end
                end
                obj.Result.home=obj.Home; obj.Result.references=obj.Reference;
                obj.File=fopen(fullfile(obj.Folder,'samples.csv'),'w'); assert(obj.File>=0);
                fprintf(obj.File,'t,x,theta1,theta2,xdot,omega1,omega2,acceleration,voltage,stage,compute_seconds,lateness_seconds,raw_voltage,vref,soft_reset,fault,count_x,count_a,count_b\n');
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
                    while obj.ClockBase+toc(obj.StartClock)<t, end
                    now=obj.ClockBase+toc(obj.StartClock); obj.Tick=tic;
                    if ~strcmp(obj.Mode,'readonly') || obj.Count>=100
                        assert(now-obj.Previous<obj.C.timeout,'pendulum:Timing','Control sample timeout.');
                    end
                    obj.Previous=now;
                    s=obj.IO.read(); obj.Cached=s; obj.LastTime=t;
                else, s=obj.Cached;
                end
                counts=s.counts-obj.Reference; limits=s.limits; warningLimit=obj.Warning;
            catch err, obj.fault(err); rethrow(err); end
        end
        function write(obj,packet,t)
            try
                dt=obj.C.dt(obj.Order); elapsed=toc(obj.Tick); late=obj.Previous-t;
                assert(~logical(packet(2)),'pendulum:NativeFault','Native controller limit/nonfinite fault.');
                firstHardware=strcmp(obj.Mode,'hardware') && obj.Count==0;
                if firstHardware
                    obj.Result.startup_compute_seconds=elapsed;
                    % First evaluation/JIT occurred with the servo disabled.
                    % Reject movement during initialization before enabling it.
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
                    obj.StartClock=tic; obj.ClockBase=t; obj.Previous=t; obj.Tick=tic;
                    late=0; elapsed=0;
                end
                warm=strcmp(obj.Mode,'readonly') && obj.Count<100;
                if ~warm
                    assert(elapsed<dt,'pendulum:Timing','Native input/control computation exceeded %.1f ms (%.3f ms).',1000*dt,1000*elapsed);
                end
                if strcmp(obj.Mode,'hardware'), obj.IO.write(packet(1)); end
                elapsed=toc(obj.Tick);
                obj.Count=obj.Count+1; raw=obj.Cached.counts-obj.Reference; raw(end+1:3)=0;
                v=packet(3:end); % six states, u, unsupervised voltage, stage, vref, reset
                row=[t v(1:7)' packet(1) v(9) elapsed late v(8) v(10:11)' packet(2) raw];
                fprintf(obj.File,[repmat('%.17g,',1,numel(row)-1) '%.17g\n'],row);
                if ~warm
                    assert(obj.ClockBase+toc(obj.StartClock)-(t+dt)<dt,'pendulum:Timing','Missed native control deadline.');
                else
                    % Driver/adapter JIT warm-up is permitted only with outputs
                    % unopened. The measured window retains the original period.
                    obj.ClockBase=t+dt; obj.StartClock=tic; obj.Previous=t;
                end
                if t+dt>=obj.Duration
                    obj.Result.status='completed';
                    set_param(sprintf('Pendulum_Native_%d',obj.Order),'SimulationCommand','stop');
                end
            catch err, obj.fault(err); rethrow(err); end
        end
        function fault(obj,err)
            obj.Result.status='fault'; obj.Result.error=getReport(err,'extended','hyperlinks','off'); obj.finish();
        end
        function finish(obj)
            if obj.Finished, return; end
            obj.Finished=true;
            if ~isempty(obj.IO), obj.IO.stop(); delete(obj.IO); obj.IO=[]; end
            if obj.File>=0, fclose(obj.File); obj.File=-1; end
            if strcmp(obj.Result.status,'running'), obj.Result.status='stopped'; end
            obj.Result.samples=obj.Count;
            if obj.Count>0
                data=readmatrix(fullfile(obj.Folder,'samples.csv'));
                obj.Result.stages_visited=unique(data(:,10))';
                tail=data(:,1)>=data(end,1)-2;
                obj.Result.settled=data(end,1)>=2 && all(abs(data(tail,3:2+obj.Order))<0.05,'all') && ...
                    all(abs(data(tail,5:5+obj.Order))<0.25,'all');
                measured=data;
                if strcmp(obj.Mode,'readonly') && size(data,1)>100, measured=data(101:end,:); end
                obj.Result.warmup_samples=strcmp(obj.Mode,'readonly')*min(100,size(data,1));
                obj.Result.max_compute_seconds=max(measured(:,11)); obj.Result.max_lateness_seconds=max(measured(:,12));
                obj.Result.hardware_swingup_verified=strcmp(obj.Mode,'hardware') && strcmp(obj.Result.status,'completed') && ...
                    obj.Result.settled && all(ismember(1:obj.Order+1,obj.Result.stages_visited));
            end
            result=obj.Result; save(fullfile(obj.Folder,'run.mat'),'result');
            fprintf('Native session %s; %d samples; outputs released. %s\n',result.status,obj.Count,obj.Folder);
        end
    end
end
