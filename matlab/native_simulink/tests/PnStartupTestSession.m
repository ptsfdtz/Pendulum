classdef PnStartupTestSession < PnHardwareSession
    % Exercise the production read/write methods with no vendor dependencies.
    properties
        Trace; StartupDelay=0;
    end
    methods
        function obj=PnStartupTestSession(duration,delay)
            obj@PnHardwareSession(1,'hardware',duration);
            obj.StartupDelay=delay;
            obj.Trace=containers.Map('KeyType','char','ValueType','any');
            obj.Trace('enabled')=false; obj.Trace('ever_enabled')=false; obj.Trace('voltage')=0;
        end
        function start(obj)
            obj.IO=PnStartupMockIO(obj.Trace);
            obj.Home=struct('center',0,'travel',33000);
            obj.Reference=[0 4000]; obj.Warning=0.25;
            obj.Result.home=obj.Home; obj.Result.references=obj.Reference; obj.Result.mock_io=true;
            obj.File=fopen(fullfile(obj.Folder,'samples.csv'),'w'); assert(obj.File>=0);
            fprintf(obj.File,'t,x,theta1,theta2,xdot,omega1,omega2,acceleration,voltage,stage,compute_seconds,lateness_seconds,raw_voltage,vref,soft_reset,fault,count_x,count_a,count_b\n');
            obj.StartClock=tic; obj.Previous=0; obj.Result.status='running';
            pause(obj.StartupDelay);
        end
    end
end
