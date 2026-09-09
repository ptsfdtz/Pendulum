classdef PnStartupMockIO < handle
    properties
        Trace; counts=[13 0]; limits=[false false];
    end
    methods
        function obj=PnStartupMockIO(trace), obj.Trace=trace; end
        function s=read(obj), s=struct('counts',obj.counts,'limits',obj.limits); end
        function servo(obj,on)
            obj.Trace('enabled')=on;
            if on, obj.Trace('ever_enabled')=true; end
        end
        function write(obj,v)
            if v~=0 && ~obj.Trace('enabled'), error('test:NotEnabled','Command applied before servo enable.'); end
            obj.Trace('voltage')=v;
        end
        function stop(obj), obj.Trace('voltage')=0; obj.Trace('enabled')=false; end
    end
end
