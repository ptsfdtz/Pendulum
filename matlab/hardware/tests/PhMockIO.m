classdef PhMockIO < handle
    properties
        x=0; voltage=0; enabled=false; stuck=false;
    end
    methods
        function s=read(obj)
            if obj.enabled && ~obj.stuck, obj.x=obj.x+200*obj.voltage; end
            s=struct('counts',[-obj.x 0 0],'limits',[obj.x<=-1000 obj.x>=1000]);
        end
        function write(obj,v), assert(isfinite(v)&&abs(v)<=1); obj.voltage=v; end
        function servo(obj,on), obj.enabled=on; end
        function stop(obj), obj.voltage=0; obj.enabled=false; end
    end
end
