classdef PhIO < handle
    % Vendor .NET adapter. Constructor defaults to input-only operation.
    properties (Access=private)
        C; tasks = {}; counters = {}; limits = {}; writer = []; ao = [];
        lastRaw = []; counts = []; enabled = false;
    end
    methods
        function obj = PhIO(C,order)
            obj.C = C; ph_probe();
            import NationalInstruments.DAQmx.*
            try
                for k=1:order+1
                    t=Task(); obj.tasks{end+1}=t;
                    ch=t.CIChannels.CreateAngularEncoderChannel(C.counters{k},'', ...
                        CIEncoderDecodingType.X4,false,0,CIEncoderZIndexPhase.AHighBHigh, ...
                        int32(2000),0,CIAngularEncoderUnits.Ticks);
                    ch.EncoderAInputDigitalFilterMinimumPulseWidth=C.filterSeconds;
                    ch.EncoderBInputDigitalFilterMinimumPulseWidth=C.filterSeconds;
                    ch.EncoderAInputDigitalFilterEnable=true;
                    ch.EncoderBInputDigitalFilterEnable=true;
                    t.Stream.Timeout=int32(ceil(1000*C.timeout)); t.Start();
                    obj.counters{k}=CounterReader(t.Stream);
                end
                for k=1:2
                    t=Task(); obj.tasks{end+1}=t;
                    t.DIChannels.CreateChannel(C.limitLines{k},'',ChannelLineGrouping.OneChannelForEachLine);
                    t.Stream.Timeout=int32(ceil(1000*C.timeout)); t.Start();
                    obj.limits{k}=DigitalSingleChannelReader(t.Stream);
                end
            catch err
                delete(obj); rethrow(err);
            end
        end
        function s=read(obj)
            raw=zeros(1,numel(obj.counters));
            for k=1:numel(raw), raw(k)=double(obj.counters{k}.ReadSingleSampleUInt32()); end
            if isempty(obj.lastRaw)
                obj.counts=zeros(size(raw));
            else
                delta=mod(raw-obj.lastRaw+2^31,2^32)-2^31;
                assert(all(abs(delta)<=obj.C.jump(1:numel(raw))), ...
                    'pendulum:EncoderJump','Encoder discontinuity.');
                obj.counts=obj.counts+delta;
            end
            obj.lastRaw=raw;
            s.counts=obj.counts; s.limits=false(1,2);
            for k=1:2
                s.limits(k)=logical(obj.limits{k}.ReadSingleSampleSingleLine())==obj.C.limitActiveHigh(k);
            end
            assert(~all(s.limits),'pendulum:Limits','Both limits are active.');
        end
        function openOutputs(obj)
            import NationalInstruments.DAQmx.*
            obj.ao=Automation.BDaq.InstantAoCtrl();
            obj.ao.SelectedDevice=Automation.BDaq.DeviceInformation(obj.C.aoDevice);
            obj.write(0);
            t=Task(); obj.tasks{end+1}=t;
            t.DOChannels.CreateChannel(obj.C.servoLine,'',ChannelLineGrouping.OneChannelForEachLine);
            t.Stream.Timeout=int32(ceil(1000*obj.C.timeout));
            obj.writer=DigitalSingleChannelWriter(t.Stream);
            obj.servo(false);
        end
        function servo(obj,on)
            assert(~isempty(obj.writer),'pendulum:Output','Outputs are not open.');
            if on && obj.enabled, return; end
            if on, obj.write(0); end
            obj.writer.WriteSingleSampleSingleLine(true,logical(on)==obj.C.servoActiveHigh);
            obj.enabled=on;
        end
        function write(obj,v)
            assert(isscalar(v)&&isfinite(v)&&abs(v)<=obj.C.voltageLimit, ...
                'pendulum:Voltage','Invalid output voltage.');
            assert(~isempty(obj.ao),'pendulum:Output','AO is not open.');
            status=obj.ao.Write(int32(obj.C.aoChannel),double(v));
            assert(int32(status)>=0,'pendulum:AO','DAQNavi write failed: %s',char(status.ToString()));
        end
        function stop(obj)
            % Attempt both actions even if the first driver reports failure.
            if ~isempty(obj.ao)
                try, obj.write(0); catch err, warning('pendulum:Stop','AO zero failed: %s',err.message); end
            end
            if ~isempty(obj.writer)
                try, obj.servo(false); catch err, warning('pendulum:Stop','Servo disable failed: %s',err.message); end
            end
        end
        function delete(obj)
            obj.stop();
            for k=numel(obj.tasks):-1:1
                try, obj.tasks{k}.Dispose(); catch, end
            end
            obj.tasks={}; obj.writer=[];
            if ~isempty(obj.ao), obj.ao.Dispose(); obj.ao=[]; end
        end
    end
end


