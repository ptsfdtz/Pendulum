function report=verify_output_api()
%VERIFY_OUTPUT_API Writes ONLY vendor demo/simulated devices.
C=ph_config(); ph_probe();
system=NationalInstruments.DAQmx.DaqSystem.Local;
device=system.LoadDevice('SimDev1');
assert(logical(device.IsSimulated),'pendulum:Simulation','SimDev1 is not simulated.');
C.servoLine='SimDev1/port0/line3'; C.aoDevice='DemoDevice,BID#0';
io=PhIO(C,1); cleanup=onCleanup(@() delete(io));
io.openOutputs(); io.servo(true); io.write(0.1); io.write(-0.1); io.stop();
failed=false; try, io.write(NaN); catch, failed=true; end
assert(failed);
failed=false; try, io.write(1.01); catch, failed=true; end
assert(failed);
report=struct('output_api_passed',true,'outputs_target','vendor demo + NI simulated device', ...
    'physical_outputs_written',false); disp(report);
end
