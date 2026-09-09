function report = ph_probe()
%PH_PROBE Enumerate only: no AO selection, digital writes or servo enable.
C = ph_config();
loadDriver('NationalInstruments.DAQmx',C.niAssembly);
loadDriver('Automation.BDaq',C.aoAssembly);
system = NationalInstruments.DAQmx.DaqSystem.Local;
names = cell(system.Devices); report.ni = names;
assert(any(strcmp(names,C.niDevice)),'pendulum:Device','Configured NI device missing.');
device = system.LoadDevice(C.niDevice);
report.product = char(device.ProductType);
assert(strcmp(report.product,C.niProduct),'pendulum:Device','Unexpected NI product.');
ao = Automation.BDaq.InstantAoCtrl(); cleanup = onCleanup(@() ao.Dispose());
devices = ao.SupportedDevices; report.ao = cell(1,devices.Count);
for k = 1:devices.Count, report.ao{k} = char(devices.Item(k-1).Description); end
assert(any(strcmp(report.ao,C.aoDevice)),'pendulum:Device','Configured AO device missing.');
report.release = version('-release'); report.outputs_written = false;
disp(report);
end

function loadDriver(name,path)
% Resolve GAC identity first, avoiding a redundant different-path load.
try
    NET.addAssembly(name);
catch
    NET.addAssembly(path);
end
end