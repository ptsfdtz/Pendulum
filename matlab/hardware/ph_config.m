function C = ph_config()
%PH_CONFIG Self-contained hardware configuration (SI units, raw X4 counts).
C.niDevice = 'Dev1'; C.niProduct = 'PCI-6602';
C.aoDevice = 'PCI-1723,BID#15'; C.aoChannel = 0;
C.counters = {'Dev1/ctr0','Dev1/ctr1','Dev1/ctr2'};
C.limitLines = {'Dev1/port0/line0','Dev1/port0/line2'};
C.servoLine = 'Dev1/port0/line3'; C.servoActiveHigh = true;
C.limitActiveHigh = [true true]; C.filterSeconds = 10e-6;
C.niAssembly = fullfile(getenv('ProgramFiles(x86)'), 'National Instruments', ...
    'MeasurementStudioVS2012','DotNET','Assemblies (64-bit)','Current','NationalInstruments.DAQmx.dll');
C.aoAssembly = 'C:\Advantech\DAQNavi\Automation.BDaq\1.0.0.0\Automation.BDaq.dll';
C.dt = [0.01 0.005]; C.timeout = 0.05;
C.jump = [1000 2000 1000]; C.voltageLimit = 1;
C.stationaryVoltage = -0.00152587890625;
C.countsPerRev = [8000 4000]; C.cartScale = [0.163/8000 0.734/33259];
C.K = [20.755626070279462 136.68455751706179 -254.85837173801795 ...
    24.440647335478328 3.3044801353394511 -40.750449384516202];
C.positionStop = [0.30 0.35]; C.positionWarning = [0.25 0.30];
C.home.search = 0.20; C.home.fine = 0.02; C.home.escape = 0.03;
C.home.escapeCounts = 200; C.home.searchTimeout = 60; C.home.backoffTimeout = 5;
C.home.centerTimeout = 30; C.home.minimumTravel = 1000; C.home.disagreement = 0.02;
C.zeroSeconds = 1.5; C.zeroTimeout = [20 30]; C.zeroSpan = [8 0;80 40];
C.duration = 60; % Bounded run; operator can stop earlier with Ctrl+C.
end
