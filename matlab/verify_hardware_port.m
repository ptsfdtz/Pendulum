function report=verify_hardware_port()
%VERIFY_HARDWARE_PORT No drivers and no physical outputs. Regression vectors
% correspond to tests/Phase1Tests.cpp and physical controller invariants.
root=fileparts(mfilename('fullpath')); addpath(fullfile(root,'hardware'));
C=ph_config();
[o,S]=ph_control([0 0],[],C,1); assert(o.voltage==0);
[o,~]=ph_control([0 -1],S,C,1); assert(o.voltage<0);
[o,~]=ph_control([0 -1],[],C,1); assert(o.voltage==0);
[o,~]=ph_control([0 1],[],C,1);
theta=-2*pi/8000; acc=-58.6*theta-10.69*theta/0.01;
assert(abs(o.voltage-0.45*0.005*acc)<1e-12);
[o,S]=ph_control([0 -4000],[],C,1);
assert(o.stage==1 && o.acceleration==-5 && o.voltage==0);
[o,~]=ph_control([0 -4000],S,C,1); assert(o.acceleration==0);
[o,~]=ph_control([1 0],[],C,1); assert(abs(o.state(1)+0.163/8000)<1e-15);
[o,~]=ph_control([0 -4000 0],[],C,2); assert(o.stage==1 && o.acceleration>0);
[o,S]=ph_control([0 0 0],[],C,2); assert(o.stage==2);
[o,S]=ph_control([0 0 0],S,C,2); assert(o.stage==3);
assert(abs(o.voltage-C.stationaryVoltage)<1e-15);
[o,~]=ph_control([0 round(-0.5*8000/(2*pi)) 0],S,C,2); assert(o.stage==1);
[o,~]=ph_control([0 0 round(-0.5*4000/(2*pi))],S,C,2); assert(o.stage==2);
[o,~]=ph_control([-100 100 100],S,C,2);
assert(o.state(1)>0 && o.state(3)<o.state(2) && o.state(2)<0);
% Wrap crossings must not create artificial angular-velocity impulses in double.
[~,S]=ph_control([0 -3999 0],[],C,2);
[o,~]=ph_control([0 -4001 0],S,C,2); assert(max(abs(o.state(5:6)))<1);
rng(9); timing=zeros(2,1);
for order=1:2
    S=[]; tick=tic;
    for k=1:10000
        counts=[round(100*sin(k/70)),round(4000*sin(k/500)),round(2000*sin(k/400))];
        [o,S]=ph_control(counts(1:order+1),S,C,order);
        assert(abs(o.voltage)<=1 && abs(o.velocityReference)<=0.6);
    end
    timing(order)=toc(tick)/10000;
end
failed=false;
try, ph_control([NaN 0],[],C,1); catch, failed=true; end
assert(failed);
failed=false;
try, ph_control([-20000 0],[],C,1); catch, failed=true; end
assert(failed);
report=struct('passed',true,'controller_mean_seconds',timing, ...
    'outputs_written',false,'hardware_swingup_verified',false);
folder=fullfile(root,'output','hardware'); if ~isfolder(folder), mkdir(folder); end
save(fullfile(folder,'port_validation.mat'),'report'); disp(report);
end
