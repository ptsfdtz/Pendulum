function report=pn_verify_recording(folder)
% Independent reference replay of the actual hardware encoder stream.
saved=load(fullfile(folder,'run.mat')); r=saved.result; C=r.config; order=r.order;
data=readmatrix(fullfile(folder,'samples.csv')); expected=zeros(size(data,1),11); S=[];
warningLimit=C.positionWarning(order);
if order==1, warningLimit=min(warningLimit,0.85*r.home.travel/2*C.cartScale(1)); end
for k=1:size(data,1)
    [o,S]=ph_control(data(k,17:17+order),S,C,order); voltage=o.voltage;
    reset=abs(o.state(1))>=warningLimit && o.state(1)*voltage>0;
    if reset
        if order==1, voltage=0; else, voltage=-sign(o.state(1))*0.03; end
        S.vref=0; S.integral=0; S.vfree=false; S.ifree=false;
    end
    expected(k,:)=[o.state o.acceleration voltage o.stage o.velocityReference reset];
end
actual=[data(:,2:10) data(:,14:15)]; delta=max(abs(actual-expected),[],'all');
assert(delta<1e-9,'pendulum:RecordedParity','Hardware stream differs from MATLAB reference.');
report=struct('passed',true,'samples',size(data,1),'max_abs_error',delta,'folder',folder);
save(fullfile(folder,'reference_replay.mat'),'report','expected','actual'); disp(report);
end
