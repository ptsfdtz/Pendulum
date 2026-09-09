function report=ph_check_timing(order,seconds)
%PH_CHECK_TIMING Exercise the input/controller loop with NO output tasks.
if nargin<1, order=2; end
if nargin<2, seconds=5; end
C=ph_config(); io=PhIO(C,order); cleanup=onCleanup(@() delete(io));
S=[]; for k=1:100, s=io.read(); [~,S]=ph_control(s.counts,S,C,order); end
n=ceil(seconds/C.dt(order)); data=zeros(n,2); start=tic;
for k=1:n
    deadline=(k-1)*C.dt(order);
    while toc(start)<deadline, end
    data(k,1)=toc(start)-deadline; tick=tic;
    s=io.read(); [~,S]=ph_control(s.counts,S,C,order); data(k,2)=toc(tick);
end
report=struct('order',order,'samples',n,'max_lateness',max(data(:,1)), ...
    'max_compute_seconds',max(data(:,2)),'median_compute_seconds',median(data(:,2)), ...
    'missed',sum(data(:,1)>=C.dt(order)),'outputs_written',false);
disp(report);
end
