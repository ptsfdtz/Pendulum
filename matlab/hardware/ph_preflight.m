function report=ph_preflight(io,C,order)
%PH_PREFLIGHT Warm input/JIT paths with outputs unopened, then enforce period.
dt=C.dt(order); S=[]; warm=zeros(100,1);
for k=1:numel(warm)
    tick=tic; s=io.read();
    [~,S]=ph_control(zeros(size(s.counts)),S,C,order);
    warm(k)=toc(tick);
end
% A fresh continuous window must meet the original deadline; no relaxed limit.
n=200; elapsed=zeros(n,1); lateness=zeros(n,1); start=tic;
for k=1:n
    deadline=(k-1)*dt;
    while toc(start)<deadline, end
    lateness(k)=toc(start)-deadline; tick=tic;
    s=io.read(); [~,S]=ph_control(zeros(size(s.counts)),S,C,order);
    elapsed(k)=toc(tick);
end
report=struct('warmup_max_seconds',max(warm),'period_seconds',dt, ...
    'max_compute_seconds',max(elapsed),'median_compute_seconds',median(elapsed), ...
    'max_lateness_seconds',max(lateness),'samples',n, ...
    'overruns',sum(elapsed>=dt | lateness>=dt),'outputs_written',false);
report.passed=report.overruns==0;
fprintf('Preflight: warm-up %.3f ms; measured max %.3f ms; period %.3f ms; overruns %d/%d.\n', ...
    1000*max(warm),1000*max(elapsed),1000*dt,report.overruns,n);
end
