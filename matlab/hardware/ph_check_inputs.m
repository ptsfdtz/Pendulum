function report=ph_check_inputs(order,seconds)
%PH_CHECK_INPUTS Read-only latency and input validation. Does not open outputs.
if nargin<1, order=2; end
if nargin<2, seconds=2; end
validateattributes(order,{'numeric'},{'scalar','integer','>=',1,'<=',2});
C=ph_config(); io=PhIO(C,order); cleanup=onCleanup(@() delete(io));
s=io.read(); dt=zeros(ceil(seconds/C.dt(order)),1); start=tic;
for k=1:numel(dt)
    tick=tic; s=io.read(); dt(k)=toc(tick);
    pause(max(0,C.dt(order)-dt(k)));
end
report=struct('order',order,'sample_count',numel(dt),'elapsed',toc(start), ...
    'max_read_seconds',max(dt),'median_read_seconds',median(dt), ...
    'counts',s.counts,'limits',s.limits,'outputs_written',false);
disp(report);
end
