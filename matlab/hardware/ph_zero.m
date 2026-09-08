function zero=ph_zero(io,C,order)
%PH_ZERO Capture stationary downward references with servo disabled.
io.stop(); fprintf('Waiting for downward pendulum(s) to settle...\n');
n=ceil(C.zeroSeconds/0.01); window=zeros(n,order); k=0; start=tic;
while toc(start)<C.zeroTimeout(order)
    tick=tic; s=io.read();
    assert(toc(tick)<C.timeout && ~any(s.limits),'pendulum:Input','Invalid input during zero capture.');
    k=k+1; window(mod(k-1,n)+1,:)=s.counts(2:order+1);
    if k>=n && all(max(window)-min(window)<=C.zeroSpan(order,1:order))
        zero=median(window,1); fprintf('Stationary downward references captured.\n'); return;
    end
    pause(0.01);
end
error('pendulum:Zero','Pendulum did not settle before timeout.');
end
