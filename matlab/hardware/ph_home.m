function H=ph_home(io,C)
%PH_HOME Fine edge measurement in both directions, then return to session center.
h=C.home; fprintf('Homing: measuring left/right limits...\n');
s=io.read();
if any(s.limits), backoff(find(s.limits,1)); end
seek(1,h.search); left=refine(1); backoff(1);
seek(2,h.search); right=refine(2); backoff(2);
seek(1,h.search); left2=refine(1); backoff(1);
travel=abs(right-left); reverse=abs(right-left2);
assert(min(travel,reverse)>=h.minimumTravel && ...
    abs(travel-reverse)<=max(travel,reverse)*h.disagreement, ...
    'pendulum:Homing','Forward/reverse travel verification failed.');
% Controllers use a fixed negative encoder-to-position sign.
assert(right<left2,'pendulum:Polarity','Cart encoder polarity differs from calibrated controller.');
center=(left2+right)/2; tolerance=max(10,round(travel*0.0005)); start=tic;
while true
    s=sample(); assert(~any(s.limits),'pendulum:Limits','Limit during centering.');
    err=center-s.counts(1);
    if abs(err)<=tolerance
        io.stop(); pause(0.2); s=sample();
        if abs(center-s.counts(1))<=tolerance, break; end
    else
        magnitude=0.03;
        if abs(err)>travel*0.15, magnitude=0.12;
        elseif abs(err)>travel*0.03, magnitude=0.075; end
        command(-sign(err)*magnitude);
    end
    assert(toc(start)<h.centerTimeout,'pendulum:Timeout','Center timeout.'); pause(0.005);
end
H=struct('center',center,'left',left2,'right',right,'travel',(travel+reverse)/2);
fprintf('Center verified: %.1f counts; travel %.1f counts.\n',center,H.travel);
    function s=sample()
        tick=tic; s=io.read();
        assert(toc(tick)<C.timeout,'pendulum:Timeout','Homing input timeout.');
    end
    function command(v)
        io.servo(true); io.write(v);
    end
    function position=seek(side,voltage)
        start=tic; direction=2*side-3;
        while true
            s=sample();
            if any(s.limits)
                io.stop(); assert(s.limits(side),'pendulum:Polarity','Unexpected limit during search.');
                position=s.counts(1); return;
            end
            assert(toc(start)<h.searchTimeout,'pendulum:Timeout','Limit search timeout.');
            command(direction*voltage); pause(0.005);
        end
    end
    function backoff(side)
        start=tic; released=false; release=0;
        while true
            s=sample(); assert(~s.limits(3-side),'pendulum:Limits','Opposite limit during release.');
            if ~s.limits(side) && ~released, release=s.counts(1); released=true; end
            if released && abs(s.counts(1)-release)>=h.escapeCounts, io.stop(); return; end
            assert(toc(start)<h.backoffTimeout,'pendulum:Timeout','Limit release timeout.');
            command(-(2*side-3)*h.escape); pause(0.005);
        end
    end
    function position=refine(side)
        backoff(side); position=seek(side,h.fine);
    end
end

