function verify_preflight()
% A persistently slow input must still fail after warm-up.
C=ph_config(); io=struct('read',@slowRead);
r=ph_preflight(io,C,2);
assert(~r.passed && r.overruns>0 && r.max_compute_seconds>=C.dt(2));
fprintf('PREFLIGHT_PERSISTENT_OVERRUN_REJECTED\n');
end
function s=slowRead()
pause(0.006);
s=struct('counts',[0 0 0]);
end
