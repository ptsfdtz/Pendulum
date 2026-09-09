function report=verify_homing()
% Mock-only test: no driver assemblies or physical outputs.
root=fileparts(mfilename('fullpath')); addpath(root,fullfile(root,'tests'));
C=ph_config(); io=PhMockIO(); H=ph_home(io,C);
assert(abs(H.center)<10 && H.travel>=2000 && abs(io.x)<10);
assert(~io.enabled && io.voltage==0);
zero=ph_zero(io,C,2); assert(all(zero==0));
io=PhMockIO(); io.stuck=true; C.home.searchTimeout=0.02;
cleanup=onCleanup(@() io.stop()); failed=false;
try, ph_home(io,C); catch err, failed=strcmp(err.identifier,'pendulum:Timeout'); end
io.stop(); assert(failed && io.voltage==0 && ~io.enabled);
report=struct('homing_passed',true,'zero_passed',true,'timeout_passed',true,'physical_outputs',false);
disp(report);
end
