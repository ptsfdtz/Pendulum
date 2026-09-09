function report=verify_cleanup()
root=fileparts(mfilename('fullpath')); addpath(root,fullfile(root,'tests'));
C=ph_config(); folder=fullfile(fileparts(root),'output','hardware');
if ~isfolder(folder), mkdir(folder); end
folder=tempname(folder); mkdir(folder);
record=PhRunRecord(1,C,folder); io=PhMockIO(); record.IO=io;
io.servo(true); io.write(0.1); record.Result.status='running';
try, failWithCleanup(record); catch err, assert(strcmp(err.identifier,'pendulum:Injected')); end
assert(record.Finished && isfile(fullfile(folder,'run.mat')));
saved=load(fullfile(folder,'run.mat'));
assert(strcmp(saved.result.status,'operator_interrupted'));
record.finish(); % Idempotent after disposal.
report=struct('cleanup_passed',true,'outputs_written',false); disp(report);
end
function failWithCleanup(record)
cleanup=onCleanup(@() record.finish());
error('pendulum:Injected','Injected exception for cleanup test.');
end
