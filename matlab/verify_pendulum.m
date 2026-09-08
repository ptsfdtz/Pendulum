function report = verify_pendulum()
%VERIFY_PENDULUM Check all four scenarios in both offline engines.
root = setup_pendulum();
originalFolder = pwd; restoreFolder = onCleanup(@() cd(originalFolder));
cd(fileparts(root)); % Verify that current directory need not be a model folder.
report = struct([]); index = 0;
for engine = {'numeric','simulink'}
    for order = {'single','double'}
        for scenario = {'swingup','balance'}
            r = run_pendulum(order{1},scenario{1},struct('engine',engine{1}, ...
                'show_ui',false,'show_animation',false));
            index = index+1;
            report(index).order = order{1};
            report(index).scenario = scenario{1};
            report(index).engine = engine{1};
            report(index).passed = r.passed;
            report(index).max_abs_x = r.max_abs_x;
            report(index).max_abs_u = r.max_abs_u;
            report(index).stages = r.stages_visited;
            report(index).output_file = r.output_file;
        end
    end
end
fid = fopen(fullfile(root,'output','verification.json'),'w');
cleanup = onCleanup(@() fclose(fid));
fprintf(fid,'%s',jsonencode(report));
assert(all([report.passed]),'pendulum:ValidationFailed','An offline scenario failed.');
fprintf('ALL_EIGHT_OFFLINE_SCENARIOS_PASSED\n');
end
