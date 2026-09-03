% Read-only inspection harness for the archived LQR_lp2 Simulink reference.
% It does not compile, simulate, create hardware tasks, or write the model.

repoRoot = fileparts(fileparts(mfilename('fullpath')));
referenceDir = fullfile(repoRoot, 'reference', 'LQR_lp2');
modelPath = fullfile(referenceDir, 'LQR_lp2.slx');

fprintf('MATLAB_RELEASE=%s\n', version('-release'));
fprintf('MODEL_PATH=%s\n', modelPath);
fprintf('MODEL_SHA256=');
system(sprintf('certutil -hashfile "%s" SHA256 | findstr /v hash', modelPath));

oldFolder = cd(referenceDir);
restoreFolder = onCleanup(@() cd(oldFolder));
run('P_2.m');
fprintf('LQR_K=');
fprintf('%.17g ', K);
fprintf('\n');
fprintf('A=');
fprintf('%.17g ', A(:));
fprintf('\nB=');
fprintf('%.17g ', B(:));
fprintf('\n');

modelName = 'LQR_lp2';
load_system(modelPath);
closeModel = onCleanup(@() close_system(modelName, 0));

fprintf('MODEL_VERSION=%s\n', get_param(modelName, 'ModelVersion'));
fprintf('SOLVER=%s\n', get_param(modelName, 'Solver'));
fprintf('FIXED_STEP=%s\n', get_param(modelName, 'FixedStep'));
fprintf('START_TIME=%s\n', get_param(modelName, 'StartTime'));
fprintf('STOP_TIME=%s\n', get_param(modelName, 'StopTime'));

maskedBlocks = find_system(modelName, 'LookUnderMasks', 'all', ...
    'FollowLinks', 'on', 'Mask', 'on');
for index = 1:numel(maskedBlocks)
    block = maskedBlocks{index};
    fprintf('MASK|%s', strrep(block, sprintf('\n'), '\\n'));
    variables = get_param(block, 'MaskWSVariables');
    for variableIndex = 1:numel(variables)
        value = variables(variableIndex).Value;
        if isnumeric(value) || islogical(value)
            valueText = mat2str(value, 17);
        elseif ischar(value)
            valueText = value;
        else
            valueText = class(value);
        end
        fprintf('|%s=%s', variables(variableIndex).Name, ...
            strrep(strrep(valueText, sprintf('\n'), '\\n'), '|', '\\x7c'));
    end
    fprintf('\n');
end

topLevelBlocks = find_system(modelName, 'SearchDepth', 1, 'Type', 'Block');
for index = 1:numel(topLevelBlocks)
    destination = topLevelBlocks{index};
    ports = get_param(destination, 'PortHandles');
    for portIndex = 1:numel(ports.Inport)
        line = get_param(ports.Inport(portIndex), 'Line');
        if line ~= -1
            sourcePort = get_param(line, 'SrcPortHandle');
            sourceBlock = get_param(sourcePort, 'Parent');
            fprintf('TOP_INPUT|%s|%d|%s\n', ...
                strrep(destination, sprintf('\n'), '\\n'), portIndex, ...
                strrep(sourceBlock, sprintf('\n'), '\\n'));
        end
    end
end

blocks = find_system(modelName, 'LookUnderMasks', 'all', ...
    'FollowLinks', 'on', 'Type', 'Block');
for index = 1:numel(blocks)
    block = blocks{index};
    blockType = get_param(block, 'BlockType');
    fprintf('BLOCK|%s|%s', strrep(block, sprintf('\n'), '\\n'), blockType);
    names = {'SourceBlock', 'Gain', 'Value', 'Inputs', 'Outputs', ...
        'Operator', 'UpperLimit', 'LowerLimit', 'SampleTime', ...
        'InitialCondition', 'FunctionName'};
    for parameterIndex = 1:numel(names)
        parameterName = names{parameterIndex};
        try
            value = get_param(block, parameterName);
            if ischar(value) && ~isempty(value)
                fprintf('|%s=%s', parameterName, ...
                    strrep(strrep(value, sprintf('\n'), '\\n'), '|', '\\x7c'));
            end
        catch
        end
    end
    fprintf('\n');

    connectivity = get_param(block, 'PortConnectivity');
    for portIndex = 1:numel(connectivity)
        destinationBlocks = connectivity(portIndex).DstBlock;
        destinationPorts = connectivity(portIndex).DstPort;
        if ~isempty(destinationBlocks)
            for destinationIndex = 1:numel(destinationBlocks)
                destinationPath = getfullname(destinationBlocks(destinationIndex));
                fprintf('EDGE|%s|%d|%s|%d\n', ...
                    strrep(block, sprintf('\n'), '\\n'), portIndex - 1, ...
                    strrep(destinationPath, sprintf('\n'), '\\n'), ...
                    destinationPorts(destinationIndex));
            end
        end
    end
end

machine = sfroot;
charts = machine.find('-isa', 'Stateflow.Chart');
for index = 1:numel(charts)
    chart = charts(index);
    if startsWith(chart.Path, modelName)
        fprintf('CHART|%s\n', strrep(chart.Path, sprintf('\n'), '\\n'));
        transitions = chart.find('-isa', 'Stateflow.Transition');
        for transitionIndex = 1:numel(transitions)
            fprintf('TRANSITION|%s\n', strrep( ...
                transitions(transitionIndex).LabelString, sprintf('\n'), '\\n'));
        end
    end
end
