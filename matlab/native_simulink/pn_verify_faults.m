function report=pn_verify_faults()
% Exercise native shutdown outputs without opening devices.
here=fileparts(mfilename('fullpath')); addpath(here,fullfile(here,'..','hardware')); C=ph_config();
report=struct();
for order=1:2
    source=sprintf('Pendulum_Native_%d',order); load_system(source);
    model='Pn_Fault_Test'; if bdIsLoaded(model), close_system(model,0); end
    new_system(model); cleanup=onCleanup(@()close_system(model,0));
    set_param(model,'SolverType','Fixed-step','Solver','FixedStepDiscrete','FixedStep',num2str(C.dt(order)), ...
        'StopTime','0','ReturnWorkspaceOutputs','on');
    add_block([source '/Native_Control'],[model '/Controller']);
    add_block('simulink/Sources/Constant',[model '/Counts'],'Value','testCounts');
    add_block('simulink/Sources/Constant',[model '/Limits'],'Value','testLimits');
    add_block('simulink/Sources/Constant',[model '/Warning'],'Value',num2str(C.positionWarning(order)));
    names={'Counts','Limits','Warning'};
    for k=1:3, add_line(model,[names{k} '/1'],['Controller/' num2str(k)]); end
    for k=1:2
        add_block('simulink/Sinks/To Workspace',[model '/Log' num2str(k)],'VariableName',['y' num2str(k)],'SaveFormat','Array');
        add_line(model,['Controller/' num2str(k)],['Log' num2str(k) '/1']);
    end
    add_block('simulink/Sinks/Terminator',[model '/Telemetry']); add_line(model,'Controller/3','Telemetry/1');
    countCases=zeros(6,order+1); limitCases=[1 0;0 1;1 1;0 0;0 0;0 0];
    countCases(4,1)=ceil(C.positionStop(order)/C.cartScale(order));
    countCases(5,1)=NaN; countCases(6,1)=Inf;
    for j=1:6
        input=Simulink.SimulationInput(model); input=input.setVariable('testCounts',countCases(j,:));
        input=input.setVariable('testLimits',limitCases(j,:)); out=sim(input);
        assert(out.y1==0 && out.y2==1,'pendulum:FaultTest','Fault did not inhibit voltage.');
    end
    report.(sprintf('order%d',order))=struct('passed',true,'cases',6);
    fprintf('FAULT ORDER %d PASS: left/right/both limits, travel, NaN, Inf.\n',order);
    clear cleanup
end
report.passed=true; save(fullfile(here,'..','output','native_simulink','fault_verification.mat'),'report');
end
