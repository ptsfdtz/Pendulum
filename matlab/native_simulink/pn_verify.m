function report=pn_verify(rebuild)
% Differential tests: identical measured counts, independent MATLAB recurrence.
% No physical I/O is opened by this test.
here=fileparts(mfilename('fullpath')); addpath(here,fullfile(here,'..','hardware'));
if nargin<1, rebuild=true; end
C=ph_config();
if rebuild, models=pn_build(); else, models={'Pendulum_Native_1','Pendulum_Native_2'}; cellfun(@load_system,models); end
report=struct();
folder=fullfile(here,'..','output','native_simulink');
for order=1:2
    model=models{order}; set_param(model,'SimulationCommand','update');
    blocks=find_system([model '/Native_Control'],'LookUnderMasks','all','Type','Block');
    types=get_param(blocks,'BlockType');
    assert(~any(ismember(types,{'S-Function','MATLABFcn','MATLABSystem'})),'Controller contains nonnative blocks.');
    test=[model '_Test']; if bdIsLoaded(test), close_system(test,0); end
    new_system(test); cleaner=onCleanup(@() close_system(test,0));
    set_param(test,'SolverType','Fixed-step','Solver','FixedStepDiscrete','FixedStep',num2str(C.dt(order),17), ...
        'SaveTime','off','SaveOutput','off','SignalLogging','off','ReturnWorkspaceOutputs','on');
    add_block([model '/Native_Control'],[test '/Controller'],'Position',[260 80 460 200]);
    add_block('simulink/Sources/From Workspace',[test '/Counts'],'VariableName','pn_counts','Interpolate','off', ...
        'OutputAfterFinalValue','Holding final value','Position',[30 60 170 90]);
    add_block('simulink/Sources/Constant',[test '/Limits'],'Value','[0 0]','Position',[30 120 170 150]);
    add_block('simulink/Sources/Constant',[test '/Warning'],'Value',num2str(C.positionWarning(order),17),'Position',[30 180 170 210]);
    for k=1:3
        names={'Counts','Limits','Warning'}; add_line(test,[names{k} '/1'],['Controller/' num2str(k)]);
        add_block('simulink/Sinks/To Workspace',[test '/Log' num2str(k)],'VariableName',['pn_y' num2str(k)],'SaveFormat','Array', ...
            'Position',[540 40+60*k 650 65+60*k]);
        add_line(test,['Controller/' num2str(k)],['Log' num2str(k) '/1']);
    end
    n=12000; k=(0:n-1)';
    counts=[round(100*sin(k/70)),round(4000*sin(k/500)),round(2000*sin(k/400))];
    % Start downward, force exact upright/capture/fall, sign changes, wrap and
    % near travel-warning recovery. Each case runs from an independent reset.
    cases={counts(:,1:order+1)};
    edge=[0 -4000 0;0 -4000 0;0 -3999 0;0 -4001 0;zeros(1000,3);0 -700 0;zeros(1000,3);0 0 -400;zeros(1000,3)];
    cases{end+1}=edge(:,1:order+1);
    travel=[round(-(C.positionWarning(order)+0.02)/C.cartScale(order))*ones(500,1),round(100*sin((1:500)'/12)),zeros(500,1)];
    cases{end+1}=travel(:,1:order+1);
    maximum=0; stages=[]; resets=0; total=0;
    for c=1:numel(cases)
        counts=cases{c}; n=size(counts,1); expected=zeros(n,13); S=[];
        for j=1:n
            [o,S]=ph_control(counts(j,:),S,C,order); v=o.voltage;
            reset=abs(o.state(1))>=C.positionWarning(order) && o.state(1)*v>0;
            if reset
                if order==1, v=0; else, v=-sign(o.state(1))*0.03; end
                S.vref=0; S.integral=0; S.vfree=false; S.ifree=false;
            end
            expected(j,:)=[v 0 o.state o.acceleration o.voltage o.stage o.velocityReference reset];
        end
        pn_counts=[kron((0:n-1)',C.dt(order)),counts];
        input=Simulink.SimulationInput(test); input=input.setVariable('pn_counts',pn_counts);
        input=input.setModelParameter('StopTime',num2str((n-1)*C.dt(order),17));
        out=sim(input); actual=[out.pn_y1 out.pn_y2 out.pn_y3];
        assert(isequal(size(actual),size(expected)));
        delta=max(abs(actual-expected),[],'all'); maximum=max(maximum,delta);
        assert(delta<1e-9,'pendulum:Parity','Order %d case %d max difference %.17g',order,c,delta);
        stages=union(stages,actual(:,11)'); resets=resets+sum(actual(:,13)); total=total+n;
        save(fullfile(folder,sprintf('parity_order%d_case%d.mat',order,c)),'counts','expected','actual','delta');
    end
    assert(all(ismember(1:order+1,stages)),'Missing stage coverage'); assert(resets>0,'Missing soft-reset coverage');
    report.(sprintf('order%d',order))=struct('compiled',true,'native_blocks',numel(blocks), ...
        'samples',total,'max_abs_error',maximum,'stages',stages,'soft_resets',resets,'stop_time',get_param(model,'StopTime'));
    fprintf('NATIVE ORDER %d PASS: %d samples, max error %.3g, stages %s, soft resets %d\n',order,total,maximum,mat2str(stages),resets);
    clear cleaner
end
report.passed=true; save(fullfile(folder,'verification.mat'),'report'); disp(report);
end
