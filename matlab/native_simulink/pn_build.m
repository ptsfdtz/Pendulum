function models=pn_build()
%PN_BUILD Replace the two models with the five-input, acceleration-only interface.
% Hardware owns sensor conversion, automatic servo, PI and independent safety.
here=fileparts(mfilename('fullpath')); addpath(here,fullfile(here,'..','hardware'));
C=ph_config(); models=cell(1,2);
load_system('simulink');
for order=1:2
    model=sprintf('Pendulum_Native_%d',order); models{order}=model;
    if bdIsLoaded(model), close_system(model,0); end
    new_system(model);
    set_param(model,'SolverType','Fixed-step','Solver','FixedStepDiscrete', ...
        'FixedStep',num2str(C.dt(order),17),'StopTime','inf', ...
        'SaveTime','off','SaveOutput','off','SignalLogging','off', ...
        'ReturnWorkspaceOutputs','on','AlgebraicLoopMsg','error');
    paths=sprintf(['addpath(fileparts(get_param(''%s'',''FileName'')),' ...
        'fullfile(fileparts(get_param(''%s'',''FileName'')),''..'',''hardware''));'],model,model);
    set_param(model,'PostLoadFcn',paths,'InitFcn',paths);
    pn_build_hardware(model,order,C);
    % Use the Desktop Real-Time kernel to pace Normal-mode execution.  The
    % custom vendor I/O remains in MATLAB, but no longer relies on a Windows
    % busy-wait loop for the 100/200 Hz sample clock.
    load_system('sldrtlib'); sync=find_system('sldrtlib','Type','Block','Name','Real-Time Synchronization');
    add_block(sync{1},[model '/Real-Time Synchronization'], ...
        'SampleTime',num2str(C.dt(order),17),'MaxMissedTicks','1000000', ...
        'ShowMissedTicks','off','YieldWhenWaiting','off','Priority','-100', ...
        'Position',[120 465 350 515]);
    p=[model '/Control']; add_block('built-in/Subsystem',p,'Position',[550 140 790 410]);
    g=PnGraph(p); angle1=g.in('Angle1_deg',1,1); angle2=g.in('Angle2_deg',2,1);
    x=g.in('Position_m',3,1); limits=g.in('Limits_left_right',4,2); servo=g.in('Servo',5,1);
    g.section('Sensor_and_memory');
    z=g.c(0); one=g.c(1);
    % Delay states exactly match the initial ph_control struct.
    q0=g.delay('Previous_position',0); a0=g.delay('Previous_angle1',0);
    a=g.gain(angle1,pi/180);
    if order==1
        g.block('Sinks/Terminator','Unused_second_angle',{angle2});
        enc=g.gain(a,-1); enc0=g.delay('Previous_encoder_rad',0);
        velocity=g.div(g.sub(x,q0),g.c(C.dt(1))); omega=g.div(g.sub(a,a0),g.c(C.dt(1)));
        b=z; omega2=z;
        g.section('Swingup_and_LQR');
        lqr=g.sat(g.add(g.gain(x,10),g.gain(velocity,12.23),g.gain(a,-58.6),g.gain(omega,-10.69)),-10,10);
        delta=g.wrap(g.sub(enc,enc0));
        energy=g.add(g.gain(g.sub(one,g.trig(enc,'cos')),0.134*9.8*0.223),g.gain(g.mul(delta,delta),0.0089/0.0002));
        domain=g.sub(one,g.gain(g.abs(x),0.8/0.25));
        % Clamp the log input only on the already-faulted path, avoiding complex
        % signal propagation before the native safety fault reaches the adapter.
        swing=g.sat(g.add(g.gain(g.mul(g.log(g.minmax('max',domain,g.c(realmin))),x),6), ...
            g.gain(g.sign(g.mul(g.trig(enc,'cos'),delta,g.sub(g.c(2*0.134*9.8*0.223),energy))),5)),-10,10);
        swingMode=g.cmp(g.abs(a),'>=',g.c(pi/6)); stage=g.choose(swingMode,one,g.c(2));
        u=g.choose(swingMode,swing,lqr);
        g.bind(enc0,enc);
    else
        b=g.gain(angle2,pi/180);
        b0=g.delay('Previous_angle2',0); initialized=g.delay('Initialized',0);
        vel0=g.delay('Filtered_cart_rate',0); om0=g.delay('Filtered_rate1',0); om20=g.delay('Filtered_rate2',0);
        alpha=exp(-2*pi*20*C.dt(2));
        velocity=rate(x,q0,vel0,false); omega=rate(a,a0,om0,true); omega2=rate(b,b0,om20,true);
        g.section('Stage_supervisor');
        st0=g.delay('Previous_stage',1);
        is1=g.cmp(st0,'==',one); is2=g.cmp(st0,'==',g.c(2)); is3=g.cmp(st0,'==',g.c(3));
        enter3=g.logic('AND',g.cmp(g.abs(a),'<=',g.c(0.12)),g.cmp(g.abs(b),'<=',g.c(0.26)), ...
            g.cmp(g.abs(velocity),'<=',g.c(0.12)),g.cmp(g.mul(x,velocity),'<=',z), ...
            g.cmp(g.mul(x,a),'<=',z),g.cmp(g.mul(x,b),'>=',z), ...
            g.cmp(g.abs(omega),'<=',g.c(0.60)),g.cmp(g.abs(omega2),'<=',g.c(0.80)));
        stage=g.choose(g.logic('AND',is2,enter3),g.c(3),st0);
        stage=g.choose(g.logic('AND',is2,g.cmp(g.abs(a),'>',g.c(0.70))),one,stage);
        stage=g.choose(g.logic('AND',is1,g.cmp(g.abs(a),'<=',g.c(0.37)),g.cmp(g.abs(omega),'<=',g.c(2))),g.c(2),stage);
        stage=g.choose(g.logic('AND',is3,g.cmp(g.abs(b),'>',g.c(20*pi/180))),g.c(2),stage);
        stage=g.choose(g.logic('AND',is3,g.cmp(g.abs(a),'>',g.c(23*pi/180))),one,stage);
        g.section('Swingup_and_LQR');
        energy=g.add(g.gain(g.mul(omega,omega),0.5*0.005160863235),g.gain(g.sub(g.trig(a,'cos'),one),0.3534*9.81*0.12));
        sigma=g.sub(g.c(pi),g.abs(a));
        beta=g.choose(g.cmp(sigma,'<=',g.c(1.7824)),g.c(2.1876), ...
            g.choose(g.cmp(sigma,'<=',g.c(1.8504)),g.c(4.9156),g.c(8.2100)));
        sw=g.mul(energy,omega,g.trig(a,'cos'));
        requested=g.choose(g.logic('AND',g.cmp(g.abs(sw),'<',g.c(1e-9)),g.cmp(g.abs(a),'>',g.c(2.8))), ...
            g.c(2.1876),g.choose(g.cmp(sw,'>',z),g.gain(beta,-1),beta));
        u1=g.choose(g.logic('OR',g.cmp(g.abs(velocity),'<',g.c(0.12)),g.cmp(g.mul(requested,velocity),'<=',z)),requested,z);
        state=g.mux(x,a,b,velocity,omega,omega2);
        lqr=g.block('Math Operations/Gain','Double_LQR',{state},'Gain',mat2str(-C.K,17),'Multiplication','Matrix(K*u)');
        state1=g.mux(x,a,velocity,omega);
        lqr1=g.block('Math Operations/Gain','First_rod_LQR',{state1}, ...
            'Gain',mat2str(-[-4.7434164902525747 59.170534176527148 -6.5584718328315056 5.4718267547671294],17),'Multiplication','Matrix(K*u)');
        E2=g.add(g.gain(g.mul(omega2,omega2),0.5*0.007028721867),g.gain(g.sub(g.trig(b,'cos'),one),0.1016*9.81*0.23));
        pump=g.choose(g.cmp(g.abs(b),'>',g.c(1.146)),g.gain(g.sign(omega2),1.4), ...
            g.gain(g.sign(g.mul(g.sub(E2,g.c(0.005)),omega2,g.trig(b,'cos'))),5));
        capture=g.logic('AND',g.cmp(g.abs(a),'<=',g.c(0.20)),g.cmp(g.abs(b),'<=',g.c(0.45)), ...
            g.cmp(g.abs(omega),'<=',g.c(1.20)),g.cmp(g.abs(omega2),'<=',g.c(9)));
        u2=g.choose(capture,lqr,g.add(lqr1,pump));
        u=g.choose(g.cmp(stage,'==',one),u1,g.choose(g.cmp(stage,'==',g.c(2)),u2,lqr));
        brake=g.logic('AND',g.cmp(stage,'~=',g.c(3)),g.cmp(g.abs(x),'>=',g.c(0.22)),g.cmp(g.mul(x,velocity),'>',z));
        stoppingDistance=g.gain(g.minmax('max',g.sub(g.c(0.30-0.015),g.abs(x)),g.c(0.005)),2);
        braking=g.mul(g.gain(g.sign(x),-1),g.sat(g.div(g.mul(velocity,velocity),stoppingDistance),2,30));
        u=g.sat(g.choose(brake,braking,u),-30,30);
        g.bind(b0,b); g.bind(initialized,one); g.bind(vel0,velocity); g.bind(om0,omega); g.bind(om20,omega2); g.bind(st0,stage);
    end
    g.section('Output_guard');
    g.bind(q0,x); g.bind(a0,a);
    ls=g.split(limits,2);
    fault=g.logic('OR',ls{1},ls{2},g.cmp(g.abs(x),'>=',g.c(C.positionStop(order))));
    values=g.mux(x,a,b,velocity,omega,omega2,u,servo);
    finite=g.cmp(g.abs(values),'<',g.c(inf));
    allFinite=g.block('Logic and Bit Operations/Logical Operator','AllFinite',{finite},'Operator','AND','Inputs','1');
    fault=g.logic('OR',fault,g.logic('NOT',allFinite));
    g.section('');
    g.out('Acceleration_m_s2',1,g.choose(g.logic('AND',g.cmp(servo,'==',one),g.logic('NOT',fault)),u,z));
    % Local logging is optional and is never consumed by the hardware adapter.
    g.block('Sinks/To Workspace','Stage_log',{stage},'VariableName','pn_stage','SaveFormat','Array');
    for k=1:5
        line=add_line(model,['Hardware/' num2str(k)],['Control/' num2str(k)],'autorouting','on');
        labels={'Angle1 (deg)','Angle2 (deg)','Position (m)','Limits [left right]','Servo (0 / 1)'};
        set_param(line,'Name',labels{k});
    end
    line=add_line(model,'Control/1','Hardware/1','autorouting','on'); set_param(line,'Name','Acceleration (m/s^2)');
    runLabel='Run: input preflight, home, zero, swing-up and balance. Stop: outputs off.';
    note=Simulink.Annotation(model,sprintf('PENDULUM | ORDER %d   |   Ts = %g s\n%s',order,C.dt(order),runLabel));
    note.Position=[45 30];
    g.organize();
    blocks=find_system(p,'SearchDepth',1,'Type','Block'); core=[];
    for j=2:numel(blocks)
        if ~ismember(get_param(blocks{j},'BlockType'),{'Inport','Outport','ToWorkspace'})
            core(end+1)=get_param(blocks{j},'Handle'); %#ok<AGROW>
        end
    end
    Simulink.BlockDiagram.createSubsystem(core,'Name','Swingup_and_balance');
    Simulink.BlockDiagram.arrangeSystem([p '/Swingup_and_balance']);
    Simulink.BlockDiagram.arrangeSystem(p);
    subs=find_system(model,'BlockType','SubSystem');
    for j=1:numel(subs), set_param(subs{j},'ContentPreviewEnabled','off'); end
    for sub={'Hardware','Control'}
        path=[model '/' sub{1}];
        ports=[find_system(path,'SearchDepth',1,'BlockType','Inport'); find_system(path,'SearchDepth',1,'BlockType','Outport')];
        for j=1:numel(ports), set_param(ports{j},'Name',regexprep(get_param(ports{j},'Name'),'_\d+$','')); end
    end
    set_param(model,'ZoomFactor','FitSystem');
    save_system(model,fullfile(here,[model '.slx']));
end
    function y=rate(q,old,filtered,isAngle)
        difference=g.sub(q,g.choose(initialized,old,q));
        if isAngle, difference=g.wrap(difference); end
        y=g.add(g.gain(filtered,alpha),g.div(g.gain(difference,1-alpha),g.c(C.dt(2))));
    end
end
