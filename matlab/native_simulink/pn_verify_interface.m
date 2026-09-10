function report=pn_verify_interface()
% Offline tests of the actual saved graphs; no vendor I/O or motion.
here=fileparts(mfilename('fullpath')); addpath(here,fullfile(here,'..','hardware'));
C=ph_config(); report=struct();
for order=1:2
    source=sprintf('Pendulum_Native_%d',order); load_system(source);
    assert(numel(find_system([source '/Control'],'SearchDepth',1,'BlockType','Inport'))==5);
    assert(numel(find_system([source '/Control'],'SearchDepth',1,'BlockType','Outport'))==1);
    assert(isempty(find_system([source '/Control'],'BlockType','Goto')));
    assert(isempty(find_system([source '/Control'],'BlockType','From')));
    model=sprintf('Pn_Interface_Test_%d',order); if bdIsLoaded(model), close_system(model,0); end
    new_system(model); cleanup=onCleanup(@()close_system(model,0));
    set_param(model,'SolverType','Fixed-step','Solver','FixedStepDiscrete','FixedStep',num2str(C.dt(order)), ...
        'SaveTime','off','SaveOutput','off','ReturnWorkspaceOutputs','on');
    add_block([source '/Hardware'],[model '/Hardware']); add_block([source '/Control'],[model '/Control']);
    io=find_system([model '/Hardware'],'BlockType','M-S-Function'); assert(numel(io)==1); io=io{1};
    pos=get_param(io,'Position'); ports=get_param(io,'PortHandles');
    inputLine=get_param(ports.Inport(1),'Line'); packetSource=get_param(inputLine,'SrcPortHandle'); delete_line(inputLine);
    destinations=cell(1,4);
    for k=1:4
        line=get_param(ports.Outport(k),'Line'); destinations{k}=get_param(line,'DstPortHandle'); delete_line(line);
    end
    delete_block(io);
    add_block('built-in/Subsystem',io,'Position',pos);
    add_block('simulink/Ports & Subsystems/In1',[io '/Packet']);
    add_block('simulink/Sinks/To Workspace',[io '/Record'],'VariableName','packet','SaveFormat','Array');
    add_line(io,'Packet/1','Record/1');
    variables={'testCounts','testLimits','testWarning','testServo'};
    for k=1:4
        nm=sprintf('Input%d',k); out=sprintf('Output%d',k);
        add_block('simulink/Sources/From Workspace',[io '/' nm],'VariableName',variables{k},'Interpolate','off','OutputAfterFinalValue','Holding final value');
        add_block('simulink/Ports & Subsystems/Out1',[io '/' out],'Port',num2str(k)); add_line(io,[nm '/1'],[out '/1']);
    end
    ports=get_param(io,'PortHandles'); add_line([model '/Hardware'],packetSource,ports.Inport(1),'autorouting','on');
    for k=1:4
        for dest=destinations{k}(:)', add_line([model '/Hardware'],ports.Outport(k),dest,'autorouting','on'); end
    end
    for k=1:5, add_line(model,['Hardware/' num2str(k)],['Control/' num2str(k)]); end
    add_line(model,'Control/1','Hardware/1');
    add_block('simulink/Sinks/To Workspace',[model '/Acceleration'],'VariableName','acceleration','SaveFormat','Array');
    add_line(model,'Control/1','Acceleration/1');
    for k=1:5
        nm=sprintf('Sensor%d',k); add_block('simulink/Sinks/To Workspace',[model '/' nm],'VariableName',nm,'SaveFormat','Array');
        add_line(model,['Hardware/' num2str(k)],[nm '/1']);
    end
    n=2400; t=(0:n-1)'*C.dt(order); k=(0:n-1)';
    counts=[round(150*sin(k/70)),round(3900*sin(k/200)),round(1800*sin(k/180))];
    counts(1:50,:) = repmat([0 -4000 0],50,1);
    counts(51:100,2)=round(linspace(-4000,counts(100,2),50));
    counts(700:850,:)=0; counts(851:900,:)=repmat([0 -950 0],50,1);
    counts(1300:1500,:)=0;
    limits=zeros(n,2); servo=ones(n,1);
    out=run(counts,limits,servo);
    q=[-counts(:,1)*C.cartScale(order),atan2(sin(-counts(:,2)*2*pi/8000),cos(-counts(:,2)*2*pi/8000))];
    b=zeros(n,1); if order==2, b=atan2(sin(q(:,2)-counts(:,3)*2*pi/4000),cos(q(:,2)-counts(:,3)*2*pi/4000)); end
    assert(max(abs(out.Sensor1-q(:,2)*180/pi))<1e-10);
    assert(max(abs(out.Sensor2-b*180/pi))<1e-10); assert(max(abs(out.Sensor3-q(:,1)))<1e-12);
    assert(isequal(out.Sensor4,limits) && isequal(out.Sensor5,servo));
    S=[]; expected=zeros(n,1); stages=zeros(n,1);
    for j=1:n
        [o,S]=ph_control(counts(j,1:order+1),S,C,order); expected(j)=o.acceleration; stages(j)=o.stage;
    end
    delta=max(abs(out.acceleration-expected));
    assert(delta<1e-8,'Acceleration regression: %.17g',delta);
    assert(isequal(out.pn_stage,stages));
    assert(all(isfinite(out.packet),'all') && max(abs(out.packet(:,1)))<=1);
    % Compare the motor adapter with an independent recurrence driven only by a.
    reference=actuator(out.acceleration,out.packet(:,6),order,C);
    voltageDelta=max(abs(reference-out.packet(:,1))); assert(voltageDelta<1e-9,'Voltage recurrence: %.17g',voltageDelta);
    % Cross the angular wrap with physically small encoder steps.
    circle=zeros(n,3); circle(:,2)=round(linspace(-4000,5000,n));
    wrapped=run(circle,zeros(n,2),ones(n,1)); S=[]; wrapExpected=zeros(n,1);
    for j=1:n, [o,S]=ph_control(circle(j,1:order+1),S,C,order); wrapExpected(j)=o.acceleration; end
    wrapDelta=max(abs(wrapped.acceleration-wrapExpected)); assert(wrapDelta<1e-8,'Angle wrap regression: %.17g',wrapDelta);
    % Toggle enable and each limit after PI has accumulated state.
    counts=zeros(n,3); counts(:,2)=20; servo(100:199)=0;
    limits(300:309,1)=1; limits(500:509,2)=1; limits(600,:)=[1 1];
    out=run(counts,limits,servo);
    stopped=~servo | any(limits,2);
    assert(all(out.acceleration(stopped)==0) && all(out.packet(stopped,1)==0));
    assert(all(out.packet(any(limits,2),2)==1));
    % Independent hardware defense even if a replacement algorithm ignores limits.
    delete_line(model,'Control/1','Hardware/1');
    add_block('simulink/Sources/Constant',[model '/ExternalAlgorithm'],'Value','externalAcceleration');
    add_line(model,'ExternalAlgorithm/1','Hardware/1');
    for val=[1000 NaN Inf]
        out=run(counts,limits,servo,val);
        if isfinite(val)
            assert(max(abs(out.packet(:,1)))<=1 && all(out.packet(stopped,1)==0));
        else
            assert(all(out.packet(:,1)==0) && all(out.packet(:,2)==1));
        end
    end
    counts(:,1)=ceil(C.positionStop(order)/C.cartScale(order));
    out=run(counts,zeros(n,2),ones(n,1),5);
    assert(all(out.packet(:,1)==0) && all(out.packet(:,2)==1));
    report.(sprintf('order%d',order))=struct('samples',n,'acceleration_error',delta,'voltage_error',voltageDelta, ...
        'wrap_error',wrapDelta,'stages',unique(stages)','ports',true,'faults',true,'enable',true,'passed',true);
    clear cleanup
end
report.passed=true; report.physical_io_opened=false;
save(fullfile(here,'..','output','native_simulink','interface_verification.mat'),'report'); disp(report);
    function out=run(counts,limits,servo,external)
        input=Simulink.SimulationInput(model);
        input=input.setVariable('testCounts',[t counts(:,1:order+1)]);
        input=input.setVariable('testLimits',[t limits]);
        input=input.setVariable('testWarning',[t C.positionWarning(order)*ones(n,1)]);
        input=input.setVariable('testServo',[t servo]);
        if nargin==4, input=input.setVariable('externalAcceleration',external); end
        input=input.setModelParameter('StopTime',num2str(t(end),17)); out=sim(input);
    end
end
function volt=actuator(accel,velocity,order,C)
vr=0; integral=0; vf=false; fi=false; volt=zeros(size(accel));
clamp=@(x,l) min(l,max(-l,x));
for k=1:numel(accel)
    if order==1
        gate=vf || xor(vr<=0,accel(k)<=0); raw=vr+0.005*accel(k)*gate; ref=clamp(raw,0.6);
        e=ref-velocity(k); gate=fi || xor(integral<=0,e<=0);
        integral=integral+0.005*e*54*gate; volt(k)=clamp(0.18*e+integral,1);
        vf=raw==ref; vr=raw; fi=true;
    else
        vr=clamp(vr+C.dt(order)*accel(k),0.6); e=vr-velocity(k); candidate=integral+C.dt(order)*e;
        raw=C.stationaryVoltage+0.18*e+54*candidate; volt(k)=clamp(raw,1);
        if raw==volt(k) || xor(e<0,raw-volt(k)<0), integral=candidate; end
    end
end
end
