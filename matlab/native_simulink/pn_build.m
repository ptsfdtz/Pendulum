function models=pn_build()
%PN_BUILD Native physical controllers. MATLAB is used only to construct graphs.
% Control graph: count conversion -> state estimation -> stages -> acceleration
% -> original discrete PI/integrator ordering -> travel/limit safety.
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
    add_block('simulink/User-Defined Functions/Level-2 MATLAB S-Function',[model '/Hardware_IO'], ...
        'FunctionName','pn_io_sfun','Parameters',num2str(order), ...
        'Position',[55 150 200 245]);
    p=[model '/Native_Control']; add_block('built-in/Subsystem',p,'Position',[330 145 550 265]);
    g=PnGraph(p); counts=g.in('Relative_counts',1,order+1); limits=g.in('Physical_limits',2,2);
    warningLimit=g.in('Travel_warning_m',3,1);
    g.section('Sensor_and_memory');
    z=g.c(0); one=g.c(1); cs=g.split(counts,order+1);
    % Delay states exactly match the initial ph_control struct.
    q0=g.delay('Previous_position',0); a0=g.delay('Previous_angle1',0);
    vr0=g.delay('Velocity_integrator',0); int0=g.delay('PI_integrator',0);
    x=g.gain(cs{1},-C.cartScale(order));
    a=g.wrap(g.div(g.gain(cs{2},-2*pi),g.c(C.countsPerRev(1))));
    if order==1
        enc=g.div(g.gain(cs{2},2*pi),g.c(8000)); enc0=g.delay('Previous_encoder_rad',0);
        velocity=g.div(g.sub(x,q0),g.c(C.dt(1))); omega=g.div(g.sub(a,a0),g.c(C.dt(1)));
        b=z; omega2=z;
        g.section('Swingup_and_LQR');
        lqr=g.sat(g.add(g.gain(x,10),g.gain(velocity,12.23),g.gain(a,-58.6),g.gain(omega,-10.69)),-10,10);
        delta=g.sub(enc,enc0);
        energy=g.add(g.gain(g.sub(one,g.trig(enc,'cos')),0.134*9.8*0.223),g.gain(g.mul(delta,delta),0.0089/0.0002));
        domain=g.sub(one,g.gain(g.abs(x),0.8/0.25));
        % Clamp the log input only on the already-faulted path, avoiding complex
        % signal propagation before the native safety fault reaches the adapter.
        swing=g.sat(g.add(g.gain(g.mul(g.log(g.minmax('max',domain,g.c(realmin))),x),6), ...
            g.gain(g.sign(g.mul(g.trig(enc,'cos'),delta,g.sub(g.c(2*0.134*9.8*0.223),energy))),5)),-10,10);
        swingMode=g.cmp(g.abs(a),'>=',g.c(pi/6)); stage=g.choose(swingMode,one,g.c(2));
        u=g.choose(swingMode,swing,lqr);
        g.section('Acceleration_to_voltage');
        vf0=g.delay('Velocity_free',0); if0=g.delay('Integral_free',0);
        gate=g.logic('OR',vf0,g.logic('XOR',g.cmp(vr0,'<=',z),g.cmp(u,'<=',z)));
        vrRaw=g.add(vr0,g.mul(g.gain(u,0.005),gate)); vref=g.sat(vrRaw,-0.6,0.6);
        e=g.sub(vref,velocity);
        igate=g.logic('OR',if0,g.logic('XOR',g.cmp(int0,'<=',z),g.cmp(e,'<=',z)));
        integral=g.add(int0,g.mul(g.gain(g.gain(e,0.005),54),igate));
        voltage=g.sat(g.add(g.gain(e,0.18),integral),-1,1);
        nextVr=vrRaw; nextInt=integral; vfree=g.cmp(vrRaw,'==',vref);
        g.bind(enc0,enc);
    else
        b=g.wrap(g.sub(a,g.div(g.gain(cs{3},2*pi),g.c(C.countsPerRev(2)))));
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
        leave3=g.logic('AND',is3,g.cmp(stage,'~=',g.c(3)));
        vrStart=g.choose(leave3,velocity,vr0); intStart=g.choose(leave3,z,int0);
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
        g.section('Acceleration_to_voltage');
        vref=g.sat(g.add(vrStart,g.gain(u,C.dt(2))),-0.6,0.6);
        e=g.sub(vref,velocity); candidate=g.add(intStart,g.gain(e,C.dt(2)));
        raw=g.add(g.c(C.stationaryVoltage),g.gain(e,0.18),g.gain(candidate,54));
        voltage=g.sat(raw,-1,1);
        accept=g.logic('OR',g.cmp(raw,'==',voltage),g.logic('XOR',g.cmp(e,'<',z),g.cmp(g.sub(raw,voltage),'<',z)));
        nextInt=g.choose(accept,candidate,intStart); nextVr=vref;
        g.bind(b0,b); g.bind(initialized,one); g.bind(vel0,velocity); g.bind(om0,omega); g.bind(om20,omega2); g.bind(st0,stage);
    end
    g.section('Travel_and_faults');
    reset=g.logic('AND',g.cmp(g.abs(x),'>=',warningLimit),g.cmp(g.mul(x,voltage),'>',z));
    if order==1, recovery=z; else, recovery=g.gain(g.sign(x),-0.03); end
    applied=g.choose(reset,recovery,voltage);
    g.bind(vr0,g.choose(reset,z,nextVr)); g.bind(int0,g.choose(reset,z,nextInt));
    if order==1
        g.bind(vf0,g.choose(reset,z,vfree)); g.bind(if0,g.choose(reset,z,one));
    end
    g.bind(q0,x); g.bind(a0,a);
    ls=g.split(limits,2);
    fault=g.logic('OR',ls{1},ls{2},g.cmp(g.abs(x),'>=',g.c(C.positionStop(order))));
    values=g.mux(x,a,b,velocity,omega,omega2,voltage,u);
    finite=g.cmp(g.abs(values),'<',g.c(inf));
    allFinite=g.block('Logic and Bit Operations/Logical Operator','AllFinite',{finite},'Operator','AND','Inputs','1');
    fault=g.logic('OR',fault,g.logic('NOT',allFinite));
    g.section('');
    g.out('Voltage_V',1,g.choose(fault,z,applied)); g.out('Fault',2,g.double(fault));
    g.out('Telemetry',3,g.mux(x,a,b,velocity,omega,omega2,u,voltage,stage,vref,g.double(reset)));
    add_block('simulink/Signal Routing/Mux',[model '/I_O_Command'],'Inputs','3','Position',[625 150 630 250]);
    add_line(model,'Hardware_IO/1','Native_Control/1','autorouting','on');
    add_line(model,'Hardware_IO/2','Native_Control/2','autorouting','on');
    add_line(model,'Hardware_IO/3','Native_Control/3','autorouting','on');
    for k=1:3, add_line(model,['Native_Control/' num2str(k)],['I_O_Command/' num2str(k)],'autorouting','on'); end
    add_line(model,'I_O_Command/1','Hardware_IO/1','autorouting','on');
    runLabel=sprintf('Start with start_pendulum_simulink(%d). Default Run is read-only.',order);
    if order==1, runLabel='Run: input preflight, home, zero, swing-up and balance. Stop: outputs off.'; end
    note=Simulink.Annotation(model,sprintf('NATIVE PHYSICAL CONTROL | ORDER %d\nTs = %g s    StopTime = inf\n%s\nControl: standard Simulink blocks; Hardware_IO: vendor MATLAB adapter.',order,C.dt(order),runLabel));
    note.Position=[45 30];
    g.organize();
    set_param(model,'ZoomFactor','FitSystem');
    save_system(model,fullfile(here,[model '.slx']));
end
    function y=rate(q,old,filtered,isAngle)
        difference=g.sub(q,g.choose(initialized,old,q));
        if isAngle, difference=g.wrap(difference); end
        y=g.add(g.gain(filtered,alpha),g.div(g.gain(difference,1-alpha),g.c(C.dt(2))));
    end
end
