function pn_build_hardware(model,order,C)
% Hardware boundary: calibrated SI/degree inputs, automatic enable, velocity PI.
p=[model '/Hardware']; add_block('built-in/Subsystem',p,'Position',[120 140 350 410]);
g=PnGraph(p); requested=g.in('Acceleration_m_s2',1,1);
io=g.block('User-Defined Functions/Level-2 MATLAB S-Function','Device_IO',{}, ...
    'FunctionName','pn_io_sfun','Parameters',num2str(order));
root=io(1:end-1); counts=[root '1']; limits=[root '2']; warningLimit=[root '3']; servo=[root '4'];
g.section('Sensors');
z=g.c(0); one=g.c(1); cs=g.split(counts,order+1);
x=g.gain(cs{1},-C.cartScale(order));
a=g.wrap(g.gain(cs{2},-2*pi/C.countsPerRev(1)));
if order==1, b=z; else, b=g.wrap(g.sub(a,g.gain(cs{3},2*pi/C.countsPerRev(2)))); end
angle1=g.gain(a,180/pi); angle2=g.gain(b,180/pi);
q0=g.delay('Previous_position',0); a0=g.delay('Previous_angle1',0); b0=g.delay('Previous_angle2',0);
if order==1
    velocity=g.gain(g.sub(x,q0),1/C.dt(order));
    omega=g.gain(g.sub(a,a0),1/C.dt(order)); omega2=z;
else
    initialized=g.delay('Initialized',0); alpha=exp(-2*pi*20*C.dt(order));
    velocity=rate(x,q0,false,'Cart_velocity'); omega=rate(a,a0,true,'Angular_velocity1'); omega2=rate(b,b0,true,'Angular_velocity2');
    g.bind(initialized,one);
end
g.bind(q0,x); g.bind(a0,a); g.bind(b0,b);
g.section('Acceleration_to_voltage');
vr0=g.delay('Velocity_integrator',0); int0=g.delay('PI_integrator',0);
u=g.sat(requested,-10-20*(order==2),10+20*(order==2));
if order==1
    vf0=g.delay('Velocity_free',0); if0=g.delay('Integral_free',0);
    gate=g.logic('OR',vf0,g.logic('XOR',g.cmp(vr0,'<=',z),g.cmp(u,'<=',z)));
    vrRaw=g.add(vr0,g.mul(g.gain(u,0.005),gate)); vref=g.sat(vrRaw,-0.6,0.6);
    e=g.sub(vref,velocity);
    igate=g.logic('OR',if0,g.logic('XOR',g.cmp(int0,'<=',z),g.cmp(e,'<=',z)));
    integral=g.add(int0,g.mul(g.gain(g.gain(e,0.005),54),igate));
    voltage=g.sat(g.add(g.gain(e,0.18),integral),-1,1);
    nextVr=vrRaw; nextInt=integral; vfree=g.cmp(vrRaw,'==',vref);
else
    vref=g.sat(g.add(vr0,g.gain(u,C.dt(order))),-0.6,0.6);
    e=g.sub(vref,velocity); candidate=g.add(int0,g.gain(e,C.dt(order)));
    raw=g.add(g.c(C.stationaryVoltage),g.gain(e,0.18),g.gain(candidate,54));
    voltage=g.sat(raw,-1,1);
    accept=g.logic('OR',g.cmp(raw,'==',voltage),g.logic('XOR',g.cmp(e,'<',z),g.cmp(g.sub(raw,voltage),'<',z)));
    nextInt=g.choose(accept,candidate,int0); nextVr=vref;
end
g.section('Travel_and_faults');
soft=g.logic('AND',g.cmp(g.abs(x),'>=',warningLimit),g.cmp(g.mul(x,voltage),'>',z));
if order==1, recovery=z; else, recovery=g.gain(g.sign(x),-0.03); end
applied=g.choose(soft,recovery,voltage);
ls=g.split(limits,2);
fault=g.logic('OR',ls{1},ls{2},g.cmp(g.abs(x),'>=',g.c(C.positionStop(order))));
values=g.mux(x,a,b,velocity,omega,omega2,requested,voltage,servo,warningLimit);
finite=g.cmp(g.abs(values),'<',g.c(inf));
allFinite=g.block('Logic and Bit Operations/Logical Operator','AllFinite',{finite},'Operator','AND','Inputs','1');
fault=g.logic('OR',fault,g.logic('NOT',allFinite));
disabled=g.cmp(servo,'~=',one); reset=g.logic('OR',soft,disabled,fault);
g.bind(vr0,g.choose(reset,z,nextVr)); g.bind(int0,g.choose(reset,z,nextInt));
if order==1
    g.bind(vf0,g.choose(reset,z,vfree)); g.bind(if0,g.choose(reset,z,one));
end
volts=g.choose(g.logic('OR',disabled,fault),z,applied);
% Stage slot is zero: hardware does not infer the replaceable algorithm's state.
packet=g.mux(volts,g.double(fault),x,a,b,velocity,omega,omega2,u,voltage,z,vref,g.double(reset));
g.bind(io,packet);
g.section('');
g.out('Angle1_deg',1,angle1); g.out('Angle2_deg',2,angle2); g.out('Position_m',3,x);
g.out('Limits_left_right',4,limits); g.out('Servo',5,servo);
g.organize();
% createSubsystem can expose a shared constant that is unused for order 1.
blocks=find_system(p,'SearchDepth',1,'BlockType','SubSystem');
for j=2:numel(blocks)
    ports=get_param(blocks{j},'PortHandles');
    for k=1:numel(ports.Outport)
        line=get_param(ports.Outport(k),'Line');
        if line==-1
            name=sprintf('Unused_%d_%d',j,k);
            add_block('simulink/Sinks/Terminator',[p '/' name]);
            dest=get_param([p '/' name],'PortHandles'); add_line(p,ports.Outport(k),dest.Inport,'autorouting','on');
        end
    end
end
Simulink.BlockDiagram.arrangeSystem(p);
    function y=rate(q,old,isAngle,label)
        filtered=g.delay(label,0); difference=g.sub(q,g.choose(initialized,old,q));
        if isAngle, difference=g.wrap(difference); end
        y=g.add(g.gain(filtered,alpha),g.gain(difference,(1-alpha)/C.dt(order))); g.bind(filtered,y);
    end
end
