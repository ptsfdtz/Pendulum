function [out,S]=ph_control(counts,S,C,order)
%PH_CONTROL Physical controller; counts relative to cart center/upright references.
% Single keeps original ETlab integrator ordering. Double uses relative joint 2.
if isempty(S)
    S=struct('q',[0 0 0],'velocity',[0 0 0],'vref',0,'integral',0, ...
        'vfree',false,'ifree',false,'encoder',0,'stage',1,'initialized',false);
end
wrap=@(a) atan2(sin(a),cos(a)); clamp=@(x,l) min(l,max(-l,x));
dt=C.dt(order); x=-counts(1)*C.cartScale(order);
a=wrap(-counts(2)*2*pi/C.countsPerRev(1));
if order==1
    encoder=counts(2)*2*pi/8000;
    omega=(a-S.q(2))/dt; velocity=(x-S.q(1))/dt;
    lqr=clamp(10*x+12.23*velocity-58.6*a-10.69*omega,10);
    delta=encoder-S.encoder;
    energy=0.134*9.8*0.223*(1-cos(encoder))+0.0089/0.0002*delta^2;
    % Guard original logarithm domain before evaluating it.
    assert(1-0.8/0.25*abs(x)>0,'pendulum:Track','Single swing-up position domain exceeded.');
    swing=clamp(log(1-0.8/0.25*abs(x))*x*6+ ...
        5*sign(cos(encoder)*delta*(2*0.134*9.8*0.223-energy)),10);
    S.stage=2; u=lqr;
    if abs(a)>=pi/6, S.stage=1; u=swing; end
    gate=S.vfree || ((S.vref<=0)~=(u<=0));
    vraw=S.vref+0.005*u*gate; vref=clamp(vraw,0.6);
    e=vref-velocity; gate=S.ifree || ((S.integral<=0)~=(e<=0));
    integral=S.integral+0.005*e*54*gate;
    voltage=clamp(0.18*e+integral,1);
    S.vfree=(vraw==vref); S.vref=vraw; S.integral=integral; S.ifree=true;
    S.encoder=encoder; q=[x a 0]; rates=[velocity omega 0];
else
    b=wrap(a-counts(3)*2*pi/C.countsPerRev(2)); q=[x a b];
    if ~S.initialized, S.q=q; end
    d=q-S.q; d(2:3)=wrap(d(2:3)); alpha=exp(-2*pi*20*dt);
    rates=alpha*S.velocity+(1-alpha)*d/dt;
    oldStage=S.stage;
    if S.stage==3 && abs(a)>23*pi/180, S.stage=1;
    elseif S.stage==3 && abs(b)>20*pi/180, S.stage=2;
    elseif S.stage==1 && abs(a)<=0.37 && abs(rates(2))<=2, S.stage=2;
    elseif S.stage==2 && abs(a)>0.70, S.stage=1;
    elseif S.stage==2 && abs(a)<=0.12 && abs(b)<=0.26 && abs(rates(1))<=0.12 && ...
            x*rates(1)<=0 && x*a<=0 && x*b>=0 && abs(rates(2))<=0.60 && abs(rates(3))<=0.80
        S.stage=3;
    end
    if oldStage==3 && S.stage~=3, S.vref=rates(1); S.integral=0; end
    state=[q rates];
    if S.stage==1
        E=0.5*0.005160863235*rates(2)^2+0.3534*9.81*0.12*(cos(a)-1);
        sigma=pi-abs(a); beta=8.2100;
        if sigma<=1.7824, beta=2.1876; elseif sigma<=1.8504, beta=4.9156; end
        sw=E*rates(2)*cos(a);
        if abs(sw)<1e-9 && abs(a)>2.8, requested=2.1876;
        elseif sw>0, requested=-beta; else, requested=beta; end
        u=0;
        if abs(rates(1))<0.12 || requested*rates(1)<=0, u=requested; end
    elseif S.stage==2
        u=-[-4.7434164902525747 59.170534176527148 -6.5584718328315056 5.4718267547671294]*[x;a;rates(1);rates(2)];
        if abs(b)>1.146, u=u+1.4*sign(rates(3));
        else
            E=0.5*0.007028721867*rates(3)^2+0.1016*9.81*0.23*(cos(b)-1);
            u=u+5*sign((E-0.005)*rates(3)*cos(b));
        end
        if abs(a)<=0.20 && abs(b)<=0.45 && abs(rates(2))<=1.20 && abs(rates(3))<=9
            u=-C.K*state';
        end
    else, u=-C.K*state';
    end
    if S.stage~=3 && abs(x)>=0.22 && x*rates(1)>0
        u=-sign(x)*min(30,max(2,rates(1)^2/(2*max(0.30-0.015-abs(x),0.005))));
    end
    u=clamp(u,30); S.vref=clamp(S.vref+dt*u,0.6); vref=S.vref;
    e=vref-rates(1); candidate=S.integral+dt*e;
    raw=C.stationaryVoltage+0.18*e+54*candidate; voltage=clamp(raw,1);
    if raw==voltage || (e<0)~=(raw-voltage<0), S.integral=candidate; end
end
S.q=q; S.velocity=rates; S.initialized=true;
assert(all(isfinite([q rates voltage u])),'pendulum:Nonfinite','Nonfinite controller output.');
out=struct('voltage',voltage,'acceleration',u,'state',[q rates], ...
    'stage',S.stage,'velocityReference',vref);
end
