function [u,stage] = sp_control(X,P,balanceOnly)
%SP_CONTROL Energy swing-up and upright LQR for the ideal offline plant.
% Uses upright-zero physical energy, not the raw encoder energy in ETlab.
a = atan2(sin(X(2)),cos(X(2)));
if balanceOnly || (abs(a) < P.capture_angle && abs(X(4)) < P.capture_rate)
    stage = 2;
    u = -P.K*[X(1);a;X(3);X(4)];
else
    stage = 1;
    E = 0.5*P.J*X(4)^2 + P.m*P.g*P.l*(cos(a)-1);
    requested = -P.energy_gain*sign(E*X(4)*cos(a));
    if abs(X(3)) >= P.velocity_limit && requested*X(3) > 0
        u = 0;
    else
        u = requested;
    end
    if abs(X(1)) >= P.soft_limit && X(1)*X(3) > 0
        remaining = max(P.brake_limit-0.015-abs(X(1)),0.005);
        u = -sign(X(1))*max(2,X(3)^2/(2*remaining));
    end
end
u = min(max(u,-P.u_max),P.u_max);
end
