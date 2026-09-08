function [xddot, theta1ddot, theta2ddot] = ...
    dp_dynamics(u, theta1, theta1dot, theta2, theta2dot, P)
%DP_DYNAMICS Nonlinear absolute-angle double-pendulum dynamics.
% theta1=theta2=0 is upright. Input u is cart acceleration [m/s^2].

if nargin < 6
    P = dp_config();
end

m1 = P.m1;
m2 = P.m2;
J1 = P.J1;
J2 = P.J2;
l1 = P.l1;
l2 = P.l2;
L1 = P.L1;
g = P.g;

effectiveU = P.simulation.actuator_gain * sign(u) * ...
    max(abs(u)-P.simulation.acceleration_deadzone,0);
friction1 = P.simulation.joint1_viscous*theta1dot + ...
    P.simulation.joint1_coulomb*tanh(theta1dot / ...
    P.simulation.friction_velocity_smoothing);
friction2 = P.simulation.joint2_viscous*theta2dot + ...
    P.simulation.joint2_coulomb*tanh(theta2dot / ...
    P.simulation.friction_velocity_smoothing);

xddot = effectiveU;
d = theta2 - theta1;
A11 = J1 + m2 * L1^2;
A12 = m2 * l2 * L1 * cos(d);
A22 = J2;

b1 = m2*l2*L1*sin(d)*theta2dot^2 ...
    + (m1*l1 + m2*L1)*g*sin(theta1) ...
    + (m1*l1 + m2*L1)*cos(theta1)*effectiveU - friction1;
b2 = -m2*l2*L1*sin(d)*theta1dot^2 ...
    + m2*l2*g*sin(theta2) ...
    + m2*l2*cos(theta2)*effectiveU - friction2;

qdd = [A11 A12; A12 A22] \ [b1; b2];
theta1ddot = qdd(1);
theta2ddot = qdd(2);
end
