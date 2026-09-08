function P = sp_config()
%SP_CONFIG Offline plant based on reference/P_1_1.m, SI units.
% State [x theta xdot thetadot], theta=0 upright; acceleration input.
P.m = 0.134; P.l = 0.2285; P.g = 9.8;
P.J = (4/3)*P.m*P.l^2; % joint inertia implied by 3/(4*l) in P_1_1
P.dt = 0.001; P.stopTime = 50; P.u_max = 10; P.x_limit = 0.35;
P.swing_initial = [0; pi-0.08; 0; 0];
P.balance_initial = [0; 0.10; 0; 0];
% Original reference acceleration feedback, reordered as [x theta v omega].
P.K = [-10 58.6 -12.23 10.69];
P.energy_gain = 5; P.velocity_limit = 0.30;
P.capture_angle = pi/6; P.capture_rate = 3;
P.soft_limit = 0.22; P.brake_limit = 0.30;
end
