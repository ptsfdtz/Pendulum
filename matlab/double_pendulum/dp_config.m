function P = dp_config()
%DP_CONFIG Central configuration for the double inverted pendulum.

% Physical parameters of the actual apparatus.
% J1 is the mean of two finite-amplitude-corrected physical-pendulum tests
% recorded on 2026-09-04. J1 and J2 are inertias about their joint axes.
% J2 uses a uniform-thin-rod centroidal inertia plus the measured l2 via
% the parallel-axis theorem.
P.m1 = 0.3534;
P.m2 = 0.1016;
P.J1 = 0.005160863235;
P.J2 = 0.007028721867;
P.l1 = 0.12;
P.l2 = 0.23;
P.L1 = 0.1294;
P.L2 = 0.442;
P.g = 9.81;

% Optional non-idealities used by robust offline searches.  Defaults keep
% the nominal Simulink plant unchanged.
P.simulation.actuator_gain = 1.0;
P.simulation.acceleration_deadzone = 0.0;
P.simulation.joint1_viscous = 0.0;
P.simulation.joint2_viscous = 0.0;
P.simulation.joint1_coulomb = 0.0;
P.simulation.joint2_coulomb = 0.0;
P.simulation.friction_velocity_smoothing = 0.05;

% Safety and operating limits.
P.x_limit = 0.35;
P.u_max = 30;

% Default final-validation initial state. Mode 4 starts both links close to
% the downward equilibrium; the small offsets make the ideal simulation
% leave the exact zero-velocity equilibrium deterministically.
P.x0 = 0;
% Opposite small offsets avoid the non-physical perfectly symmetric limit
% cycle of the deterministic, noise-free sign-switching simulation.
P.theta1_0 = pi - 0.08;
P.theta2_0 = -pi + 0.06;
P.xdot0 = 0;
P.theta1dot_0 = 0;
P.theta2dot_0 = 0;

% 0=free, 1=test, 2=saved best LQR, 3=reserved VGLQR, 4=paper swing-up.
P.control_mode = 4;
P.test_frequency = 0.25;
P.test_accel = 0.25;

% Simulink settings.
P.stopTime = 50;
P.maxStep = 0.001;
P.animationTs = 0.02;
P.figurePosition = [80 100 1200 600];
P.viewX = 0.65;
P.viewYMin = -0.20;
P.viewYMax = 0.85;
P.cartWidth = 0.14;
P.cartHeight = 0.07;

% Paper-derived three-stage swing-up supervisor (paper equation 71).
% Energy targets are scaled by each apparatus' m*g*l energy scale because
% the measured plant differs substantially from the paper apparatus.
S.epsilon1 = 0.37;
S.epsilon2 = 0.115;
S.velocity_limit = 0.12;
S.sigma1 = 1.7824;
S.sigma2 = 1.8504;
S.sigma3 = 1.9391;
% Robust nonlinear search selected 2x the paper-adapted stage-1 gains.
S.beta11 = 2.1876;
S.beta12 = 4.9156;
S.beta13 = 8.2100;
S.alpha = 1.146;
S.beta21 = 1.40;
S.beta22 = 5.00;
S.second_input_sign = -1;
S.paper_energy1_scale = 2.34/(0.304*9.81*0.33);
S.paper_energy2_scale = 0.19/(0.093*9.81*0.149);
S.paper_energy1_target = ...
    (2.34/(0.304*9.81*0.33))*0.3534*9.81*0.12;
S.energy1_target = 0;
S.energy2_target = 0.005;
% Boundary layer for the discontinuous energy switch. Zero reproduces the
% paper's ideal sign law; robust searches may select a small positive value.
S.energy_switch_smoothing = 0;
% The current nonlinear model has the opposite acceleration-energy sign
% from equation (21), as verified from dp_dynamics; compensate here.
S.energy1_input_sign = -1;
S.track_limit = 0.30;
% At 0.30 m the automatic controller must drive inward and continue.  Only
% the independent 0.35 m guard is terminal in numerical simulation.
S.hard_track_limit = 0.35;
S.soft_track_limit = 0.22;
S.track_brake_margin = 0.015;
S.minimum_brake_accel = 2.0;
S.numeric_dt = 0.001;
S.initial_stage = 1;
S.capture_angle1 = 0.12;
S.capture_angle2 = 0.26;
S.capture_rate1 = 0.60;
S.capture_rate2 = 0.80;
S.capture_cart_speed = 0.12;
S.capture_assist_angle1 = 0.20;
S.capture_assist_angle2 = 0.45;
S.capture_assist_rate1 = 1.20;
S.capture_assist_rate2 = 9.00;
S.stage1_capture_rate = 2.0;
S.stage1_reentry_angle = 0.70;

% Offline fixed upright gain for paper stage u21. It stabilizes the first
% link while u22 swings the second link. State: [x theta1 xdot theta1dot].
S.Ks = [-4.7434164902525747 59.170534176527148 ...
    -6.5584718328315056 5.4718267547671294];
P.swingup = S;


% LQR optimization budget and deterministic sampling.
P.optimize.enabled = true;
P.optimize.coarse_trials = 300;
P.optimize.refine_trials = 300;
P.optimize.top_count = 10;
P.optimize.refine_log10_sigma = 0.35;
P.optimize.sim_time = 8;
P.optimize.dt = 0.005;
P.optimize.settle_window = 0.5;
P.optimize.theta_tol = 0.02;
P.optimize.theta_dot_tol = 0.1;
P.optimize.xdot_tol = 0.05;
P.optimize.random_seed = 1;
P.optimize.checkpoint_interval = 20;
P.optimize.progress_interval = 10;
P.optimize.make_plots = true;
P.optimize.max_abs_angular_rate = 200;
P.optimize.max_abs_state = 1e4;

% Rows follow [x theta1 theta2 xdot theta1dot theta2dot].
P.optimize.test_initial_states = [ ...
    0 -0.13  0.09 0 0 0;
    0  0.10 -0.10 0 0 0;
    0  0.15  0.10 0 0 0;
    0 -0.15 -0.10 0 0 0;
    0  0.20 -0.15 0 0 0];

% Search bounds for [qx qtheta1 qtheta2 qxdot qtheta1dot qtheta2dot R].
P.optimize.lower = [0.1 10 10 0.001 0.001 0.001 0.01];
P.optimize.upper = [1000 10000 20000 100 500 500 100];

% Continuous objective weights.
P.optimize.weights.theta1 = 140;
P.optimize.weights.theta2 = 180;
P.optimize.weights.x = 45;
P.optimize.weights.theta1dot = 2.0;
P.optimize.weights.theta2dot = 2.0;
P.optimize.weights.u = 0.015;
P.optimize.weights.settling_time = 12;
P.optimize.weights.max_x = 1500;
P.optimize.weights.saturation = 150;
P.optimize.penalty_unstable = 1e7;
P.optimize.penalty_track = 1e8;

% Reference-paper baseline (zero velocity weights are intentional here only).
P.paper.Q = diag([10 300 500 0 0 0]);
P.paper.R = 1;
P.paper.A = [ ...
    0 0 0 1 0 0;
    0 0 0 0 1 0;
    0 0 0 0 0 1;
    0 0 0 0 0 0;
    0 27.608 -5.7726 0 0 0;
    0 -57.013 56.368 0 0 0];
P.paper.B = [0; 0; 0; 1; 2.2258; -0.065769];
end
