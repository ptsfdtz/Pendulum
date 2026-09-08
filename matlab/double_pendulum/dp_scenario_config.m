function P = dp_scenario_config(scenario)
%DP_SCENARIO_CONFIG Explicit initial conditions for each reproduction case.
if nargin < 1, scenario = 'swingup'; end
scenario = validatestring(scenario,{'swingup','balance'});
P = dp_config();
if strcmp(scenario,'balance')
    P.control_mode = 2; P.stopTime = 15;
    P.x0 = 0; P.theta1_0 = -0.13; P.theta2_0 = 0.09;
    P.xdot0 = 0; P.theta1dot_0 = 0; P.theta2dot_0 = 0;
end
end
