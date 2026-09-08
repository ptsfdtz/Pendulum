function dX = sp_dynamics_vector(input)
%SP_DYNAMICS_VECTOR Offline Simulink adapter: [state(4); acceleration].
dX = sp_dynamics(reshape(input(1:4),4,1),input(5),sp_config());
end
