function y = sp_control_vector(X,balanceOnly)
%SP_CONTROL_VECTOR Offline Simulink adapter for the shared control function.
[u,stage] = sp_control(X(:),sp_config(),balanceOnly);
y = [u;stage];
end
