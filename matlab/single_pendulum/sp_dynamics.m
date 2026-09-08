function dX = sp_dynamics(X,u,P)
%SP_DYNAMICS Positive acceleration convention matches reference/P_1_1.m.
dX = [X(3); X(4); u; P.m*P.l/P.J*(P.g*sin(X(2))+u*cos(X(2)))];
end
