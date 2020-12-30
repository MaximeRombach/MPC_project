%% To do 2.1
quad = Quad();
[xs,us] = quad.trim(); % Compute steady−state for which 0 = f(xs,us) 
% trim computes the state and inputs xs and us
% xs(5)= 1.5;% change the yaw_s (index=6)
% xs(length(xs))= 6; % change the z_s or x_s or y_s

sys = quad.linearize(xs, us) % Linearize the nonlinear model
% linearize computes the jacobian (w.r.t x and u to compute the matrices A
% and B of the linearized system x+ = Ax + Bu

%% To do 2.2
% change of input variable
sys_transformed = sys * inv(quad.T) % New system is A * x + B * inv(T) * v

%% To do 2.3
% decomposition into 4 independant systems
[sys_x, sys_y, sys_z, sys_yaw] = quad.decompose(sys, xs, us)