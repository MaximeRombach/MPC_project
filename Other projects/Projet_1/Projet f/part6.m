function [ctrl, traj] = ctrl_NMPC(quad)
import casadi.*
opti = casadi.Opti(); % Optimization problem
N = 20; % MPC horizon [SET THIS VARIABLE]
% ???? decision variables ?????????
X = opti.variable(12,N+1); % state trajectory variables
U = opti.variable(4, N); % control trajectory (throttle, brake)
X0 = opti.parameter(12,1); % initial state
REF = opti.parameter(4,1); % reference position [x,y,z,yaw]
%%%%%%%%%%%%%%%%%%%%%%%%
%%%% YOUR CODE HERE %%%%
%%%%%%%%%%%%%%%%%%%%%%%%
h = 0.025;
f_discrete = @(x,u) RK4(x,u,h,f);
% ---- objective ---------
opti.minimize(...
  -10*sum(X(2,:))  + ... % Max velocity
  0.1*U(1,:)*U(1,:)' + ... % Minimize accel
  10*U(2,:)*U(2,:)'   + ... % Minimize braking
  10000*(epsilon_speed(1,:)*epsilon_speed(1,:)' + sum(epsilon_speed))); % Soft constraints
% ---- multiple shooting --------
for k=1:N % loop over control intervals
  
%   %%%%% REMOVE
%   opti.subject_to(X(:,k+1) == f_discrete(X(:,k), U(:,k)));
%   %%%%% REMOVE
  
  %%%% WRITE YOUR DYNAMICS CONSTRAINT HERE
  %   opti.subject_to( ... );
  
end
% ---- path constraints -----------

limit = track.maxspeed;
opti.subject_to(speed  <=   limit(pos) + epsilon_speed); % track speed limit
opti.subject_to(0 <= U <= 1);  % control is limited

% ---- boundary conditions --------
opti.subject_to(pos(1)==pos0);   % use initial position
opti.subject_to(speed(1)==v0); % use initial speed
opti.subject_to(epsilon_speed >= 0); % slack lower bound

% Pass parameter values
opti.set_value(pos0, 0.0);
opti.set_value(v0, 0);

% ---- Setup solver NLP    ------
ops = struct;
ops.ipopt.print_level = 0;
ops.ipopt.tol = 1e-3;
opti.solver('ipopt', ops);
sol = opti.solve();   % actual solve

ctrl = @(x,ref) eval_ctrl(x, ref, opti, X0, REF, X, U);
end
function u = eval_ctrl(x, ref, opti, X0, REF, X, U)
% ???? Set the initial state and reference ????
opti.set_value(X0, x);
opti.set_value(REF, ref);
% ???? Setup solver NLP ??????
ops = struct('ipopt', struct('print level',0, 'tol', 1e-3), 'print time', false);
opti.solver('ipopt', ops);
% ???? Solve the optimization problem ????
sol = opti.solve();
assert(sol.stats.success == 1, 'Error computing optimal input');
u = opti.value(U(:,1));
% Use the current solution to speed up the next optimization
opti.set_initial(sol.value_variables());
opti.set_initial(opti.lam_g, sol.value(opti.lam_g));
end