function ctrl= nonlinear_controller(quad)
import casadi.*
opti = casadi.Opti(); % Optimization problem
N = 15; % MPC horizon [SET THIS VARIABLE]
% −−−− decision variables −−−−−−−−−
X = opti.variable(12,N+1); % state trajectory variables
U = opti.variable(4, N); % control trajectory (throttle, brake)
X0 = opti.parameter(12,1); % initial state
REF = opti.parameter(4,1); % reference position [x,y,z,yaw]
%%%%%%%%%%%%%%%%%%%%%%%%
% System dynamics
h = quad.Ts;
f_quad= @(X,U) quad.RK4(X,U,h);

% steady-state variables
Xs = opti.variable(12,1); 
Us = opti.variable(4, 1);

%% perform control action

Q_xs_ref = 100*eye(4);
Q = 2*eye(12);
R = 3*eye(4);

% initial conditions
opti.subject_to(X(:,1)==X0);
% we can not guarantee existence of states variable satisfying reference
% ouput under constraints.
% cost to minimize disparity between reference ouput and desired ouput
cost = (Xs([10,11,12,6])-REF)'*Q_xs_ref*(Xs([10,11,12,6])-REF);

for i = 2:N
    cost = cost + (X(:,i)-Xs)'*Q*(X(:,i)-Xs)+(U(:,i)-Us)'*R*(U(:,i)-Us);
end
cost = cost + (X(:,N+1)-Xs)'*Q*(X(:,N+1)-Xs);

% boundary conditions
u_min = quad.thrustLimits(1,1);
u_max = quad.thrustLimits(2,1);

% steady-state conditions
            
opti.subject_to(Xs == f_quad(Xs,Us));
opti.subject_to( -0.035< Xs(4));
opti.subject_to( -0.035< Xs(5));
opti.subject_to( Xs(4) < 0.035);
opti.subject_to( Xs(5) < 0.035);
opti.subject_to(u_min <= Us); 
opti.subject_to(Us <= u_max'); 

opti.minimize(cost);
% loop over control intervals
for k=1:N
    opti.subject_to(X(:,k+1) == f_quad(X(:,k), U(:,k)));
end
opti.subject_to( -0.035< X(4,:));
opti.subject_to( -0.035< X(5,:));
opti.subject_to( X(4,:) < 0.035);
opti.subject_to( X(5,:) < 0.035);
opti.subject_to(u_min <= U<= u_max); 
ctrl = @(x,ref) eval_ctrl(x, ref, opti, X0, REF, X, U);
end
function u = eval_ctrl(x, ref, opti, X0, REF, X, U)
% −−−− Set the initial state and reference −−−−
opti.set_value(X0, x);
opti.set_value(REF, ref);
% −−−− Setup solver NLP −−−−−−
ops = struct('ipopt', struct('print_level',0, 'tol', 1e-3), 'print_time', false);
opti.solver('ipopt', ops);
% −−−− Solve the optimization problem −−−−
sol = opti.solve();
assert(sol.stats.success == 1, 'Error computing optimal input');
u = opti.value(U(:,1));
% Use the current solution to speed up the next optimization
opti.set_initial(sol.value_variables());
opti.set_initial(opti.lam_g, sol.value(opti.lam_g));
end