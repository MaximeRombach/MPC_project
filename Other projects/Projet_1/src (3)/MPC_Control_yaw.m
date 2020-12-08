classdef MPC_Control_yaw < MPC_Control
  
  methods
    % Design a YALMIP optimizer object that takes a steady-state state
    % and input (xs, us) and returns a control input
    function ctrl_opt = setup_controller(mpc)

      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % INPUTS
      %   x(:,1) - initial state (estimate)
      %   xs, us - steady-state target
      % OUTPUTS
      %   u(:,1) - input to apply to the system
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

      [n,m] = size(mpc.B);
      
      % Steady-state targets (Ignore this before Todo 3.2)
      xs = sdpvar(n, 1);
      us = sdpvar(m, 1);
      
      % SET THE HORIZON HERE
      N = 20
      
      % Predicted state and input trajectories
      x = sdpvar(n, N);
      u = sdpvar(m, N-1);
      

      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE 

      % NOTE: The matrices mpc.A, mpc.B, mpc.C and mpc.D are 
      %       the DISCRETE-TIME MODEL of your system

      % WRITE THE CONSTRAINTS AND OBJECTIVE HERE
      A=mpc.A; B=mpc.B; C=mpc.C; D=mpc.D;
      %define the constraint on x and u
      %constraint of V = {v | Mv <= m}
      M = [1;-1;1;-1;1;-1;1;-1]; m = [0.3;0.3;0.3;0.3;0.3;0.2;0.2;0.2];
      % X = {x | Fx <= f}
      F=zeros(2*n,n); f=zeros(2*n,1);
      
      Q=diag([0.1;1]);
      R=0.01;
      % Compute optimal LQR controller for unconstrained system
      [K,Qf,~] = dlqr(A,B,Q,R);
      K = -K; % Matlab defines K as -K

        % Compute maximal invariant set
        Xf = polytope([F;M*K],[f;m]);
        Acl = [A+B*K];
        while 1
            prevXf = Xf;
            [T,t] = double(Xf);
            preXf = polytope(T*Acl,t);
            Xf = intersect(Xf, preXf);
            if isequal(prevXf, Xf)
                break
            end
        end
        [Ff,ff] = double(Xf);

%       figure(1)
%       Xf.projection(1:2).plot();
%       figure(2)
%       Xf.projection(2:3).plot();
%       figure(3)
%       Xf.projection(3:4).plot();
      
       con = (x(:,2) == A*x(:,1) + B*u(:,1)) + (M*u(:,1) <= m);
       obj = u(:,1)'*R*u(:,1);
    for i = 2:N-1
       con = con + (x(:,i+1) == A*x(:,i) + B*u(:,i));
       con = con + (M*u(:,i) <= m);
       obj = obj + x(:,i)'*Q*x(:,i) + u(:,i)'*R*u(:,i);
    end
       con = con + (Ff*x(:,N) <= ff);
       obj = obj + x(:,N)'*Qf*x(:,N);
  
      
      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE 
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      
      
      ctrl_opt = optimizer(con, obj, sdpsettings('solver','gurobi'), ...
        {x(:,1), xs, us}, u(:,1));
    end
    
    
    % Design a YALMIP optimizer object that takes a position reference
    % and returns a feasible steady-state state and input (xs, us)
    function target_opt = setup_steady_state_target(mpc)

      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % INPUTS
      %   ref    - reference to track
      % OUTPUTS
      %   xs, us - steady-state target
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

      % Steady-state targets
      n = size(mpc.A,1);
      xs = sdpvar(n, 1);
      us = sdpvar;
      
      % Reference position (Ignore this before Todo 3.2)
      ref = sdpvar;            
            
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE       
      % You can use the matrices mpc.A, mpc.B, mpc.C and mpc.D
 A=mpc.A; B=mpc.B; C=mpc.C; D=mpc.D;
nx = size(A,1);
nu = size(B,2);

u = sdpvar(nu,1);
x = sdpvar(nx,1);
umin=0;
umax=1.5;


con = [umin <= u <= umax ,...
                x == A*x + B*u    ,...
               ref == C*x   ];

obj  = u^2;
diagnostics = solvesdp(con,obj,sdpsettings('verbose',0));

if diagnostics.problem == 0
   % Good! 
elseif diagnostics.problem == 1
    throw(MException('','Infeasible'));
else
    throw(MException('','Something else happened'));
end

xs = x;
us = u;

      
      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE 
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      
      
      % Compute the steady-state target
      target_opt = optimizer(con, obj, sdpsettings('solver', 'gurobi'), ref, {xs, us});
      
    end
  end
end
