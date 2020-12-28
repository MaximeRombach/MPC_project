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
      N = 15;
      
      % Predicted state and input trajectories
      x = sdpvar(n, N);
      u = sdpvar(m, N-1);
      
      A = mpc.A;
      B = mpc.B;
      Q = 10*eye(n);
      R = 1;

      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE 

      % NOTE: The matrices mpc.A, mpc.B, mpc.C and mpc.D are 
      %       the DISCRETE-TIME MODEL of your system

      % WRITE THE CONSTRAINTS AND OBJECTIVE HERE
            
      % Conditions on state for x
      F = [];

      f = [];
      
      % Conditions on inputs for M_beta
      M = [1; -1]; m = [0.2;0.2];
      
      %%%% Conventional way %%%%
      [K,Qf,~] = dlqr(A,B,Q,R);
      K = -K;
      
    
      Xf = polytope([F;M*K],[f;m]);
      k = 1;
      Acl = A + B*K;
      fprintf("Computing max invariant set for Yaw\n")
      while 1
          prev_O = Xf;
          [T,t] = double(Xf);
          pre_O = polytope(T*Acl, t); % Compute pre set 
          Xf = intersect(pre_O,Xf); % Intersect pre set and set
          if prev_O == Xf
              fprintf("Finished at iteration n°%d\n", k)
              break;
          end
          fprintf("Iteration n°%d \n", k)
          k = k + 1;

      end
      
      [Ff, ff] = double(Xf);
      
      con = [];
      obj = 0;      
      con = [con, x(:,2) == A*x(:,1) + B*u(:,1)];
      con = [con M*u(:,1) <= m];
      obj = u(:,1)'*R*u(:,1);
      for i = 2:N-1
          con = [con, x(:,i+1) == A*x(:,i) + B*u(:,i)]; % System dynamics
          con = [con, + M*u(:,i) <= m]; % Input constraints
          obj = obj + x(:,i)'*Q*x(:,i) + u(:,i)'*R*u(:,i); 
      end
        con = [con, (Ff*x(:,N) <= ff)]; % Terminal constraint
        obj = obj + x(:,N)'*Qf*x(:,N);% Terminal weight
      
      figure
      Xf.projection(1:2).plot();
      xlabel("Yaw velocity"); ylabel("Yaw")
      sgtitle('MPC Control yaw')

      
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
      con = [];
      obj = 0;

      
      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE 
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      
      
      % Compute the steady-state target
      target_opt = optimizer(con, obj, sdpsettings('solver', 'gurobi'), ref, {xs, us});
      
    end
  end
end
