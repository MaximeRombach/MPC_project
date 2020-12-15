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
      N = 10;
      
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
            
      sysTemp = LTISystem('A',A,'B',B);
      sysTemp.u.max = 0.2; sysTemp.u.min = -0.2;
      sysTemp.x.penalty = QuadFunction(Q);
      sysTemp.u.penalty = QuadFunction(R);

      K = sysTemp.LQRGain;
      Qf = sysTemp.LQRPenalty.weight;
      Xf = sysTemp.LQRSet
      [ff] = double(Xf);
      
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
