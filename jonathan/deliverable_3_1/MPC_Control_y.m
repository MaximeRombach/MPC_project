classdef MPC_Control_y < MPC_Control
  
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
      

      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE 

      % NOTE: The matrices mpc.A, mpc.B, mpc.C and mpc.D are 
      %       the DISCRETE-TIME MODEL of your system

      % WRITE THE CONSTRAINTS AND OBJECTIVE HERE
      con = [];
      obj = 0;
      
      A = mpc.A; B = mpc.B;
      
      Q = diag([1,1,1,1]);%to be tuned
      R = 8;%to be tuned when using 10 the terminal set projected on 1:2 disappears!!!
      
      % State Constraints
      % x in X = { x | Fx <= f }
      F = [0,1,0,0; 0,-1,0,0]; f = 0.035*[1;1];
      
      % Input Constraints
      % u in U = { u | Mu <= m }
      M = [1;-1]; m = 0.3*[1;1];
      
      % Compute the unconstrained LQR controller
      [K,Qf,~] = dlqr(A,B,Q,R);
      K=-K; % Matlab inverts the K matrix
      
      % Compute the terminal set (maximum invariant set under local LQR
      % controller)
      Xf = polytope([F;M*K],[f;m]);
      Acl = [A+B*K];
      while 1
          prevXf = Xf;
          [T,t]= double(Xf);
          preXf = polytope(T*Acl,t);
          Xf = intersect(Xf, preXf);
          if isequal(prevXf, Xf)
              break
          end
      end
      [Ff, ff] = double(Xf);
      
      % Plot of the terminal set
      figure();
      subplot(1,3,1);
      Xf.projection(1:2).plot(); title('Terminal set projected on states 1 & 2');
      xlabel('roll velocity [rad/s]'); ylabel('roll [rad]');
      subplot(1,3,2);
      Xf.projection(2:3).plot(); title('Terminal set projected on states 2 & 3');
      xlabel('roll [rad]'); ylabel('y velocity [m/s]');
      subplot(1,3,3);
      Xf.projection(3:4).plot(); title('Terminal set projected on states 3 & 4');
      xlabel('y velocity [m/s]'); ylabel('y position [m]');
      
      % Definition of the constraints and objective 
      %(no condition on the initial state)
      con = [con, x(:,2) == A*x(:,1) + B*u(:,1)];
      con = [con, M*u(:,1) <= m];
      obj = obj + u(:,1)'*R*u(:,1) ;
      for i = 2:(N-1)
          con = [con, x(:,i+1) == A*x(:,i) + B*u(:,i)];
          con = [con, F*x(:,i) <= f];
          con = [con, M*u(:,i) <= m];
          obj = obj + x(:,i)'*Q*x(:,i) + u(:,i)'*R*u(:,i);
      end
      con = [con, x(:,N) == A*x(:,N-1) + B*u(:,N-1)];
      con = [con, Ff*x(:,N) <= ff]; % terminal constraint 
      obj = x(:,N)'*Qf*x(:,N); % terminal cost

      
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
