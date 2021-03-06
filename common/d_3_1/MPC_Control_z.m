classdef MPC_Control_z < MPC_Control
  properties
    A_bar, B_bar, C_bar % Augmented system for disturbance rejection    
    L                   % Estimator gain for disturbance rejection
  end
  
  methods
    function mpc = MPC_Control_z(sys, Ts)
      mpc = mpc@MPC_Control(sys, Ts);
      
      [mpc.A_bar, mpc.B_bar, mpc.C_bar, mpc.L] = mpc.setup_estimator();
    end
    
    % Design a YALMIP optimizer object that takes a steady-state state
    % and input (xs, us) and returns a control input
    function ctrl_opt = setup_controller(mpc)

      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % INPUTS
      %   x(:,1) - initial state (estimate)
      %   d_est  - disturbance estimate
      %   xs, us - steady-state target
      % OUTPUTS
      %   u(:,1) - input to apply to the system
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

      [n,m] = size(mpc.B);
      
      % Steady-state targets (Ignore this before Todo 3.3)
      xs = sdpvar(n, 1);
      us = sdpvar(m, 1);
      
      % Disturbance estimate (Ignore this before Part 5)
      d_est = sdpvar(1);

      % SET THE HORIZON HERE
      N = 15;
      
      % Predicted state and input trajectories
      x = sdpvar(n, N);
      u = sdpvar(m, N-1);
      

      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE 

      % NOTE: The matrices mpc.A, mpc.B, mpc.C and mpc.D are 
      %       the DISCRETE-TIME MODEL of your system

      % SET THE PROBLEM CONSTRAINTS con AND THE OBJECTIVE obj HERE
      con = [];
      obj = 0;
      A=mpc.A;
      B=mpc.B;
      Q=1*eye(n);
      R=100;
      Cu=[1;-1];
      cu=[0.3;0.2];

      %compute LQR controller
      [K,Qf,~]=dlqr(A,B,Q,R);
      K=-K;
      %compute maximal invariant set
      Xf=polytope(Cu*K,cu);
      Upd=A+B*K;
      fprintf("Computing max invariant set for Z\n")
      k = 1;
      while 1
          prevXf=Xf;
          [W,V]=double(Xf);
          pXf=polytope(W*Upd,V);
          Xf=intersect(Xf,pXf);
          if prevXf==Xf
              fprintf("Finished at iteration n�%d\n", k)
              break
          end
          fprintf("Iteration n�%d \n", k)
          k = k + 1;
      end
      [FW,FV]=double(Xf);
      con=[con,x(:,2)==A*x(:,1)+B*u(:,1)];
      con=[con,Cu*u(:,1)<=cu];
      obj=u(:,1)'*R*u(:,1);
      for i=2:1:N-1
          con=[con, x(:,i+1)==A*x(:,i)+B*u(:,i)];
          con=[con,Cu*u(:,i)<=cu];
          obj=obj+(x(:,i))'*Q*(x(:,i))+(u(:,i)')*R*(u(:,i));
      end
      con=[con,FW*x(:,N)<=FV];
      obj=obj+(x(:,N))'*Qf*(x(:,N));
      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE 
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      
      
      ctrl_opt = optimizer(con, obj, sdpsettings('solver','gurobi'), ...
        {x(:,1), xs, us}, u(:,1));
    
        %plot projected maximal invariant sets of each state
        figure;
%<<<<<<< HEAD
        Xf.plot();
        xlabel('z velocity [m/s]');ylabel('z position [m]');grid on;
        sgtitle('Terminal set for the z system');
%=======
        Xf.projection(1:2).plot();
        xlabel('velocity of z');
        ylabel('z');
        grid on;
        sgtitle('Terminal maximal invariant set for sys z');
%>>>>>>> 8d2ee10736851a31a442d0056596de41cf45a55a
      
      
      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE 
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      
      
      ctrl_opt = optimizer(con, obj, sdpsettings('solver','gurobi'), ...
        {x(:,1), xs, us, d_est}, u(:,1));
    end
    
    
    % Design a YALMIP optimizer object that takes a position reference
    % and returns a feasible steady-state state and input (xs, us)
    function target_opt = setup_steady_state_target(mpc)
      
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % INPUTS
      %   ref    - reference to track
      %   d_est  - disturbance estimate
      % OUTPUTS
      %   xs, us - steady-state target
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

      % Steady-state targets
      n = size(mpc.A,1);
      xs = sdpvar(n, 1);
      us = sdpvar;
      
      % Reference position (Ignore this before Todo 3.3)
      ref = sdpvar;
            
      % Disturbance estimate (Ignore this before Part 5)
      d_est = sdpvar(1);
      
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE 
      % You can use the matrices mpc.A, mpc.B, mpc.C and mpc.D
      con = [];
      obj = 0;
      

      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE 
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      
      
      % Compute the steady-state target
      target_opt = optimizer(con, obj, sdpsettings('solver', 'gurobi'), {ref, d_est}, {xs, us});
    end
    
    
    % Compute augmented system and estimator gain for input disturbance rejection
    function [A_bar, B_bar, C_bar, L] = setup_estimator(mpc)
      
      %%% Design the matrices A_bar, B_bar, L, and C_bar
      %%% so that the estimate x_bar_next [ x_hat; disturbance_hat ]
      %%% converges to the correct state and constant input disturbance
      %%%   x_bar_next = A_bar * x_bar + B_bar * u + L * (C_bar * x_bar - y);
      
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE 
      % You can use the matrices mpc.A, mpc.B, mpc.C and mpc.D
      
      A_bar = [];
      B_bar = [];
      C_bar = [];
      L = [];
      
      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE 
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    end

    
  end
end
