classdef MPC_Control_x < MPC_Control
  
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
      

      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE 

      % NOTE: The matrices mpc.A, mpc.B, mpc.C and mpc.D are 
      %       the DISCRETE-TIME MODEL of your system

      % WRITE THE CONSTRAINTS AND OBJECTIVE HERE
      con = [];
      obj = 0;
      %extract system matrics
      A=mpc.A;
      B=mpc.B;
      Q=diag([1, 1, 1 , 1]);%eye(n);
      R=9;
      Cu=[1;-1];
      cu=[0.3;0.3];
      Fx=[0,1,0,0;0,-1,0,0];
      fx=[0.035;0.035];
      %compute LQR controller
      [K,Qf,~]=dlqr(A,B,Q,R);
      K=-K;
      %compute maximal invariant set
      Xf=polytope([Fx;Cu*K],[fx;cu]);
      Upd=A+B*K;
      fprintf("Computing max invariant set for X\n")
      k = 1;
      while 1
          prevXf=Xf;
          [W,V]=double(Xf);
          pXf=polytope(W*Upd,V);
          Xf=intersect(Xf,pXf);
          if prevXf==Xf
              fprintf("Finished at iteration n°%d\n", k)
              break
          end
          fprintf("Iteration n°%d \n", k)
          k = k + 1;
      end
      [FW,FV]=double(Xf);
      con=[con,x(:,2)==A*x(:,1)+B*u(:,1)];
      con=[con,Cu*u(:,1)<=cu];
      obj=u(:,1)'*R*u(:,1);
      for i=2:1:N-1
          con=[con, x(:,i+1)==A*x(:,i)+B*u(:,i)];
          con=[con,Fx*x(:,i)<=fx];
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
        subplot(1,3,1);
        Xf.projection(1:2).plot();
        xlabel('pitch velocity [rad/s]');
        ylabel('pitch angle [rad]');
        grid on;
        subplot(1,3,2);
        Xf.projection(2:3).plot();
        xlabel('pitch angle [rad]');
        ylabel('x velocity');
        grid on;
        subplot(1,3,3);
        Xf.projection(3:4).plot();
%<<<<<<< HEAD
        xlabel('x velocity [m/s]');
        ylabel('x position [m]');
        sgtitle('Terminal set for the x system');
%=======
        xlabel('velocity of x');
        ylabel('x');
        sgtitle('Terminal maximal invariant set for sys x');
%>>>>>>> 8d2ee10736851a31a442d0056596de41cf45a55a
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
