classdef MPC_Control_yaw < MPC_Control
  
  methods
    % Design a YALMIP optimizer object that takes a steady-state state
    % and input (xs, us) and returns a control input
    function ctrl_opt = setup_controller(mpc,xs,us)

      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % INPUTS
      %   x(:,1) - initial state (estimate)
      %   xs, us - steady-state target
      % OUTPUTS
      %   u(:,1) - input to apply to the system
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

      [n,m] = size(mpc.B);
      
      % Steady-state targets (Ignore this before Todo 3.2)
%       xs = sdpvar(n, 1);
%       us = sdpvar(m, 1);
%       
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
      A=mpc.A;
      B=mpc.B;
      Q=eye(n);
      R=10;
      Cu=[1;-1];
      cu=[0.2;0.2];
      %compute LQR controller
      [K,Qf,~]=dlqr(A,B,Q,R);
      K=-K;
      %compute maximal invariant set
      ccu=cu-us;
      Xf=polytope(Cu*K,ccu);
      Upd=A+B*K;
      while 1
          prevXf=Xf;
          [W,V]=double(Xf);
          pXf=polytope(W*Upd,V);
          Xf=intersect(Xf,pXf);
          if prevXf==Xf
              break
          end
      end
      [FW,FV]=double(Xf);
      con=[con,x(:,2)==A*x(:,1)+B*u(:,1)];
      con=[con,Cu*u(:,1)<=cu];
      obj=u(:,1)'*R*u(:,1);
      for i=2:1:N-1
          con=[con, x(:,i+1)==A*x(:,i)+B*u(:,i)];
          con=[con,Cu*u(:,i)<=ccu];
          obj=obj+(x(:,i))'*Q*(x(:,i))+(u(:,i)')*R*(u(:,i));
      end
      con=[con,FW*x(:,N)<=FV];
      obj=obj+(x(:,N))'*Qf*(x(:,N));
      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE 
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      
      
      ctrl_opt = optimizer(con, obj, sdpsettings('solver','gurobi'), ...
        {x(:,1)}, u(:,1)+us);
         %plot projected maximal incariant sets of each state
%         figure;
%         subplot(2,2,1);
%         Xf.projection(1:2).plot();
%         xlabel('velocity of yaw');
%         ylabel('yaw');
%         grid on;
%         sgtitle('Terminal maximal invariant set for transformed sys_yaw');
%       
      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE 
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      

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
      A=mpc.A;
      B=mpc.B;
      C=mpc.C;
      R=2;
      Cu=[1;-1];
      cu=[0.2;0.2];
      con=[con,xs==A*xs+B*us];
      con=[con,Cu*us<=cu];
      obj=obj+(C*xs-ref)'*R*(C*xs-ref);
      
      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE 
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      
      
      % Compute the steady-state target
      target_opt = optimizer(con, obj, sdpsettings('solver', 'gurobi'), ref, {xs, us});
      
    end
  end
end
