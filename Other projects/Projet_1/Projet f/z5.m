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
      N = 20;
      
      % Predicted state and input trajectories
      x = sdpvar(n, N);
      u = sdpvar(m, N-1);
      

      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE 

      % NOTE: The matrices mpc.A, mpc.B, mpc.C and mpc.D are 
      %       the DISCRETE-TIME MODEL of your system

      % SET THE PROBLEM CONSTRAINTS con AND THE OBJECTIVE obj HERE
            A=mpc.A; B=mpc.B; C=mpc.C; D=mpc.D;
      %define the constraint on x and u
      %constraint of V = {v | Mv <= m}
%       M = [1;-1;1;-1;1;-1;1;-1]; m = [0.3;0.3;0.3;0.3;0.3;0.2;0.2;0.2];
M=[1;-1]; m=[0.2;0.2];
      % X = {x | Fx <= f}
      F=zeros(2*n,n); f=zeros(2*n,1); 
      Q=diag([0.1;1]);
      R=0.01;
      % Compute optimal LQR controller for unconstrained system
      [K,Qf,~] = dlqr(A,B,Q,R);
      K = -K; % Matlab defines K as -K

%         % Compute maximal invariant set
%         Xf = polytope([F;M*K],[f;m]);
%         Acl = [A+B*K];
%         while 1
%             prevXf = Xf;
%             [T,t] = double(Xf);
%             preXf = polytope(T*Acl,t);
%             Xf = intersect(Xf, preXf);
%             if isequal(prevXf, Xf)
%                 break
%             end
%         end
%         [Ff,ff] = double(Xf);
%       figure(7)
%       plot(Xf)
      
       con = (x(:,2) == A*(x(:,1)-xs) + B*(u(:,1)-us)+B*d_est) + (M*u(:,1) <= m);
       obj = (u(:,1)-us)'*R*(u(:,1)-us);
    for i = 2:N-1
       con = con + (x(:,i+1) == A*(x(:,i)) + B*u(:,i)+B*d_est);
       con = con + (M*u(:,i) <= m);
       obj = obj + x(:,i)'*Q*x(:,i) + (u(:,i)-us)'*R*(u(:,i)-us);
    end
       con = con ;
       obj = obj + x(:,N)'*Qf*x(:,N);
  

      
      
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
            con = [0 <= us <= 1.5 ,...
                xs == mpc.A*xs + mpc.B*us + mpc.B*d_est   ,...
                ref == mpc.C*xs      ];
      obj = us^2;
      

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
       A=mpc.A; B=mpc.B; C=mpc.C; D=mpc.D;
       nx   = size(A,1);
nu   = size(B,2);
ny   = size(C,1);
 A_bar = [A          ,B;
         zeros(1,nx),1          ];
B_bar = [B;zeros(1,nu)];
C_bar = [C,zeros(ny,1)];
      L = -place(A_bar',C_bar',[0.5,0.6,0.7])';
      
      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE 
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    end

    
  end
end
