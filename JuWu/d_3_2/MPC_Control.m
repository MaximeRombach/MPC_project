classdef MPC_Control
  properties
    ctrl_opt % YALMIP object to compute control law
    target_opt % YALMIP object to compute steady-state target
    % Discrete-time system matrices
    A, B, C, D
  end
  
  methods
    function mpc = MPC_Control(sys, Ts)
      
      % Discretize the system and extract the A,B,C,D matrices
      sys_d = c2d(sys, Ts);
      [mpc.A,mpc.B,mpc.C,mpc.D] = ssdata(sys_d);
      
%       mpc.target_opt = mpc.setup_steady_state_target();
%       mpc.ctrl_opt = mpc.setup_controller();
    end
    
    % Compute the MPC controller
    function u = get_u(mpc, x, ref)
        mpc.target_opt=mpc.setup_steady_state_target();
        [target,isfeasible]=mpc.target_opt(ref);
        assert(isfeasible==0, 'isfeasible in target computationn');
        [xs,us] = deal(target{:});
        mpc.ctrl_opt=mpc.setup_controller(xs,us);
        [u, isfeasible] = mpc.ctrl_opt(x-xs);
        assert(isfeasible==0, 'isfeasible in control computationn');
    end
  end
  
  methods (Abstract)
    setup_controller(mpc)
    setup_steady_state_target(mpc)
  end
end
