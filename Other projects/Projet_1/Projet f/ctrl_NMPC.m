function [CTRL, traj] = ctrl_NMPC(quad)
        addpath('C:\Users\antoine\Downloads\casadi-windows-matlabR2016a-v3.5.1')
        import casadi.*
        opti = casadi.Opti(); % Optimization problem
        N = 20; % MPC horizon [SET THIS VARIABLE]
        % ???? decision variables ?????????
        X = opti.variable(12,N+1); % state trajectory variables
        U = opti.variable(4, N); % control trajectory (throttle, brake)
        X0 = opti.parameter(12,1); % initial state
        REF = opti.parameter(4,1); % reference position [x,y,z,yaw]
        %%%%%%%%%%%%%%%%%%%%%%%%
        %%%% YOUR CODE HERE %%%%
        %%%%%%%%%%%%%%%%%%%%%%%%
        [~, Us] = quad.trim();

        f_discrete = @(x,u) RK4(x,u,quad.Ts,quad.f(x,u));

        opti.minimize(...
            sum(X(10,:)-REF(1)) + ...
            sum(X(11,:)-REF(2)) + ...
            sum(X(12,:)-REF(3)) + ...
            sum(X(6,:)-REF(4)) + ...
            (U(1,:)-Us(1))*(U(1,:)-Us(1))' + ...
            (U(2,:)-Us(2))*(U(2,:)-Us(2))' + ...
            (U(3,:)-Us(3))*(U(3,:)-Us(3))' + ...
            (U(4,:)-Us(4))*(U(4,:)-Us(4))'); 
    %         -10*sum(X(2,:))  + ... % Max velocity
    %         0.1*U(1,:)*U(1,:)' + ... % Minimize accel
    %         10*U(2,:)*U(2,:)'   + ... % Minimize braking
    %         10000*(epsilon_speed(1,:)*epsilon_speed(1,:)' + sum(epsilon_speed))); % Soft constraints

        for k=1:N % loop over control intervals

          %%%%% REMOVE
          opti.subject_to(X(:,k+1) == f_discrete(X(:,k), U(:,k)));
          %%%%% REMOVE

          %%%% WRITE YOUR DYNAMICS CONSTRAINT HERE
          opti.subject_to(X(:,k+1) == X(:,k) + quad.f(X(:,k),U(:,k)));

        end

        opti.subject_to(0 <= U <= 1.5);  % control is limited
        opti.subject_to(-0.035 <= X(4,:) <= 0.035);
        opti.subject_to(-0.035 <= X(5,:) <= 0.035);


        CTRL = @(x,ref) eval_ctrl(x, ref, opti, X0, REF, X, U);
    end


