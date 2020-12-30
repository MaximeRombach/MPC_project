%% Import Casadi
addpath('/Library/gurobi903/mac64/matlab')
import casadi.*

%% To do 3.1
clear all;

Ts = 1/5;
quad = Quad(Ts);
[xs, us] = quad.trim();
sys = quad.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_yaw] = quad.decompose(sys, xs, us);

%% Evaluate Controller z
% Design MPC controller
mpc_z = MPC_Control_z(sys_z, Ts);

sim_time = 15; % seconds
time = 0:Ts:sim_time;
sim_length = length(time); % number of simulation steps 
x_hist = zeros(2,sim_length);
ux_hist = zeros(1,sim_length-1);

x_hist(:,1) = [0;17];% starting position
tic;
for i = 1:sim_length
    % Get control inputs with
    ux = mpc_z.get_u(x_hist(:,i));
    ux_hist(:,i)= ux;
    x_hist(:,i+1) = mpc_z.A*x_hist(:,i) + mpc_z.B*ux_hist(:,i);
end
toc
% determine the settling time 
for i=1:length(x_hist(2,:))
    threshold = 0.01; % (error on the position < threshold in [m])
    if abs(x_hist(2,i)) < threshold
        break;
    end
end

set_time = time(i)
%% Plots for sys_z controller
close all;

% Plot the input
figure();
plot(time,ux_hist(1,:),'.-'); hold on;
plot(time, 0.3*ones(1,sim_length), '--r');
plot(time, -0.2*ones(1,sim_length), '--r');
axis([0 sim_time -0.3 0.4]); grid;
xlabel('time in [s]'); ylabel('input'); title('Z System: Input Applied');
legend('Applied input','Input limits');

% plot the state evolution
figure();
subplot(1,2,1);
plot(time, x_hist(1,1:(sim_length)),'.-'); grid;
xlabel('time in [s]'); ylabel('z Velocity in [m/s]');title('Z Velocity Evolution');

subplot(1,2,2);
plot(time, x_hist(2,1:(sim_length)),'.-'); grid;
xlabel('time in [s]'); ylabel('z in [m]');title('Z Position Evolution');
%% tuning sys_z

% Discretize the system and extract the A,B,C,D matrices
sys_d = c2d(sys_z, Ts);
[A,B,C,D] = ssdata(sys_d);


[n,m] = size(B);
for R = [1]%, 5, 10, 20]
    % Set the LQR controller
    Q = diag([1,1]);%to be tuned

    % State Constraints
    % x in X = { x | Fx <= f } No constraints on the state

    % Input Constraints
    % u in U = { u | Mu <= m }
    M = [1;-1]; m = [0.3;0.2];

    % Compute the unconstrained LQR controller
    [K,Qf,~] = dlqr(A,B,Q,R);
    K=-K; % Matlab inverts the K matrix

    % Compute the terminal set (maximum invariant set under local LQR
    % controller)
    Xf = polytope([M*K],[m]);
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
    Xf.projection(1:2).plot(); title(['Terminal set with R = ', num2str(R)]);
    xlabel('Z velocity [m/s]'); ylabel('Z position [m]');
end