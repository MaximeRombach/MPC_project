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

%% Evaluate Controller x
% Design MPC controller
mpc_x = MPC_Control_x(sys_x, Ts);

sim_time = 15; % total simulation time in seconds
time = 0:Ts:sim_time;
sim_length = length(time); % number of simulation steps 
x_hist = zeros(4,sim_length);
ux_hist = zeros(1,sim_length-1);

x_hist(:,1) = [0;0;0;2];% initial state 

for i = 1:sim_length
    % Get control inputs with
    ux = mpc_x.get_u(x_hist(:,i));
    ux_hist(:,i)= ux;
    x_hist(:,i+1) = mpc_x.A*x_hist(:,i) + mpc_x.B*ux_hist(:,i);
end

% determine the settling time
for i=1:length(x_hist(2,:))
    threshold = 0.01; % (error on the position < threshold in [m])
    if abs(x_hist(4,i)) < threshold
        if(abs(x_hist(2,i:end)) < threshold)
            break;
        end
    end
end
if i == length(x_hist(2,:))
    fprintf('Does not stabilize over the simulation\n');
else
    fprintf('Settling time: %.1f seconds \n',time(i));
end
%% Plots for sys_x controller

% Plot the input
figure();
plot(time,ux_hist(1,:)); hold on;
plot(time, 0.3*ones(1,sim_length), '--r');
plot(time, -0.3*ones(1,sim_length), '--r');
axis([0 sim_time -0.4 0.4]); grid;
xlabel('time [s]'); ylabel('input'); title('X System: Input Applied');

% Plot the states
figure();

subplot(2,2,1);
plot(time, x_hist(1,1:(sim_length)), '.-'); grid;
xlabel('time [s]'); ylabel('pitch velocity [rad/s]');title('Pitch Velocity Evolution');

subplot(2,2,2);
plot(time, x_hist(2,1:(sim_length)), '.-'); grid; hold on;
plot(time, 0.035*ones(1,sim_length), '--r');
plot(time, -0.035*ones(1,sim_length), '--r');
xlabel('time [s]'); ylabel('pitch angle [rad]');title('Pitch Angle Evolution');

subplot(2,2,3);
plot(time, x_hist(3,1:(sim_length)), '.-'); grid;
xlabel('time [s]'); ylabel('x velocity [m/s]');title('X Velocity Evolution');


subplot(2,2,4);
plot(time, x_hist(4,1:(sim_length)), '.-'); grid;
xlabel('time [s]'); ylabel('x position [m]');title('X Position Evolution');


%% Evaluate Controller y
% Design MPC controller
mpc_y = MPC_Control_y(sys_y, Ts);

sim_time = 15; % total simulation time in seconds
time = 0:Ts:sim_time;
sim_length = length(time); % number of simulation steps 
y_hist = zeros(4,sim_length);
uy_hist = zeros(1,sim_length-1);

y_hist(:,1) = [0;0;0;2];

for i = 1:sim_length
    % Get control inputs with
    uy = mpc_y.get_u(y_hist(:,i));
    uy_hist(:,i)= uy;
    y_hist(:,i+1) = mpc_y.A*y_hist(:,i) + mpc_y.B*uy_hist(:,i);
end

% determine the settling time
for i=1:length(y_hist(4,:))
    threshold = 0.01; % (error on the position < threshold in [rad])
    if abs(y_hist(4,i)) < threshold
        if(abs(y_hist(4,i:end)) < threshold)
            break;
        end
    end
end
if i == length(y_hist(2,:))
    fprintf('Does not stabilize over the simulation\n');
else
    fprintf('Settling time: %.1f seconds \n',time(i));
end

% Plots for sys_y controller
%close all;

% Plot the input
figure();
plot(time,uy_hist(1,:)); hold on;
plot(time, 0.3*ones(1,sim_length), '--r');
plot(time, -0.3*ones(1,sim_length), '--r');
axis([0 sim_time -0.4 0.4]); grid;
xlabel('time [s]'); ylabel('input'); title('Y System: Input Applied');

% plot the states
figure();

subplot(2,2,1);
plot(time, y_hist(1,1:(sim_length)), '.-'); grid;
xlabel('time [s]'); ylabel('roll velocity [rad/s]');title('Roll Velocity Evolution');

subplot(2,2,2);
plot(time, y_hist(2,1:(sim_length)), '.-'); grid;hold on;
plot(time, 0.035*ones(1,sim_length), '--r');
plot(time, -0.035*ones(1,sim_length), '--r');
xlabel('time [s]'); ylabel('roll angle [rad]');title('Roll Angle Evolution');

subplot(2,2,3);
plot(time, y_hist(3,1:(sim_length)), '.-'); grid;
xlabel('time [s]'); ylabel('y velocity [m/s]');title('Y Velocity Evolution');


subplot(2,2,4);
plot(time, y_hist(4,1:(sim_length)), '.-'); grid;
xlabel('time [s]'); ylabel('y position [m]');title('Y Position Evolution');
%% Evaluate Controller z
% Design MPC controller
mpc_z = MPC_Control_z(sys_z, Ts);

sim_time = 15; % seconds
time = 0:Ts:sim_time;
sim_length = length(time); % number of simulation steps 
z_hist = zeros(2,sim_length);
uz_hist = zeros(1,sim_length-1);

z_hist(:,1) = [0;2];% starting state

for i = 1:sim_length
    % Get control inputs with
    uz = mpc_z.get_u(z_hist(:,i));
    uz_hist(:,i)= uz;
    z_hist(:,i+1) = mpc_z.A*z_hist(:,i) + mpc_z.B*uz_hist(:,i);
end

% determine the settling time
for i=1:length(z_hist(2,:))
    threshold = 0.001; % (error on the position < threshold in [rad])
    if abs(z_hist(2,i)) < threshold
        if(abs(z_hist(2,i:end)) < threshold)
            break;
        end
    end
end
if i == length(z_hist(2,:))
    fprintf('Does not stabilize over the simulation\n');
else
    fprintf('Settling time: %.1f seconds \n',time(i));
end
% Plots for sys_z controller
%close all;

% Plot the input
figure();
plot(time,uz_hist(1,:),'.-'); hold on;
plot(time, 0.3*ones(1,sim_length), '--r');
plot(time, -0.2*ones(1,sim_length), '--r');
axis([0 sim_time -0.3 0.4]); grid;
xlabel('time [s]'); ylabel('input'); title('Z System: Input Applied');
%legend('Applied input','Input limits');

% plot the state evolution
figure();
subplot(1,2,1);
plot(time, z_hist(1,1:(sim_length)),'.-'); grid;
xlabel('time [s]'); ylabel('z velocity [m/s]');title('Z Velocity Evolution');

subplot(1,2,2);
plot(time, z_hist(2,1:(sim_length)),'.-'); grid;
xlabel('time [s]'); ylabel('z position [m]');title('Z Position Evolution');
%% tuning sys_z

% Discretize the system and extract the A,B,C,D matrices
sys_d = c2d(sys_z, Ts);
[A,B,C,D] = ssdata(sys_d);


[n,m] = size(B);

% Set the LQR controller
Q = diag([1,1]);%to be tuned

f1 = figure;  %title(['Feasible sets with different R values']);
f2 = figure; %title(['Terminal set with different R values']);
colors = ['r','c','y'];
R_values = [100 10 1];
for j=1:length(R_values)
    
    R = R_values(j);
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
    [Ff, ff] = double(Xf);% terminal set

    % Plot of the terminal set
    figure(f1);
    Xf.projection(1:2).plot('alpha', 1, 'color', colors(j)); hold on; title(['Terminal set with different R values']);
    xlabel('Z velocity [m/s]'); ylabel('Z position [m]');

    lin = size(M,1);
    col = size(A, 2);
    N=15;
    S = Xf;
    for i=1:N-1
        preS = S;
        [Fs, fs] = double(preS);
        Su = polytope([Fs*A Fs*B; zeros(lin, col) M],[fs; m]);
        S = Su.projection(1:2);
        
        %fprintf('Controlled Invariant set with N = %d\n', i);
        %pause;
    end
    % Plot of the feasible sets
    figure(f2); 
    S.plot('alpha', 1 , 'color', colors(j)); hold on; title(['Feasible initial states for different R values']);
    xlabel('Z velocity [m/s]'); ylabel('Z position [m]');
end
figure(f1); legend('R = 100','R = 10','R = 1');
figure(f2); legend('R = 100','R = 10','R = 1');

%% Evaluate Controller yaw
% Design MPC controller
mpc_yaw = MPC_Control_yaw(sys_yaw, Ts);

sim_time = 15; % seconds
time = 0:Ts:sim_time;
sim_length = length(time); % number of simulation steps 
yaw_hist = zeros(2,sim_length);
uyaw_hist = zeros(1,sim_length-1);

yaw_hist(:,1) = [0;pi/4];

for i = 1:sim_length
    % Get control inputs with
    uyaw = mpc_yaw.get_u(yaw_hist(:,i));
    uyaw_hist(:,i)= uyaw;
    yaw_hist(:,i+1) = mpc_yaw.A*yaw_hist(:,i) + mpc_yaw.B*uyaw_hist(:,i);
end

% determine the settling time
for i=1:length(yaw_hist(2,:))
    threshold = pi/180/10; % (error on the position < threshold in [rad])
    if abs(yaw_hist(2,i)) < threshold
        if(abs(yaw_hist(2,i:end)) < threshold)
            break;
        end
    end
end
if i == length(yaw_hist(2,:))
    fprintf('Does not stabilize over the simulation\n');
else
    fprintf('Settling time: %.1f seconds \n',time(i));
end

% Plots for sys_yaw controller
%close all;

% Plot the input
figure();
plot(time,uyaw_hist(1,:)); hold on;
plot(time, 0.2*ones(1,sim_length), '--r');
plot(time, -0.2*ones(1,sim_length), '--r');
axis([0 sim_time -0.3 0.4]); grid;
xlabel('time [s]'); ylabel('input'); title('Yaw System: Input Applied');

% plot the states
figure();

subplot(1,2,1);
plot(time, yaw_hist(1,1:(sim_length)),'.-'); grid;
xlabel('time [s]'); ylabel('yaw velocity [rad/s]');
title('Yaw Velocity Evolution');

subplot(1,2,2);
plot(time, yaw_hist(2,1:(sim_length)),'.-'); grid;
xlabel('time [s]'); ylabel('yaw angle [rad]');
title('Yaw Angle Evolution');
%% tuning sys_yaw

% Discretize the system and extract the A,B,C,D matrices
sys_d = c2d(sys_yaw, Ts);
[A,B,C,D] = ssdata(sys_d);


[n,m] = size(B);

% Set the LQR controller
Q = diag([1,1]);%to be tuned

f1 = figure;  %title(['Feasible sets with different R values']);
f2 = figure; %title(['Terminal set with different R values']);
colors = ['r','c','y'];
R_values = [100 10 1];

for j= 1:length(R_values)
    R = R_values(j);

    % State Constraints
    % x in X = { x | Fx <= f } No constraints on the state

    % Input Constraints
    % u in U = { u | Mu <= m }
    M = [1;-1]; m = [0.2;0.2];

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
    [Ff, ff] = double(Xf);% terminal set

     % Plot of the terminal set
    figure(f1);
    Xf.projection(1:2).plot('alpha', 0.5, 'color', colors(j)); hold on; title(['Terminal set with different R values']);
    xlabel('Yaw velocity [rad/s]'); ylabel('Yaw angle [rad]');

    lin = size(M,1);
    col = size(A, 2);
    N=15;
    S = Xf;
    for i=1:N-1
        preS = S;
        [Fs, fs] = double(preS);
        Su = polytope([Fs*A Fs*B; zeros(lin, col) M],[fs; m]);
        S = Su.projection(1:2);
        %S.plot('alpha',0.1);
        %fprintf('Controlled Invariant set with N = %d\n', i);
        %pause;
    end

    % Plot of the feasible sets
    figure(f2); 
    S.plot('alpha',0.5 , 'color', colors(j)); hold on; title(['Feasible sets with different R values']);
    xlabel('Yaw velocity [rad/s]'); ylabel('Yaw angle [rad]');
    
end

figure(f1); legend('R = 100','R = 10','R = 1');
figure(f2); legend('R = 100','R = 10','R = 1');