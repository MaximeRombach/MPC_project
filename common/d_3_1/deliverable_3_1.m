clc;
clear all;
close all;
%model setup
quad = Quad();
[xs,us] = quad.trim();
sys = quad.linearize(xs, us);
sys_transformed = sys * inv(quad.T);
[sys_x, sys_y, sys_z, sys_yaw] = quad.decompose(sys, xs, us);
Ts=0.2;
%controller construction
save = false;
mpc_x = MPC_Control_x(sys_x, Ts);
save_plot('del_31_x', save)
mpc_y = MPC_Control_y(sys_y, Ts);
save_plot('del_31_y', save)
mpc_z = MPC_Control_z(sys_z, Ts);
save_plot('del_31_z', save)
mpc_yaw = MPC_Control_yaw(sys_yaw, Ts);
save_plot('del_31_yaw', save)
ctrl_n_ref= quad.merge_controllers_non_ref(mpc_x, mpc_y, mpc_z, mpc_yaw);
%% simulate the 4 subsystems simultaneously
ind = quad.ind;
I_x = [ind.omega(2),ind.theta(2),ind.vel(1),ind.pos(1)];
I_y = [ind.omega(1),ind.theta(1),ind.vel(2),ind.pos(2)];
I_z = [ind.vel(3),ind.pos(3)];
I_yaw = [ind.omega(3),ind.theta(3)];
% initial with [x,y,z]=[2,2,2]
Tt = 8;
nbSteps = ceil(Tt/Ts);
x0 = zeros(12,1); 
x0(I_yaw(2)) = pi/4;
x0([I_x(4),I_y(4),I_z(2)]) = 2;

v = zeros(4,nbSteps-1);
u = zeros(4,nbSteps-1);
X= zeros(12,nbSteps);
X(:,1) = x0;
% create simulation with control action 
for i=1:1:nbSteps-1
    u(:,i)  = ctrl_n_ref(X(:,i));
    X(:,i+1) = quad.step(X(:,i), u(:,i), Ts);
end
tt = (0:1:nbSteps-1)*Ts;
u(:,nbSteps)  = ctrl_n_ref(X(:,nbSteps));
%visualize the simulation result
sim.x = tt';
sim.y = X;
quad.plot(sim,u);


%% Evaluate Controller x independently
% Design MPC controller
mpc_x = MPC_Control_x(sys_x, Ts);

sim_time = 15; % total simulation time in seconds
time = 0:Ts:sim_time;
sim_length = length(time); % number of simulation steps 
x_hist = zeros(4,sim_length); % states history
ux_hist = zeros(1,sim_length-1); % inputs applied

x_hist(:,1) = [0;0;0;2];% initial state (2 meters away from the origin)

% simulation loop
for i = 1:sim_length
    % Get control input
    ux = mpc_x.get_u(x_hist(:,i));
    ux_hist(:,i)= ux;
    % apply the input
    x_hist(:,i+1) = mpc_x.A*x_hist(:,i) + mpc_x.B*ux_hist(:,i);
end

% determine the settling time
for i=1:length(x_hist(2,:))
    threshold = 0.01; % defines the error band in [m] (threshold = 1 cm)
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
plot(time,ux_hist(1,:), '.-'); hold on;
plot(time, 0.3*ones(1,sim_length), '--r');
plot(time, -0.3*ones(1,sim_length), '--r');
axis([0 sim_time -0.4 0.4]); grid;
xlabel('time [s]'); ylabel('input'); title('X System: Input Applied');

% Plot the states evolution
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


%% Evaluate Controller y independently
% Design MPC controller
mpc_y = MPC_Control_y(sys_y, Ts);

sim_time = 15; % total simulation time in seconds
time = 0:Ts:sim_time;
sim_length = length(time); % number of simulation steps 
y_hist = zeros(4,sim_length); % states history
uy_hist = zeros(1,sim_length-1); % inputs applied

y_hist(:,1) = [0;0;0;2];% initial state (2 meters away from the origin)

% simulation loop
for i = 1:sim_length
    % Get control inputs with
    uy = mpc_y.get_u(y_hist(:,i));
    uy_hist(:,i)= uy;
    y_hist(:,i+1) = mpc_y.A*y_hist(:,i) + mpc_y.B*uy_hist(:,i);
end

% determine the settling time
for i=1:length(y_hist(4,:))
    threshold = 0.01; % defines the error band in [m] (threshold = 1 cm)
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

%% Plots for sys_y controller


% plot the inputs
figure();
plot(time,uy_hist(1,:), '.-'); hold on;
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
%% Evaluate Controller z independently
% Design MPC controller
mpc_z = MPC_Control_z(sys_z, Ts);

sim_time = 15; % seconds
time = 0:Ts:sim_time;
sim_length = length(time); % number of simulation steps 
z_hist = zeros(2,sim_length);
uz_hist = zeros(1,sim_length-1);

z_hist(:,1) = [0;2];% starting state (2 meters away from the origin)

% simulation loop
for i = 1:sim_length
    % Get control inputs with
    uz = mpc_z.get_u(z_hist(:,i));
    uz_hist(:,i)= uz;
    % apply the input
    z_hist(:,i+1) = mpc_z.A*z_hist(:,i) + mpc_z.B*uz_hist(:,i);
end

% determine the settling time
for i=1:length(z_hist(2,:))
    threshold = 0.001;% defines the error band in [m] (threshold = 1 mm)
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
%% Plots for sys_z controller

% Plot the input
figure();
plot(time,uz_hist(1,:),'.-'); hold on;
plot(time, 0.3*ones(1,sim_length), '--r');
plot(time, -0.2*ones(1,sim_length), '--r');
axis([0 sim_time -0.3 0.4]); grid;
xlabel('time [s]'); ylabel('input'); title('Z System: Input Applied');

% plot the state evolution
figure();
subplot(1,2,1);
plot(time, z_hist(1,1:(sim_length)),'.-'); grid;
xlabel('time [s]'); ylabel('z velocity [m/s]');title('Z Velocity Evolution');

subplot(1,2,2);
plot(time, z_hist(2,1:(sim_length)),'.-'); grid;
xlabel('time [s]'); ylabel('z position [m]');title('Z Position Evolution');
%% tuning sys_z: compute the region of attraction

% Discretize the system and extract the A,B,C,D matrices
sys_d = c2d(sys_z, Ts);
[A,B,C,D] = ssdata(sys_d);


[n,m] = size(B);

% Set the LQR controller
Q = diag([1,1]);

f1 = figure;
f2 = figure;
colors = ['r','c','y'];
R_values = [100 10 1];
for j=1:length(R_values)
    
    R = R_values(j);

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
    Xf.projection(1:2).plot('alpha', 1, 'color', colors(j)); hold on; 
    title(['Terminal set with different R values']);
    xlabel('Z velocity [m/s]'); ylabel('Z position [m]');
    
    % compute the region of attraction
    lin = size(M,1);
    col = size(A, 2);
    N=15;
    S = Xf;
    for i=1:N-1
        preS = S;
        [Fs, fs] = double(preS);
        Su = polytope([Fs*A Fs*B; zeros(lin, col) M],[fs; m]);
        S = Su.projection(1:2);
        
    end
    % Plot of the feasible sets
    figure(f2); 
    S.plot('alpha', 1 , 'color', colors(j)); hold on; title(['Region of attraction for different R values']);
    xlabel('Z velocity [m/s]'); ylabel('Z position [m]');
end
figure(f1); legend('R = 100','R = 10','R = 1');
figure(f2); legend('R = 100','R = 10','R = 1');

%% Evaluate Controller yaw independently
% Design MPC controller
mpc_yaw = MPC_Control_yaw(sys_yaw, Ts);

sim_time = 15; % seconds
time = 0:Ts:sim_time;
sim_length = length(time); % number of simulation steps 
yaw_hist = zeros(2,sim_length);
uyaw_hist = zeros(1,sim_length-1);

yaw_hist(:,1) = [0;pi/4]; % initial state (yaw angle = 45 deg)

% simulation loop
for i = 1:sim_length
    % Get control inputs with
    uyaw = mpc_yaw.get_u(yaw_hist(:,i));
    uyaw_hist(:,i)= uyaw;
    % apply the input
    yaw_hist(:,i+1) = mpc_yaw.A*yaw_hist(:,i) + mpc_yaw.B*uyaw_hist(:,i);
end

% determine the settling time
for i=1:length(yaw_hist(2,:))
    threshold = pi/180/10; % defines the error band in [rad] (threshold = 0.1 deg)
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

%% Plots for sys_yaw controller

% Plot the input
figure();
plot(time,uyaw_hist(1,:), '.-'); hold on;
plot(time, 0.2*ones(1,sim_length), '--r');
plot(time, -0.2*ones(1,sim_length), '--r');
axis([0 sim_time -0.3 0.4]); grid;
xlabel('time [s]'); ylabel('input'); title('Yaw System: Input Applied');

% plot the states evolution
figure();

subplot(1,2,1);
plot(time, yaw_hist(1,1:(sim_length)),'.-'); grid;
xlabel('time [s]'); ylabel('yaw velocity [rad/s]');
title('Yaw Velocity Evolution');

subplot(1,2,2);
plot(time, yaw_hist(2,1:(sim_length)),'.-'); grid;
xlabel('time [s]'); ylabel('yaw angle [rad]');
title('Yaw Angle Evolution');
%% tuning sys_yaw: compute the region of attraction

% Discretize the system and extract the A,B,C,D matrices
sys_d = c2d(sys_yaw, Ts);
[A,B,C,D] = ssdata(sys_d);


[n,m] = size(B);

% Set the LQR controller
Q = diag([1,1]);

f1 = figure;
f2 = figure;
colors = ['r','c','y'];
R_values = [100 10 1];

for j= 1:length(R_values)
    R = R_values(j);

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
    Xf.projection(1:2).plot( 'color', colors(j)); hold on; title(['Terminal set with different R values']);
    xlabel('Yaw velocity [rad/s]'); ylabel('Yaw angle [rad]');
    
    % compute the region of attraction
    lin = size(M,1);
    col = size(A, 2);
    N=15;
    S = Xf;
    for i=1:N-1
        preS = S;
        [Fs, fs] = double(preS);
        Su = polytope([Fs*A Fs*B; zeros(lin, col) M],[fs; m]);
        S = Su.projection(1:2);
    end

    % Plot of the region of attraction
    figure(f2); 
    S.plot('color', colors(j)); hold on; title(['Region of attraction with different R values']);
    xlabel('Yaw velocity [rad/s]'); ylabel('Yaw angle [rad]');
    
end

figure(f1); legend('R = 100','R = 10','R = 1');
figure(f2); legend('R = 100','R = 10','R = 1');