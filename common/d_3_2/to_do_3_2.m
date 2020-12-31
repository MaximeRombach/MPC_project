%% Import Casadi
addpath('/Library/gurobi903/mac64/matlab')
import casadi.*

%% To do 3.2
clear all;

Ts = 1/5;
quad = Quad(Ts);
[xs, us] = quad.trim();
sys = quad.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_yaw] = quad.decompose(sys, xs, us);

% Design MPC controller
mpc_x = MPC_Control_x(sys_x, Ts);
mpc_y = MPC_Control_y(sys_y, Ts);
mpc_z = MPC_Control_z(sys_z, Ts);
mpc_yaw = MPC_Control_yaw(sys_yaw, Ts);

%% Simulation for x system

sim_time = 10; % seconds
time = 0:Ts:sim_time;
sim_length = length(time); % number of simulation steps 
x_hist = zeros(4,sim_length);
ux_hist = zeros(1,sim_length-1);

x_hist(:,1) = [0;0;0;0];% x0
x_position_reference = -2;

for i = 1:sim_length
    fprintf('simulation step %d/%d\n', i, sim_length);
    % Get control inputs with
    ux = mpc_x.get_u(x_hist(:,i), x_position_reference);
    ux_hist(:,i)= ux;
    x_hist(:,i+1) = mpc_x.A*x_hist(:,i) + mpc_x.B*ux_hist(:,i);
end

%% Plots for sys_x controller
%close all;

% determine the settling time
for i=1:length(x_hist(4,:))
    threshold = 0.01; % (error on the position < threshold in [m])
    if abs(x_hist(4,i)-x_position_reference) < threshold
        if(abs(x_hist(4,i:end)-x_position_reference) < threshold)
            break;
        end
    end
end
if i == length(x_hist(4,:))
    fprintf('Does not stabilize over the simulation\n');
else
    fprintf('Settling time: %.1f seconds \n',time(i));
end

% Plot the input
figure();
plot(time,ux_hist(1,:)); hold on;
plot(time, 0.3*ones(1,sim_length), '--r');
plot(time, -0.3*ones(1,sim_length), '--r');
axis([0 sim_time -0.35 0.35]); grid;
xlabel('time [s]'); ylabel('input'); title('X System: Input Applied');
legend('input','limit');

% plot the states
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
plot(time, x_hist(4,1:(sim_length)), '.-'); hold on; grid;
plot(time, x_position_reference*ones(1,sim_length), '--k');
xlabel('time [s]'); ylabel('x position [m]');title('X Position Evolution');
axis([0 sim_time (min(x_hist(4,:))-0.2) (max(x_hist(4,:))+0.2)]);

%% Simulation for y system

sim_time = 10; % seconds
time = 0:Ts:sim_time;
sim_length = length(time); % number of simulation steps 
y_hist = zeros(4,sim_length);
uy_hist = zeros(1,sim_length-1);

y_hist(:,1) = [0;0;0;0];% y0 (initial state)
y_position_reference = -2;

for i = 1:sim_length
    fprintf('simulation step %d/%d\n', i, sim_length);
    % Get control inputs with
    uy = mpc_y.get_u(y_hist(:,i), y_position_reference);
    uy_hist(:,i)= uy;
    y_hist(:,i+1) = mpc_y.A*y_hist(:,i) + mpc_y.B*uy_hist(:,i);
end

%% Plots for sys_y controller
%close all;

% determine the settling time
for i=1:length(y_hist(4,:))
    threshold = 0.01; % (error on the position < threshold in [m])
    if abs(y_hist(4,i)-y_position_reference) < threshold
        if(abs(y_hist(4,i:end)-y_position_reference) < threshold)
            break;
        end
    end
end
if i == length(y_hist(4,:))
    fprintf('Does not stabilize over the simulation\n');
else
    fprintf('Settling time: %.1f seconds \n',time(i));
end

% Plot the input
figure();
plot(time,uy_hist(1,:)); hold on;
plot(time, 0.3*ones(1,sim_length), '--r');
plot(time, -0.3*ones(1,sim_length), '--r');
axis([0 sim_time -0.35 0.35]); grid;
xlabel('time [s]'); ylabel('input'); title('Y System: Input Applied');
legend('input','limit');

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
plot(time, y_hist(4,1:(sim_length)), '.-'); hold on; grid;
plot(time, y_position_reference*ones(1,sim_length), '--k');
xlabel('time [s]'); ylabel('y position [m]');title('Y Position Evolution');
axis([0 sim_time (min(y_hist(4,:))-0.2) (max(y_hist(4,:))+0.2)]);

%% Simulation for z system

sim_time = 10; % seconds
time = 0:Ts:sim_time;
sim_length = length(time); % number of simulation steps 
z_hist = zeros(2,sim_length);
uz_hist = zeros(1,sim_length-1);

z_hist(:,1) = [0;0];% z0 (initial state)
z_position_reference = -2;

for i = 1:sim_length
    fprintf('simulation step %d/%d\n', i, sim_length);
    % Get control inputs with
    uz = mpc_z.get_u(z_hist(:,i), z_position_reference);
    uz_hist(:,i)= uz;
    z_hist(:,i+1) = mpc_z.A*z_hist(:,i) + mpc_z.B*uz_hist(:,i);
end
%% Plots for sys_z controller
%close all;

% determine the settling time
for i=1:length(z_hist(2,:))
    threshold = 0.001; % (error on the position < threshold in [m])
    if abs(z_hist(2,i)-z_position_reference) < threshold
        if(abs(z_hist(2,i:end)-z_position_reference) < threshold)
            break;
        end
    end
end
if i == length(z_hist(2,:))
    fprintf('Does not stabilize over the simulation\n');
else
    fprintf('Settling time: %.1f seconds \n',time(i));
end

% Plot the input
figure();
plot(time,uz_hist(1,:),'.-'); hold on;
plot(time, 0.3*ones(1,sim_length), '--r');
plot(time, -0.2*ones(1,sim_length), '--r');
axis([0 sim_time -0.3 0.4]); grid;
xlabel('time [s]'); ylabel('input'); title('Z System: Input Applied');

% plot the states
figure();


subplot(1,2,1);
plot(time, z_hist(1,1:(sim_length)),'.-'); grid;
xlabel('time [s]'); ylabel('z velocity [m/s]');title('Z Velocity Evolution');

subplot(1,2,2);
plot(time, z_hist(2,1:(sim_length)),'.-'); hold on; grid;
plot(time, z_position_reference*ones(1,sim_length), '--k');
xlabel('time [s]'); ylabel('z position [m]');title('Z Position Evolution');
axis([0 sim_time (min(z_hist(2,:))-0.2) (max(z_hist(2,:))+0.2)]);

%% Simulation for yaw system

sim_time = 10; % seconds
time = 0:Ts:sim_time;
sim_length = length(time); % number of simulation steps 
yaw_hist = zeros(2,sim_length);
uyaw_hist = zeros(1,sim_length-1);

yaw_hist(:,1) = [0;0];% yaw0 (initial state)
yaw_position_reference = pi/4;

for i = 1:sim_length
    fprintf('simulation step %d/%d\n', i, sim_length);
    % Get control inputs with
    uyaw = mpc_yaw.get_u(yaw_hist(:,i), yaw_position_reference);
    uyaw_hist(:,i)= uyaw;
    yaw_hist(:,i+1) = mpc_yaw.A*yaw_hist(:,i) + mpc_yaw.B*uyaw_hist(:,i);
end

%% Plots for sys_yaw controller
%close all;

% determine the settling time
for i=1:length(yaw_hist(2,:))
    threshold = pi/180/10; % (error on the position < threshold in [m])
    if abs(yaw_hist(2,i)-yaw_position_reference) < threshold
        if(abs(yaw_hist(2,i:end)-yaw_position_reference) < threshold)
            break;
        end
    end
end
if i == length(yaw_hist(2,:))
    fprintf('Does not stabilize over the simulation\n');
else
    fprintf('Settling time: %.1f seconds \n',time(i));
end

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
xlabel('time [s]'); ylabel('yaw velocity in [rad/s]');title('Yaw Velocity Evolution');

subplot(1,2,2);
plot(time, yaw_hist(2,1:(sim_length)),'.-'); hold on; grid;
plot(time, yaw_position_reference*ones(1,sim_length), '--k');
xlabel('time [s]'); ylabel('yaw angle [rad]');title('Yaw Angle Evolution');
axis([0 sim_time (min(yaw_hist(2,:))-0.1) (max(yaw_hist(2,:))+0.1)]);
