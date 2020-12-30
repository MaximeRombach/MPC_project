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

sim_time = 15; % seconds
time = 0:Ts:sim_time;
sim_length = length(time); % number of simulation steps 
x_hist = zeros(4,sim_length);
ux_hist = zeros(1,sim_length-1);

x_hist(:,1) = [0;0;0;2];% x0
x_position_reference = 6;

for i = 1:sim_length
    % Get control inputs with
    ux = mpc_x.get_u(x_hist(:,i), x_position_reference);
    ux_hist(:,i)= ux;
    x_hist(:,i+1) = mpc_x.A*x_hist(:,i) + mpc_x.B*ux_hist(:,i);
end

%% Plots for sys_x controller
close all;

% Plot the input
figure();
plot(time,ux_hist(1,:)); hold on;
plot(time, 0.3*ones(1,sim_length), '--r');
plot(time, -0.3*ones(1,sim_length), '--r');
axis([0 sim_time -0.35 0.35]); grid;
xlabel('time in [s]'); ylabel('input'); title('X System: Input Applied');
legend('input','limit');

% plot the states
figure();

subplot(2,2,1);
plot(time, x_hist(1,1:(sim_length)), '.-'); grid;
xlabel('time in [s]'); ylabel('Pitch Velocity in [rad/s]');title('Pitch Velocity Evolution');

subplot(2,2,2);
plot(time, x_hist(2,1:(sim_length)), '.-'); grid;
xlabel('time in [s]'); ylabel('Pitch in [rad]');title('Pitch Evolution');

subplot(2,2,3);
plot(time, x_hist(3,1:(sim_length)), '.-'); grid;
xlabel('time in [s]'); ylabel('x Velocity in [m/s]');title('X Velocity Evolution');


subplot(2,2,4);
plot(time, x_hist(4,1:(sim_length)), '.-'); hold on; grid;
plot(time, x_position_reference*ones(1,sim_length), '-k');
xlabel('time in [s]'); ylabel('x in [m]');title('X Position Evolution');
axis([0 sim_time (min(x_hist(4,:))-0.2) (max(x_hist(4,:))+0.2)]);

%% Simulation for y system

sim_time = 15; % seconds
time = 0:Ts:sim_time;
sim_length = length(time); % number of simulation steps 
x_hist = zeros(4,sim_length);
ux_hist = zeros(1,sim_length-1);

x_hist(:,1) = [0;0;0;0];% y0 (initial state)
y_position_reference = -3;

for i = 1:sim_length
    % Get control inputs with
    ux = mpc_y.get_u(x_hist(:,i), y_position_reference);
    ux_hist(:,i)= ux;
    x_hist(:,i+1) = mpc_y.A*x_hist(:,i) + mpc_y.B*ux_hist(:,i);
end

%% Plots for sys_y controller
close all;

% Plot the input
figure();
plot(time,ux_hist(1,:)); hold on;
plot(time, 0.3*ones(1,sim_length), '--r');
plot(time, -0.3*ones(1,sim_length), '--r');
axis([0 sim_time -0.35 0.35]); grid;
xlabel('time in [s]'); ylabel('input'); title('Y System: Input Applied');
legend('input','limit');

% plot the states
figure();

subplot(2,2,1);
plot(time, x_hist(1,1:(sim_length)), '.-'); grid;
xlabel('time in [s]'); ylabel('Roll Velocity in [rad/s]');title('Roll Velocity Evolution');

subplot(2,2,2);
plot(time, x_hist(2,1:(sim_length)), '.-'); grid;
xlabel('time in [s]'); ylabel('Roll in [rad]');title('Roll Evolution');

subplot(2,2,3);
plot(time, x_hist(3,1:(sim_length)), '.-'); grid;
xlabel('time in [s]'); ylabel('y Velocity in [m/s]');title('Y Velocity Evolution');


subplot(2,2,4);
plot(time, x_hist(4,1:(sim_length)), '.-'); hold on; grid;
plot(time, y_position_reference*ones(1,sim_length), '-k');
xlabel('time in [s]'); ylabel('y in [m]');title('Y Position Evolution');
axis([0 sim_time (min(x_hist(4,:))-0.2) (max(x_hist(4,:))+0.2)]);

%% Simulation for z system

sim_time = 15; % seconds
time = 0:Ts:sim_time;
sim_length = length(time); % number of simulation steps 
x_hist = zeros(2,sim_length);
ux_hist = zeros(1,sim_length-1);

x_hist(:,1) = [0;0];% z0 (initial state)
z_position_reference = -15;

for i = 1:sim_length
    % Get control inputs with
    ux = mpc_z.get_u(x_hist(:,i), z_position_reference);
    ux_hist(:,i)= ux;
    x_hist(:,i+1) = mpc_z.A*x_hist(:,i) + mpc_z.B*ux_hist(:,i);
end
%% Plots for sys_z controller
close all;

% Plot the input
figure();
plot(time,ux_hist(1,:),'.-'); hold on;
plot(time, 0.3*ones(1,sim_length), '--r');
plot(time, -0.2*ones(1,sim_length), '--r');
axis([0 sim_time -0.3 0.4]); grid;
xlabel('time in [s]'); ylabel('input'); title('Z System: Input Applied');

% plot the states
figure();


subplot(1,2,1);
plot(time, x_hist(1,1:(sim_length)),'.-'); grid;
xlabel('time in [s]'); ylabel('z Velocity in [m/s]');title('Z Velocity Evolution');

subplot(1,2,2);
plot(time, x_hist(2,1:(sim_length)),'.-'); hold on; grid;
plot(time, z_position_reference*ones(1,sim_length), '-k');
xlabel('time in [s]'); ylabel('z in [m]');title('Z Position Evolution');
axis([0 sim_time (min(x_hist(2,:))-0.2) (max(x_hist(2,:))+0.2)]);

%% Simulation for yaw system

sim_time = 15; % seconds
time = 0:Ts:sim_time;
sim_length = length(time); % number of simulation steps 
x_hist = zeros(2,sim_length);
ux_hist = zeros(1,sim_length-1);

x_hist(:,1) = [0;-45*pi/180];% yaw0 (initial state)
yaw_position_reference = 180*pi/180;

for i = 1:sim_length
    % Get control inputs with
    ux = mpc_yaw.get_u(x_hist(:,i), yaw_position_reference);
    ux_hist(:,i)= ux;
    x_hist(:,i+1) = mpc_yaw.A*x_hist(:,i) + mpc_yaw.B*ux_hist(:,i);
end

%% Plots for sys_yaw controller
close all;

% Plot the input
figure();
plot(time,ux_hist(1,:)); hold on;
plot(time, 0.2*ones(1,sim_length), '--r');
plot(time, -0.2*ones(1,sim_length), '--r');
axis([0 sim_time -0.3 0.4]); grid;
xlabel('time in [s]'); ylabel('input'); title('Yaw System: Input Applied');

% plot the states
figure();

subplot(1,2,1);
plot(time, x_hist(1,1:(sim_length)),'.-'); grid;
xlabel('time in [s]'); ylabel('Yaw Velocity in [rad/s]');title('Yaw Velocity Evolution');

subplot(1,2,2);
plot(time, x_hist(2,1:(sim_length)),'.-'); hold on; grid;
plot(time, yaw_position_reference*ones(1,sim_length), '-k');
xlabel('time in [s]'); ylabel('yaw in [rad]');title('Yaw Evolution');
axis([0 sim_time (min(x_hist(2,:))-0.1) (max(x_hist(2,:))+0.1)]);
