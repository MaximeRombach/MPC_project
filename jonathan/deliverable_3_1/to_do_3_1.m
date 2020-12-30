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

sim_time = 10; % seconds
time = 0:Ts:sim_time;
sim_length = length(time); % number of simulation steps 
x_hist = zeros(4,sim_length);
ux_hist = zeros(1,sim_length-1);

x_hist(:,1) = [0;0;0;2];

for i = 1:sim_length
    % Get control inputs with
    ux = mpc_x.get_u(x_hist(:,i));
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
axis([0 sim_time -0.4 0.4]); grid;
xlabel('time in [s]'); ylabel('input'); title('X System: Input Applied');

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
plot(time, x_hist(4,1:(sim_length)), '.-'); grid;
xlabel('time in [s]'); ylabel('x in [m]');title('X Position Evolution');
%% Evaluate Controller y
% Design MPC controller
mpc_y = MPC_Control_y(sys_y, Ts);

sim_time = 10; % seconds
time = 0:Ts:sim_time;
sim_length = length(time); % number of simulation steps 
x_hist = zeros(4,sim_length);
ux_hist = zeros(1,sim_length-1);

x_hist(:,1) = [0;0;0;2];

for i = 1:sim_length
    % Get control inputs with
    ux = mpc_y.get_u(x_hist(:,i));
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
axis([0 sim_time -0.4 0.4]); grid;
xlabel('time in [s]'); ylabel('input'); title('Y System: Input applied');

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
plot(time, x_hist(4,1:(sim_length)), '.-'); grid;
xlabel('time in [s]'); ylabel('y in [m]');title('Y Position Evolution');

%% Evaluate Controller z
% Design MPC controller
mpc_z = MPC_Control_z(sys_z, Ts);

sim_time = 15; % seconds
time = 0:Ts:sim_time;
sim_length = length(time); % number of simulation steps 
x_hist = zeros(2,sim_length);
ux_hist = zeros(1,sim_length-1);

x_hist(:,1) = [0;2];

for i = 1:sim_length
    % Get control inputs with
    ux = mpc_z.get_u(x_hist(:,i));
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
xlabel('time in [s]'); ylabel('x Velocity in [m/s]');title('Z Position Evolution');

subplot(1,2,2);
plot(time, x_hist(2,1:(sim_length)),'.-'); grid;
xlabel('time in [s]'); ylabel('x in [m]');title('Z Velocity Evolution');

%% Evaluate Controller yaw
% Design MPC controller
mpc_yaw = MPC_Control_yaw(sys_z, Ts);

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

%% Plots for sys_yaw controller
close all;

% Plot the input
figure();
plot(time,uyaw_hist(1,:)); hold on;
plot(time, 0.2*ones(1,sim_length), '--r');
plot(time, -0.2*ones(1,sim_length), '--r');
axis([0 sim_time -0.3 0.4]); grid;
xlabel('time in [s]'); ylabel('input'); title('Yaw System: Input Applied');

% plot the states
figure();

subplot(1,2,1);
plot(time, yaw_hist(1,1:(sim_length)),'.-'); grid;
xlabel('time in [s]'); ylabel('Yaw Velocity in [rad/s]');

subplot(1,2,2);
plot(time, yaw_hist(2,1:(sim_length)),'.-'); grid;
xlabel('time in [s]'); ylabel('yaw in [rad]');

