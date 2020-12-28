clear;
close all;
clc;
%% Simulation 
%/!\ u = 0-1.5 /!\
quad = Quad();
Tf = 1.0; % Time to simulate for
x0 = zeros(12,1); % Initial state


%% Initial input
u = [0.6;0.6;0.6;0.6]; % Input to apply
u = [u(1);u(2);u(3);u(4)]
% [u1, u3] on x-axis (roll axis)
% [u2, u4] on y-axis (pitch axis)

%% Flat and level inputs
% u = 0.7007*ones(4,1) 

%% Roll inputs
% u1 = 0.7; u3 = 0.7;
% u2 = 0.8; u4 = 0.6;
% u = [u1;u2;u3;u4]; 

%% Pitch inputs
% u1 = 0.6; u3 = 0.8;
% u2 = 0.7; u4 = 0.7;
% u = [u1;u2;u3;u4];  

%% Yaw inputs
% u1 = 0.5; u3 = 0.5;
% u2 = 1; u4 = 1;
% u = [u1;u2;u3;u4]; 

sim = ode45(@(t, x) quad.f(x, u), [0, Tf], x0); % Solve the system ODE
quad.plot(sim, u); % Plot the result