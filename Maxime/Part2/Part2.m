clear; close all; clc;
%% Tool 2.1

% Generate trimmed (f(x_bar, u_bar) = 0) and linearized version of the
% system

quad = Quad();
[xs,us] = quad.trim(); % Compute steady?state for which 0 = f(xs,us)
sys = quad.linearize(xs, us); % Linearize the nonlinear model

%% Tool 2.2

sys_transformed = sys * inv(quad.T); % New system is A * x + B * inv(T) * v

%% Deliverable 2.1
% The A,C,D matrices don't change only the B matrix changes
% All its lines become 0, except for 4 of them: the ones concerning the
% angular velocities and z velocity.
% There is only one non zero term on each one of them, which means that
% each v_i acts on a single coordinate of the state.
% v1 --> z velocity
% v2 --> roll velocity
% v3 --> pitch velocity
% v4 --> yaw velocity

%% Tool 2.3

[sys_x, sys_y, sys_z, sys_yaw] = quad.decompose(sys, xs, us);
