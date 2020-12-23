clc;
clear all;

quad = Quad();
[xs,us] = quad.trim();
sys = quad.linearize(xs, us);
sys_transformed = sys * inv(quad.T);
[sys_x, sys_y, sys_z, sys_yaw] = quad.decompose(sys, xs, us);
Ts=0.2;
%controller construction
mpc_x = MPC_Control_x(sys_x, Ts);

ux=mpc_x.get_u([0;0;0;0],-2)