clc;
clear all;
%model setup
BIAS = -0.1;
quad = Quad();
[xs,us] = quad.trim();
sys = quad.linearize(xs, us);
sys_transformed = sys * inv(quad.T);
[sys_x, sys_y, sys_z, sys_yaw] = quad.decompose(sys, xs, us);
Ts=0.2;
%controller construction
mpc_x = MPC_Control_x(sys_x, Ts);
mpc_y = MPC_Control_y(sys_y, Ts);
mpc_z = MPC_Control_z(sys_z, Ts);
mpc_yaw = MPC_Control_yaw(sys_yaw, Ts);
sim = quad.sim(mpc_x, mpc_y, mpc_z, mpc_yaw,BIAS);
quad.plot(sim);
