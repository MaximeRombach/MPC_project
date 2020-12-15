clear;
Ts = 1/5;
quad = Quad(Ts);
[xs, us] = quad.trim();
sys = quad.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_yaw] = quad.decompose(sys, xs, us);
% Design MPC controller
% mpc_x = MPC_Control_x(sys_x, Ts);
mpc_yaw = MPC_Control_yaw(sys_yaw, Ts);
% Get control inputs with
% x = [0;0;0;0];
% ux = mpc_x.get_u(x)
yaw = [pi/4;0];
u_yaw = mpc_yaw.get_u(yaw)