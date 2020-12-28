clear; clc; close all;
Ts = 1/5;
quad = Quad(Ts);
[xs, us] = quad.trim();
sys = quad.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_yaw] = quad.decompose(sys, xs, us);

% Save plots or not 
plot = true;

% Design MPC controller
mpc_y = MPC_Control_y(sys_y, Ts);
save_plot('del_31_y', plot)

mpc_x = MPC_Control_x(sys_x, Ts);
save_plot('del_31_x', plot)

mpc_z = MPC_Control_z(sys_z, Ts);
save_plot('del_31_z', plot)

mpc_yaw = MPC_Control_yaw(sys_yaw, Ts);
save_plot('del_31_yaw', plot)

% Get control inputs with
x = [0;0;0;2];
ux = mpc_x.get_u(x)

y = [0;0;0;2];
uy = mpc_y.get_u(y)

z = [0;2];
uz = mpc_z.get_u(z)

yaw = [0;pi/4];
u_yaw = mpc_yaw.get_u(yaw)