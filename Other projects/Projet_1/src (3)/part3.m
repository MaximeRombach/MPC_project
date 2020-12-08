clear all;clc

Ts = 1/5;
quad = Quad(Ts);
[xs, us] = quad.trim();
sys = quad.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_yaw] = quad.decompose(sys, xs, us);

% Design MPC controller
mpc_x = MPC_Control_x(sys_x, Ts);
% Get control inputs with
x=[0;0;0;2];
ux = mpc_x.get_u(x)

% Design MPC controller
mpc_y = MPC_Control_y(sys_y, Ts);
% Get control inputs with
x=[0;0;0;2];
uy = mpc_y.get_u(x)

% Design MPC controller
mpc_z = MPC_Control_z(sys_z, Ts);
% Get control inputs with
x=[0;2];
uz = mpc_z.get_u(x)

% Design MPC controller
mpc_yaw = MPC_Control_yaw(sys_yaw, Ts);
% Get control inputs with
x=[0;0.785398];
uyaw = mpc_yaw.get_u(x)

%%
quad = Quad();
Tf = 8; % Time to simulate for
x0 = [0; 0; 0; 0; 0; 0.785398; 0;0;0;2;2;2]; % Initial state
u = [ux;uy;uz;uyaw]; % Input to apply
sim = ode45(@(t, x) quad.f(x, u), [0, Tf], x0); % Solve the system ODE
quad.plot(sim, u);
%%
clear all
close all

Ts = 1/5;
quad = Quad(Ts);
[xs, us] = quad.trim();
sys = quad.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_yaw] = quad.decompose(sys, xs, us);
x=[0;0;0;2];
y=[0;0;0;2];
z=[0;2];
yaw=[0;0.785398];
ut=[];
xf=[0; 0; 0; 0; 0; 0.785398; 0;0;0;2;2;2];
for i=1:20
mpc_x = MPC_Control_x(sys_x, Ts);
ux = mpc_x.get_u(x);
mpc_y = MPC_Control_y(sys_y, Ts);
uy = mpc_y.get_u(y);
mpc_z = MPC_Control_z(sys_z, Ts);
uz = mpc_z.get_u(z);
mpc_yaw = MPC_Control_yaw(sys_yaw, Ts);
uyaw = mpc_yaw.get_u(yaw);
u=[uz;uy;ux;uyaw];
ut=[ut,u];
x0=[y(1);x(1);yaw(1);y(2);x(2);yaw(2);x(3);y(3);z(1);x(4);y(4);z(2)];
% sim = ode45(@(t, x) quad.f(x, u), [Ts*(i-1),Ts*i ], x0)
% xt=sim.x;
xt=sys.A*x0+sys.B*inv(quad.T)*u;
% xt=sys.A*x0+sys.B*u;
xf=[xf,xt];
x=[xt(2);xt(5);xt(7);xt(10)];
y=[xt(1);xt(4);xt(8);xt(11)];
z=[xt(9);xt(12)];
yaw=[xt(3);xt(6)];
end
%%
% quad = Quad();
% Tf = 8; % Time to simulate for
% for i=1:5
%  x0=xf(:,i);
%  u=ut(:,i);
% sim = ode45(@(t, x) quad.f(x, u), [Ts*(i-1), Ts*i], x0); % Solve the system ODE
% quad.plot(sim, u);
% hold on
% end
%%
clear all 
clc
Ts = 1/5;
quad = Quad(Ts);
[xs, us] = quad.trim();
sys = quad.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_yaw] = quad.decompose(sys, xs, us);
% Design MPC controller
mpc_x = MPC_Control_x(sys_x, Ts);
% Get control inputs with
x=[0;0;0;0];
x_position_reference= -2;
ux = mpc_x.get_u(x, x_position_reference)

mpc_y = MPC_Control_y(sys_y, Ts);
% Get control inputs with
y=[0;0;0;0];
y_position_reference= -2;
uy = mpc_y.get_u(y, y_position_reference)

mpc_z = MPC_Control_z(sys_z, Ts);
% Get control inputs with
z=[0;0];
z_position_reference= -2;
uz = mpc_z.get_u(z, z_position_reference)

mpc_yaw = MPC_Control_yaw(sys_yaw, Ts);
% Get control inputs with
yaw=[0;0];
yaw_position_reference= 0.785398;
uyaw = mpc_yaw.get_u(yaw, yaw_position_reference)
