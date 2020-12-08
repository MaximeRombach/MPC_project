%% Deliverable 2.1
Ts=1/5;
quad = Quad(Ts);
Tf = 1/5; % Time to simulate for
x0 = zeros(12,1); % Initial state
u = [1;1;1;1]; % Input to apply
sim = ode45(@(t, x) quad.f(x, u), [0, Tf], x0); % Solve the system ODE
quad.plot(sim, u); % Plot the result

quad = Quad();
%%
% sys_transformed = sys * inv(quad.T); % New system is A * x + B * inv(T) * v

%% Deliverable 3.1
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
% Get control inputs with
x=[0;0;0;2];
ux = mpc_x.get_u(x); 
y=[0;0;0;2];
uy= mpc_y.get_u(y);
z=[0;2];
uz= mpc_z.get_u(z);
yaw=[0;45*pi/180];
uyaw= mpc_yaw.get_u(yaw);
Tf = 8.0; % Time to simulate for
%u=inv(quad.T)*v;
x_state(:,1)=[x;y;z;yaw];

sys_d_x= c2d(sys_x, 1/5);
sys_d_y= c2d(sys_y, 1/5);
sys_d_z= c2d(sys_z, 1/5);
sys_d_yaw= c2d(sys_yaw, 1/5);

% for i=1:8/Ts
%     x=sys_d_x.A*x+sys_d_x.B*ux;
%     ux = mpc_x.get_u(x);
%     y=sys_d_y.A*y+sys_d_y.B*uy;
%     uy= mpc_y.get_u(y);
%     z=sys_d_z.A*z+sys_d_z.B*uz;
%     uz= mpc_z.get_u(z);
%     yaw=sys_d_yaw.A*yaw+sys_d_yaw.B*uyaw;
%     uyaw=mpc_yaw.get_u(yaw);
%     x_state(:,i)=[x;y;z;yaw];
%     norm(i)=sqrt(x(4)^2+y(4)^2+z(2)^2);
% end
% 
% %%
% figure(1)
% t=(0:Ts:7.8);
% plot(t,norm)
% Get control inputs with
x=[0;0;0;0];
ux = mpc_x.get_u(x,-2);
y=[0;0;0;0];
uy= mpc_y.get_u(y,-2);
z=[0;0];
uz= mpc_z.get_u(z,-2);
yaw=[0;0];
uyaw= mpc_yaw.get_u(yaw,45*pi/180);
%% 4.1

x=[0;0;0;0];
ux = mpc_x.get_u(x,-2);
y=[0;0;0;0];
uy= mpc_y.get_u(y,-2);
z=[0;0];
uz= mpc_z.get_u(z,-2);
yaw=[0;0];
uyaw= mpc_yaw.get_u(yaw,45*pi/180);
x_state(:,1)=[x;y;z;yaw];

% sys_d_x= c2d(sys_x, 1/5);
% sys_d_y= c2d(sys_y, 1/5);
% sys_d_z= c2d(sys_z, 1/5);
% sys_d_yaw= c2d(sys_yaw, 1/5);
% sim = quad.sim(mpc_x, mpc_y, mpc_z, mpc_yaw);
% quad.plot(sim);

for i=1:8/Ts
    x=sys_d_x.A*x+sys_d_x.B*ux;
    ux = mpc_x.get_u(x,-2);
    y=sys_d_y.A*y+sys_d_y.B*uy;
    uy= mpc_y.get_u(y,-2);
    z=sys_d_z.A*z+sys_d_z.B*uz;
    uz= mpc_z.get_u(z,-2);
    yaw=sys_d_yaw.A*yaw+sys_d_yaw.B*uyaw;
    uyaw=mpc_yaw.get_u(yaw,45*pi/180);
    x_state(:,i)=[x;y;z;yaw];
    norm(i)=sqrt(x(4)^2+y(4)^2+z(2)^2);
end
sim = quad.sim(mpc_x, mpc_y, mpc_z, mpc_yaw);
quad.plot(sim);
%% 4
Ts = 1/5;
quad = Quad(Ts);
[xs, us] = quad.trim();
sys = quad.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_yaw] = quad.decompose(sys, xs, us);
mpc_x = MPC_Control_x(sys_x, Ts);
mpc_y = MPC_Control_y(sys_y, Ts);
mpc_z = MPC_Control_z(sys_z, Ts);
mpc_yaw = MPC_Control_yaw(sys_yaw, Ts);
sim = quad.sim(mpc_x, mpc_y, mpc_z, mpc_yaw);
quad.plot(sim);

%%
norm_ref=sqrt(3*4);
figure(1)
t=(0:Ts:7.8);
plot(t,norm-norm_ref)

%% 5.1
clear all 
close all
clc
BIAS = -0.1;
Ts = 1/5;
quad = Quad(Ts);
[xs, us] = quad.trim();
sys = quad.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_yaw] = quad.decompose(sys, xs, us);
mpc_x = MPC_Control_x(sys_x, Ts);
mpc_y = MPC_Control_y(sys_y, Ts);
mpc_z = MPC_Control_z(sys_z, Ts);
mpc_yaw = MPC_Control_yaw(sys_yaw, Ts);
sim = quad.sim(mpc_x, mpc_y, mpc_z, mpc_yaw, BIAS);
quad.plot(sim);
%% 

%% part6
quad = Quad();
CTRL = ctrl_NMPC(quad);
sim = quad.sim(CTRL)
quad.plot(sim)
