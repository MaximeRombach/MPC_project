clc;
clear all;
%model setup
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
ctrl_n_ref= quad.merge_controllers_non_ref(mpc_x, mpc_y, mpc_z, mpc_yaw);
%set initial states
ind = quad.ind;
I_x = [ind.omega(2),ind.theta(2),ind.vel(1),ind.pos(1)];
I_y = [ind.omega(1),ind.theta(1),ind.vel(2),ind.pos(2)];
I_z = [ind.vel(3),ind.pos(3)];
I_yaw = [ind.omega(3),ind.theta(3)];
% initial with [x,y,z]=[2,2,2]
Tt = 8;
nbSteps = ceil(Tt/Ts);
x0 = zeros(12,1); 
x0(I_yaw(2)) = pi/4;
x0([I_x(4),I_y(4),I_z(2)]) = 2;

v = zeros(4,nbSteps-1);
u = zeros(4,nbSteps-1);
X= zeros(12,nbSteps);
X(:,1) = x0;
% create simulation with control action 
for i=1:1:nbSteps-1
    u(:,i)  = ctrl_n_ref(X(:,i));
    X(:,i+1) = quad.step(X(:,i), u(:,i), Ts);
end
tt = (0:1:nbSteps-1)*Ts;
u(:,nbSteps)  = ctrl_n_ref(X(:,nbSteps));
%visualize the simulation result
sim.x = tt';
sim.y = X;
quad.plot(sim,u);
% initial with [x,y,z]=[0,0,0] yaw=45/180*pi
% Tt = 8;
% nbSteps = ceil(Tt/Ts);
% x0 = zeros(12,1); 
% x0(I_yaw(2)) = 45/180*pi;
% 
% v = zeros(4,nbSteps-1);
% u = zeros(4,nbSteps-1);
% X= zeros(12,nbSteps);
% X(:,1) = x0;
% % create simulation with control action 
% for i=1:1:nbSteps-1
%     u(:,i)  = ctrl_n_ref(X(:,i));
%     X(:,i+1) = quad.step(X(:,i), u(:,i), Ts);
% end
% tt = (0:1:nbSteps-1)*Ts;
% u(:,nbSteps)  = ctrl_n_ref(X(:,nbSteps));
% %visualize the simulation result
% sim.x = tt';
% sim.y = X;
% quad.plot(sim,u);