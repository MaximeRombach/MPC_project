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
ctrl_ref= quad.merge_controllers(mpc_x, mpc_y, mpc_z, mpc_yaw);
%set initial states
ind = quad.ind;
I_x = [ind.omega(2),ind.theta(2),ind.vel(1),ind.pos(1)];
I_y = [ind.omega(1),ind.theta(1),ind.vel(2),ind.pos(2)];
I_z = [ind.vel(3),ind.pos(3)];
I_yaw = [ind.omega(3),ind.theta(3)];
% initial with [x,y,z]=[2,2,2]
Tt = 15;
nbSteps = ceil(Tt/Ts);
x0 = zeros(12,1); 
%ref(1):ref_x,ref(2):ref_y,ref(3):ref_z,ref(4):ref_yaw
ref=[-2,-2,-2,45/180*pi];
v = zeros(4,nbSteps-1);
u = zeros(4,nbSteps-1);
X= zeros(12,nbSteps);
X(:,1) = x0;
% create simulation with control action 
for i=1:1:nbSteps-1
    u(:,i)  = ctrl_ref(X(:,i),ref);
    X(:,i+1) = quad.step(X(:,i), u(:,i), Ts);
end
tt = (0:1:nbSteps-1)*Ts;
u(:,nbSteps)  = ctrl_ref(X(:,nbSteps),ref);
%visualize the simulation result
sim.x = tt';
sim.y = X;
quad.plot(sim,u);