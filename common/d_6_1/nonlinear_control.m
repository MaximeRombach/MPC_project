clc;
clear all;
%model setup
quad = Quad();
CTRL = nonlinear_controller(quad);
% x0=[0;0;0;0.2;0;0;0;0;0;2;2;1];
% ref=[0;0;1;0.5]
% u=CTRL(x0,ref)

sim = quad.sim(CTRL);
quad.plot(sim)
