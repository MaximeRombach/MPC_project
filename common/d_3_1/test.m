%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% EPFL | Semester: fall 2019                    %
% ME-425: Model Predictive Control | Exercise 1 %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Preliminaries
clc 
close all
clear all

% System dynamics
A = [4/3 -2/3;1 0];
B = [1;0];
C = [-2/3 1];

% Cost matrices
Q = C'*C+0.001*eye(2);
R = 0.001;

%% Prob1 (see slide 2-30)
% Horizon
N = 8;
Kcomp = zeros(size(A,1),N);

% Bellman/Riccati recursion
H = Q;
for i = N:-1:1
	K = -(R+B'*H*B)\B'*H*A;
	H = Q + K'*R*K + (A+B*K)'*H*(A+B*K);
		
	% Store the complete time-varying feedback law
    % (necessary to later plot the predictions)
	Kcomp(:,i) = K;
end

%% Prob2
% Initial condition
x0 = [10; 10];

% Simulation
tmax = 20;
x(:,1) = x0;

% Checking if CL system is stable
if max(abs(eig(A+B*Kcomp(:,1)'))) > 1, error('System is unstable!'); end
    
for t = 1:tmax
    
    % Always apply the first controller gain 
    % ("receiding horizon fashion")
    u(t) = Kcomp(:,1)'*x(:,t); 
    
    x(:,t+1) = A*x(:,t) + B*u(t);
    y(t) = C*x(:,t);
    
    t = t+1;
    
end

% Plotting the state evolution
figure(1)
plot(x0(1),x0(2),'rx'); 
axis([0 11 0 11]); hold on; grid on;
plot(x(1,:),x(2,:),'k-o','LineWidth',1.2);

% Calculating and plotting the prediction (from x0)
xpred(:,1) = x0;
for t=1:N
    xpred(:,t+1) = A*xpred(:,t) + B*(Kcomp(:,t)'*xpred(:,t)); % fixed
end 
plot(xpred(1,:),xpred(2,:),'m-o','LineWidth',1.2);

legend('Initial condition','Closed-loop evolution','Prediction');
title('State-Space');

%% Prob3
% Computing the infinite horizon LQR
[Kih,S,~] = dlqr(A,B,Q,R,[]);
% Reversing its sign (see help dlqr)
Kih = -Kih; 

% Comparing costs
tmax = 5;
xfh(:,1) = x0;
xih(:,1) = x0;
for t = 1:tmax
    
    ufh(t) = Kcomp(:,1)'*xfh(:,t);
    uih(t) = Kih*xih(:,t);
    
    xfh(:,t+1) = A*xfh(:,t) + B*ufh(t);
    xih(:,t+1) = A*xih(:,t) + B*uih(t);
    
    lfh(t) = xfh(:,t)'*Q*xfh(:,t) + ufh(t)'*R*ufh(t);
    lih(t) = xih(:,t)'*Q*xih(:,t) + uih(t)'*R*uih(t);
    
    t = t + 1;
end
lfh(t+1) = xfh(:,t)'*Q*xfh(:,t);
lih(t+1) = xih(:,t)'*Q*xih(:,t);
sss=x0'*S*x0
% Final costs = sum of stage costs
Vfh = sum(lfh);
Vih = sum(lih);
uih = Kih*xih(:,tmax);
xi = A*xih(:,t) + B*uih;
VV=sum(lih(1:tmax))+xi'*S*xi
disp(['Finite horizon cost (with N=' num2str(N) '): ' num2str(Vfh)])
disp(['Infinite horizon cost: ' num2str(Vih)])

% EOF