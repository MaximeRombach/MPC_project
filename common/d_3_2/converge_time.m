ind = quad.ind;
I_x = [ind.omega(2),ind.theta(2),ind.vel(1),ind.pos(1)];
I_y = [ind.omega(1),ind.theta(1),ind.vel(2),ind.pos(2)];
I_z = [ind.vel(3),ind.pos(3)];
I_yaw = [ind.omega(3),ind.theta(3)];
output=[mpc_x.C*X(I_x,:);mpc_y.C*X(I_y,:);mpc_z.C*X(I_z,:);mpc_yaw.C*X(I_yaw,:)];
error=output-ref';
indice=abs(error)<1e-4;
[n,m]=size(error);
A=zeros(n,m);
A(indice)=1;
sd=sum(A);
[a,b]=find(sd==4)

T=tt(b(1))