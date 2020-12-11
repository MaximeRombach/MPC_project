quad = Quad();
Tf = 1.0; % Time to simulate for
x0 = zeros(12,1); % Initial state

%Input to apply for flat and level quadcopter:
% u = quad.K\[9.81*quad.mass;0;0;0];

% Input to apply for roll
% u = quad.K\[9.81*quad.mass;1;0;0]

% Input to apply for pitch
% u = quad.K\[9.81*quad.mass;0;1;0]

% Input to apply for yaw
% u = quad.K\[9.81*quad.mass;0;0;10]

sim = ode45(@(t, x) quad.f(x, u), [0, Tf], x0); % Solve the system ODE 
quad.plot(sim, u); % Plot the result

