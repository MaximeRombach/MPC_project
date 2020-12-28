%% Set path to project folders

addpath('d_3_1','./figures')

%% Set path and import Casadi

% path points to the folder where casadi is stored in ones computer
path = 'C:\Program Files\MATLAB\R2018b\toolbox\casadi-windows-matlabR2016a-v3.5.5';
addpath(path)
import  casadi.*
x = MX.sym('x')
disp(jacobian(sin(x),x))

%% Run the following to set latex interpreter on all figures

%set(groot, 'defaultAxesTickLabelInterpreter','latex'); % set Latex to axes ticks
%set(groot, 'defaultLegendInterpreter','latex'); %set latex to legend