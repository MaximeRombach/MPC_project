%% Set path and import Casadi

% path points to the folder where casadi is stored in ones computer
path = 'C:\Program Files\MATLAB\R2018b\toolbox\casadi-windows-matlabR2016a-v3.5.5';
addpath(path)
import  casadi.*
x = MX.sym('x')
disp(jacobian(sin(x),x))