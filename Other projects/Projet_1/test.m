% addpath('\\SAMEER-PC\Users\Sameer\Documents\casadi-matlabR2016a-v3.4.5')
addpath('C:\Users\Sameer\Documents/casadi-matlabR2016a-v3.5.1')
import casadi.*
x =MX.sym ('x')
disp ( jacobian (sin ( x ) , x ))
tbxmanager restorepath

cd 'C:\gurobi900\win64\matlab'
gurobi_setup