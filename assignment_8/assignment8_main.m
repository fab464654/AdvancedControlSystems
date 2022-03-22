
clear; clc; close all;

%Add needed functions
addpath("../myFunctions/")

%Set the desired trajectory parameters
A = 0.2; omega = 0.5;

lambda = 10; %defines the relation between q_d and q_r
Kd = 20; %Kd*sigma stabilizes the tracking error via a linear PD control action
Khat_theta = 0.001; %defines the convergence rate

%Plant's parameters initialization
% I*DDq + F*Dq + G*sin(q) = tau
% G = -mgd
myRobot_1DOF.I = 0.1;
myRobot_1DOF.F = 0.05;
myRobot_1DOF.G = -0.1 * 9.81;
theta_true = [myRobot_1DOF.I, myRobot_1DOF.F, myRobot_1DOF.G].';

myRobot_1DOF.dof = 1;
myRobot_1DOF.q0 = 0;
myRobot_1DOF.Dq0 = 0;
myRobot_1DOF.theta_hat_0 = [0.2 0.1 0]; %initial estimates

%Load and open the Simulink system
open_system('adaptiveControl_1DOF');
load_system('adaptiveControl_1DOF')
model = 'adaptiveControl_1DOF';

%Run the simulation and get the outputs
in = Simulink.SimulationInput(model);
out = sim(in);
