
clear; clc; close all;

%Add needed functions
addpath("../myFunctions/")


%Call the first assignment to compute the Jacobians and the transformation
%matrix between the two
assign1withoutPrints; clc;
assign3withoutPrints;
clearvars -except Ja_complete Jg T Ta T_phi B C G Dq %remove all the other variables

%Load "myRobot" struct
loadMyRobotStruct; %here are set the initial conditions

 
%% Set the required params
f_d = [0 -0.5 0.1].'; %desired forces (x-y-z)
   
% K = diag([10 10 10]);  %environment stiffness matrix
K = diag([0 10 0]);  %environment stiffness matrix

M_d = 0.1*diag([10 10 10]);

%inner POSITION/VELOCITY controllers
%to test the "wrong shift" unexpected from theory:
% Kp_pos = 0.5*diag([10, 10, 10]); %POSITION loop proportional gain
% Kd_pos = 4*diag([10, 10, 10]);   %POSITION loop derivative gain
%to avoid the unexpected shift:
Kp_pos = 0.5*diag([0, 10, 0]); %POSITION loop proportional gain
Kd_pos = 4*diag([0, 10, 0]);   %POSITION loop derivative gain
%-----------------------------------------------
Kp_vel = 3*diag([10, 10, 10]); %VELOCITY loop proportional gain
Kd_vel = 4*diag([10, 10, 10]); %VELOCITY loop derivative gain

%outer FORCE controller (compliance matrix: forces to positions/velocities)
K_f_pos = 0.5*diag([10 10 10]);  %FORCE controller for inner POSITION loop
K_i_pos = 0.4*diag([10 10 10]); %FORCE controller for inner POSITION loop
%-----------------------------------------------
K_f_vel = 0.1*diag([10 10 10]);  %FORCE controller for inner VELOCITY loop
K_i_vel = 0.05*diag([10 10 10]); %FORCE controller for inner VELOCITY loop

C_f_P_pos = K_f_pos;   %decide in Simulink if PI or P
C_f_P_vel = K_f_vel;   %decide in Simulink if PI or P


%% Load "myRobot" struct
% myRobot.q0 = [0 -pi/2 0].';  %start with different I.C.
fprintf("Joint position initial conditions: \nd1 = %.2f \ntheta2 = %.2f \nd3 = %.2f\n", [myRobot.q0]);

%Load and open the Simulink system
open_system('forceControlInnerPosition');
load_system('forceControlInnerPosition')
model = 'forceControlInnerPosition';
% open_system('forceControlInnerVelocity');
% load_system('forceControlInnerVelocity')
% model = 'forceControlInnerVelocity';

%Run the simulation and get the outputs
in = Simulink.SimulationInput(model);
out = sim(in);

