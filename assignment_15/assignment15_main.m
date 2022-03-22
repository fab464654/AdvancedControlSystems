
clear; clc; close all;

%Add needed functions
addpath("../myFunctions/")
addpath("../assignment_1/")


%Call the first assignment to compute the Jacobians and the transformation
%matrix between the two
assign1withoutPrints; clc;
assign3withoutPrints;
clearvars -except Ja_complete Jg T Ta T_phi B C G Dq ik_joints %remove all the other variables

%Load "myRobot" struct
loadMyRobotStruct; %here are set the initial conditions

 
%% Set the required params
% 1) No forces; y=-0.1 --> disturbance to reject, no contact; z=0.6 --> outside image(K)
% f_d = [0 0 0 0 0 0].';   %desired forces (x-y-z) and torques (x-y-z)
% x_d = [0 -0.1 0.6 pi/2 -pi/2 0].'; %desired pose

% 2) Force_y=-0.5; y=-0.3 --> contact; z=0.6 --> outside image(K)
f_d = [0 -0.5 0 0 0 0].';   %desired forces (x-y-z) and torques (x-y-z)
x_d = [0 -0.3 0.6 pi/2 -pi/2 0].'; %desired pose

% 3) Trying to test the "end-effector" shift
% f_d = [0 0 0.5 0 0 0].';   %desired forces (x-y-z) and torques (x-y-z)
% x_d = [0 -0.3 0.6 pi/2 -pi/2 0].'; %desired pose


% 4) Trying to impose a torque about z... not so easy
% f_d = [0 0 0 1 0 0].';   %desired forces (x-y-z) and torques (x-y-z)
% x_d = [0.2 -0.3 0.6 pi/2 -pi/2 0].'; %desired pose


Kp = 50*diag([10 10 10 10 10 10]); %inner POSITION loop proportional gain
Kd = 20*diag([10 10 10 10 10 10]); %inner POSITION loop derivative gain

%Environment stiffness matrix, with zeros depending on the constraints:
K = diag([0 10 0 10 0 0]); %y axis and x,z angles are constrained
M_d = 0.1*diag([10 10 10 10 10 10]);

%outer FORCE controller (compliance matrix: forces to positions)
K_f = 0.3*diag([10 10 10 10 10 10]);
K_i = 0.1*diag([10 10 10 10 10 10]);
C_f_P = K_f;   %decide in Simulink if PI or P


%% Visualize the endpose 
x_d_temp = x_d(1:3);
if x_d_temp(1) == 0 %needed to avoid "Division by zero" error
    x_d_temp(1) = 0.00001;
end
jointValues = checkInverseKinematics(x_d_temp, ik_joints);
feasibleJointValues = double(jointValues(1,:));

conf = homeConfiguration(robot);
for k=1:3 conf(k).JointPosition = feasibleJointValues(k); end
  
%Show the CONSTANT configuration and the plane on a plot 
figure; subplot(1,2,1); show(robot); title("Starting configuration ([0 0 0])")
hold on; patch([0.8 0.1 0.1 0.8], [0.8 0.8 -0.8 -0.8], [0.2 0.2 0.2 0.2], [1 1 -1 -1]);
subplot(1,2,2); show(robot, conf); %show the robot in the "desired joint configuration"
hold on; patch([0.8 0.1 0.1 0.8], [0.8 0.8 -0.8 -0.8], [0.2 0.2 0.2 0.2], [1 1 -1 -1]);
title("Constant goal configuration (" + mat2str(round([conf.JointPosition],3)) +")");




%% Load "myRobot" struct
fprintf("Joint position initial conditions: \nd1 = %.2f \ntheta2 = %.2f \nd3 = %.2f\n", [myRobot.q0]);

%Load and open the Simulink system
open_system('parallelForcePosition');
load_system('parallelForcePosition')
model = 'parallelForcePosition';

%Run the simulation and get the outputs
in = Simulink.SimulationInput(model);
out = sim(in);

