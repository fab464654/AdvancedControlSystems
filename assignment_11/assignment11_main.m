
clear; clc; close all;

%Add needed functions
addpath("../myFunctions/");
addpath("../assignment_1");

%Call the first assignment to compute the Jacobians and the transformation
%matrix between the two
assign1withoutPrints; clc;
assign3withoutPrints;
clearvars -except ik_joints Ja_complete Jg robot T Ta T_phi B C G Dq %remove all the other variables

T0_to_ee = T; %we're considering Σ0

%Load "myRobot" struct
loadMyRobotStruct; %here are set the initial conditions
jointLimits = myRobot.jointLimits;
           
%% 1) Get a random feasible joint configuration and compute the direct
%%%   kinematics, to then have a feasible operational space point to reach

% randomConf = getRandomConfigurationRight(robot, jointLimits);
% figure; show(robot, randomConf); %show the robot in the "desired joint configuration"
% 
% toolbox_dk = getTransform(robot,randomConf,"ee","base_link"); %wrt ∑base
% numeric_dk = double(subs(T, [L2,L3,L4,d1(t),theta2(t),d3(t)], [0.4,0.3,0.4,[randomConf.JointPosition]])); %wrt ∑0
% 
% %To create the "goalPose" vector, the ee position and the 3 euler angles
% %(ZYZ) are needed --> goalPose: 6x1 vector (position+orientation)
% % - wrt ∑base (just for information, not used within Simulink): 
% goalPose_base = [toolbox_dk(1:3,4)                     ;  %ee position
%                  rotm2eul(toolbox_dk(1:3,1:3), 'ZYZ').']; %ee orientation
% fprintf("Random goal pose w.r.t. ∑base just for information, not used within Simulink:"...
%         +"\nx = %f\ny = %f\nz = %f\nφ = %f\nϑ = %f\nψ = %f\n\n", goalPose_base);
%     
%    
% % - wrt ∑0:    
% goalPose_0 = [numeric_dk(1:3,4)                     ;  %ee position
%               rotm2eul(numeric_dk(1:3,1:3), 'ZYZ').']; %ee orientation          
% x_d = goalPose_0;
% fprintf("Random goal pose w.r.t. ∑0: \nx = %f\ny = %f\nz = %f\n" + ...
%         "φ = %f\nϑ = %f\nψ = %f\n\n", x_d);
% 
% numericJacobianDeterminant = subs(det(Ja_complete(1:3,1:3)),[d1(t) theta2(t) d3(t) L4], [randomConf.JointPosition 0.4]);
% fprintf("Value of the Jacobian's determinant in the current configuration: \n%f\n\n", ...
%         vpa(numericJacobianDeterminant,4));
%     
% config = randomConf;


%% 2) Choose a "SCARA-like" pose to easily test the interaction with the 
%     environment (XZ-plane)

%--- VERSION a) ---%
% vertical_config = homeConfiguration(robot);
% vertical_config(3).JointPosition = -0.1;
% config = vertical_config;

%--- VERSION a2) ---%
verticalForward_config = homeConfiguration(robot);
verticalForward_config(1).JointPosition = 0.1;
verticalForward_config(3).JointPosition = -0.1;
config = verticalForward_config;

%--- VERSION b) ---%
% diagonal_config = homeConfiguration(robot);
% diagonal_config(2).JointPosition = pi/4;
% diagonal_config(3).JointPosition = -0.15;
% config = diagonal_config;

%--- VERSION c) ---%
% diagonal_config2 = homeConfiguration(robot);
% diagonal_config2(1).JointPosition = 0.1;
% diagonal_config2(2).JointPosition = pi/4;
% diagonal_config2(3).JointPosition = -0.15;
% config = diagonal_config2;


%% Show the configuration and the plane on a plot 
figure; subplot(1,2,1); show(robot); title("Starting configuration ([0 0 0])")
hold on; patch([0.8 0.1 0.1 0.8], [0.8 0.8 -0.8 -0.8], [0.2 0.2 0.2 0.2], [1 1 -1 -1]);
subplot(1,2,2); show(robot, config); %show the robot in the "desired joint configuration"
hold on; patch([0.8 0.1 0.1 0.8], [0.8 0.8 -0.8 -0.8], [0.2 0.2 0.2 0.2], [1 1 -1 -1]);
title("Goal configuration (" + mat2str(round([config.JointPosition],2)) +")");

%make here your second plot
toolbox_dk = getTransform(robot,config,"ee","base_link"); %wrt ∑base
numeric_dk = double(subs(T, [L2,L3,L4,d1(t),theta2(t),d3(t)], [0.4,0.3,0.4,[config.JointPosition]])); %wrt ∑0

%To create the "goalPose" vector, the ee position and the 3 euler angles
%(ZYZ) are needed --> goalPose: 6x1 vector (position+orientation)
% - wrt ∑base (just for information, not used within Simulink): 
goalPose_base = [toolbox_dk(1:3,4)                     ;  %ee position
                 rotm2eul(toolbox_dk(1:3,1:3), 'ZYZ').']; %ee orientation
fprintf("Imposed vertical goal pose w.r.t. ∑base just for information, not used within Simulink:"...
        +"\nx = %f\ny = %f\nz = %f\nφ = %f\nϑ = %f\nψ = %f\n\n", goalPose_base);
    
   
% - wrt ∑0:    
goalPose_0 = [numeric_dk(1:3,4)                     ;  %ee position
              rotm2eul(numeric_dk(1:3,1:3), 'ZYZ').']; %ee orientation          
x_d = goalPose_0;
fprintf("Imposed goal pose w.r.t. ∑0: \nx = %f\ny = %f\nz = %f\n" + ...
        "φ = %f\nϑ = %f\nψ = %f\n\n", x_d);

%% Choose the required params:
K = diag([0 10 0 0 0 10]); %normal stiffness (K ~= Kp)
% K  = 100*K;   %very high stiffness (K >> Kp)
% K = 0.001*K; %very low stiffness  (K << Kp)


%% 3) Start with different initial conditions

% myRobot.q0 = [0.05 pi/3 0.1].'; %joint starting conditions
fprintf("Position initial conditions: \nd1 = %.2f \ntheta2 = %.2f \nd3 = %.2f\n", [myRobot.q0]);


%% Set the PD controller values
Kp = diag([10, 20, 20, 50, 50, 50]);
% Kp = diag([1000, 900, 1000, 1000, 900, 1000]); %increase the Kp values to see the overshoot

Kd = diag([1, 5, 10, 1, 1, 1]);

%%
%Load and open the Simulink system
open_system('operationalSpacePDControl_compliance');
load_system('operationalSpacePDControl_compliance')
model = 'operationalSpacePDControl_compliance';

%Run the simulation and get the outputs
in = Simulink.SimulationInput(model);
out = sim(in);

