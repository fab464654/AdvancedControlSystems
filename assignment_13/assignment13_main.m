
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
    


%% 1) Set 2 desired poses to reach (trajectory tracking)
% Choose a "SCARA-like" pose to easily test the interaction with the 
% environment (XZ-plane)

%--- VERSION a) ---%
vertical_config = homeConfiguration(robot);
vertical_config(3).JointPosition = -0.1;

%--- VERSION b) ---%
diagonal_config = homeConfiguration(robot);
diagonal_config(2).JointPosition = pi/4;
diagonal_config(3).JointPosition = -0.15;
% diagonal_config(2).JointPosition = pi/4;

%--- VERSION c) ---%
diagonal_config2 = homeConfiguration(robot);
diagonal_config2(1).JointPosition = 0.1;
diagonal_config2(2).JointPosition = pi/4;
diagonal_config2(3).JointPosition = -0.15;

config1 = vertical_config; %first goal pose
config2 = diagonal_config; %second goal pose
    
% --- goal pose 1 --- %
toolbox_dk = getTransform(robot,config1,"ee","base_link"); %wrt ∑base
numeric_dk = double(subs(T, [L2,L3,L4,d1(t),theta2(t),d3(t)], [0.4,0.3,0.4,[config1.JointPosition]])); %wrt ∑0

%To create the "xgoalPose" vector, the ee position and the 3 euler angles
%(ZYZ) are needed --> goalPose: 6x1 vector (position+orientation)
% - wrt ∑base (just for information, not used within Simulink): 
goalPose1_base = [toolbox_dk(1:3,4)                     ; %ee position
                 rotm2eul(toolbox_dk(1:3,1:3), 'ZYZ').']; %ee orientation
fprintf("Imposed goal pose 1 w.r.t. ∑base just for information, not used within Simulink:"...
        +"\nx = %f\ny = %f\nz = %f\nφ = %f\nϑ = %f\nψ = %f\n\n", goalPose1_base);
goalPose1_0 = [numeric_dk(1:3,4)                     ;  %ee position
               rotm2eul(numeric_dk(1:3,1:3), 'ZYZ').']; %ee orientation              
           
% --- goal pose 2 --- %
toolbox_dk = getTransform(robot,config2,"ee","base_link"); %wrt ∑base
numeric_dk = double(subs(T, [L2,L3,L4,d1(t),theta2(t),d3(t)], [0.4,0.3,0.4,[config2.JointPosition]])); %wrt ∑0
goalPose2_base = [toolbox_dk(1:3,4)                     ; %ee position
                 rotm2eul(toolbox_dk(1:3,1:3), 'ZYZ').']; %ee orientation
fprintf("Imposed goal pose 1 w.r.t. ∑base just for information, not used within Simulink:"...
        +"\nx = %f\ny = %f\nz = %f\nφ = %f\nϑ = %f\nψ = %f\n\n", goalPose2_base);
goalPose2_0 = [numeric_dk(1:3,4)                     ;  %ee position
               rotm2eul(numeric_dk(1:3,1:3), 'ZYZ').']; %ee orientation 
           
%Set the desired trajectory x_d = [x0 goalPose1_0 goalPose2_0];   
startConf = homeConfiguration(robot);
for k=1:3 startConf(k).JointPosition = myRobot.q0(k); end
T_start = double(subs(T, [L2,L3,L4,d1(t),theta2(t),d3(t)], [0.4,0.3,0.4,[startConf.JointPosition]])); %wrt ∑0

x0 = [ T_start(1:3, 4) ; %if q0_position = [0 0 0] --> x0_position = [0.5 0 0.2]
       rotm2eul(T_start(1:3,1:3), 'ZYZ').']; %x0_orientation

x_d = [x0 goalPose1_0 goalPose2_0]; %desired trajectory
    


%% 2) Set a constant x_d trajectory
% --- CONSTANT goal pose wrt ∑0 --- %
constant_config = vertical_config; %change it here
numeric_dk = double(subs(T, [L2,L3,L4,d1(t),theta2(t),d3(t)], [0.4,0.3,0.4,[constant_config.JointPosition]])); %wrt ∑0
x_d_const = [numeric_dk(1:3,4)                     ;  %ee position
              rotm2eul(numeric_dk(1:3,1:3), 'ZYZ').']; %ee orientation          

% x_d_const = [0.1 -0.3 0.5 1.5708 -1.5708 0].';

fprintf("Imposed CONSTANT goal pose w.r.t. ∑0: \nx = %f\ny = %f\nz = %f\n" + ...
        "φ = %f\nϑ = %f\nψ = %f\n\n", x_d_const);
    
  
%Show the CONSTANT configuration and the plane on a plot 
figure; subplot(1,2,1); show(robot); title("Starting configuration ([0 0 0])")
hold on; patch([0.8 0.1 0.1 0.8], [0.8 0.8 -0.8 -0.8], [0.2 0.2 0.2 0.2], [1 1 -1 -1]);
subplot(1,2,2); show(robot, constant_config); %show the robot in the "desired joint configuration"
hold on; patch([0.8 0.1 0.1 0.8], [0.8 0.8 -0.8 -0.8], [0.2 0.2 0.2 0.2], [1 1 -1 -1]);
title("Constant goal configuration (" + mat2str(round([constant_config.JointPosition],3)) +")");



%% Show the configuration and the plane on a plot 
figure; subplot(1,3,1); show(robot); title("Starting configuration ([0 0 0])")
hold on; patch([0.8 0.1 0.1 0.8], [0.8 0.8 -0.8 -0.8], [0.2 0.2 0.2 0.2], [1 1 -1 -1]);
subplot(1,3,2); show(robot, config1); %show the robot in the "desired joint configuration"
hold on; patch([0.8 0.1 0.1 0.8], [0.8 0.8 -0.8 -0.8], [0.2 0.2 0.2 0.2], [1 1 -1 -1]);
title("Goal configuration 1 (" + mat2str(round([config1.JointPosition],3)) +")");
subplot(1,3,3); show(robot, config2); %show the robot in the "desired joint configuration"
hold on; patch([0.8 0.1 0.1 0.8], [0.8 0.8 -0.8 -0.8], [0.2 0.2 0.2 0.2], [1 1 -1 -1]);
title("Goal configuration 2 (" + mat2str(round([config2.JointPosition],3)) +")");

    
   
%% 3) Start with different initial conditions

% myRobot.q0 = [0.05 pi/3 0.1].'; %joint starting conditions
fprintf("Position initial conditions: \nd1 = %.2f \ntheta2 = %.2f \nd3 = %.2f\n", [myRobot.q0]);


    
    
%% Set the "motion control law" parameters

K = diag([0 10 0 0 0 10]); %normal stiffness (K ~= Kp)
% K = 100*K;   %very high stiffness (K >> Kp)
% K = 0.001*K; %very low stiffness  (K << Kp)



%% Set the "motion control law" parameters
% to guarantee a high value of the disturbance rejection factor
Kp = 50*diag([10, 10, 10, 10, 10, 10]);
Kd = 20*diag([10, 10, 10, 10, 10, 10]);
M_d = 0.1*diag([10 10 10 10 10 10]);

%% Set the "impedance control law" parameters
% to have satisfactory behavior during the interaction with the environment
Kp_t = 5*diag([10, 10, 10, 10, 10, 10]);
Kd_t = 1*diag([10, 10, 10, 10, 10, 10]);
M_t = 0.05*diag([10 10 10 10 10 10]);

%% 
%Load and open the Simulink system
open_system('operationalSpaceInverseDynamicsControl_admittance_compact');
load_system('operationalSpaceInverseDynamicsControl_admittance_compact')
model = 'operationalSpaceInverseDynamicsControl_admittance_compact';

%Run the simulation and get the outputs
in = Simulink.SimulationInput(model);
out = sim(in);

