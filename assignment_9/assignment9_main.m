
clear; clc; close all;

%Add needed functions
addpath("../myFunctions/");
addpath("../assignment_1");

%Call the first assignment to compute the Jacobians and the transformation
%matrix between the two
assign1withoutPrints; clc;
assign3withoutPrints;
clearvars -except Ja_complete Jg robot T Ta T_phi B C G Dq %remove all the other variables

T0_to_ee = T; %we're considering Σ0

%Load "myRobot" struct
loadMyRobotStruct; %here are set the initial conditions
jointLimits = myRobot.jointLimits;
           
%% 1) Get a random feasible joint configuration and compute the direct
%%%   kinematics, to then have a feasible operational space point to reach

randomConf = getRandomConfigurationRight(robot, jointLimits);

toolbox_dk = getTransform(robot,randomConf,"ee","base_link"); %wrt ∑base
numeric_dk = double(subs(T, [L2,L3,L4,d1(t),theta2(t),d3(t)], [0.4,0.3,0.4,[randomConf.JointPosition]])); %wrt ∑0

%To create the "goalPose" vector, the ee position and the 3 euler angles
%(ZYZ) are needed --> goalPose: 6x1 vector (position+orientation)
% - wrt ∑base (just for information, not used within Simulink): 
goalPose_base = [toolbox_dk(1:3,4)                     ;  %ee position
                 rotm2eul(toolbox_dk(1:3,1:3), 'ZYZ').']; %ee orientation
fprintf("Random goal pose w.r.t. ∑base just for information, not used within Simulink:"...
        +"\nx = %f\ny = %f\nz = %f\nφ = %f\nϑ = %f\nψ = %f\n\n", goalPose_base);
    
   
% - wrt ∑0:    
goalPose_0 = [numeric_dk(1:3,4)                     ;  %ee position
              rotm2eul(numeric_dk(1:3,1:3), 'ZYZ').']; %ee orientation          
x_d = goalPose_0;
fprintf("Random goal pose w.r.t. ∑0: \nx = %f\ny = %f\nz = %f\n" + ...
        "φ = %f\nϑ = %f\nψ = %f\n\n", x_d);

numericJacobianDeterminant = subs(det(Ja_complete(1:3,1:3)),[d1(t) theta2(t) d3(t) L4], [randomConf.JointPosition 0.4]);
fprintf("Value of the Jacobian's determinant in the current configuration: \n%f\n\n", ...
        vpa(numericJacobianDeterminant,4));

%% 2) Get a random ALMOST SINGULAR feasible joint configuration and compute the direct
%     kinematics, to then have a feasible operational space point to reach

% numericJacobianDeterminant = 1;
% while numericJacobianDeterminant > 0.001
%     randomConf = getRandomConfigurationRight(robot, jointLimits); 
%     [myDk, myEEpose, ~, ~] = checkDirectKinematics(robot,myRobot,randomConf,T,"0");
%     numericJacobianDeterminant = subs(det(Ja_complete(1:3,1:3)),[d1(t) theta2(t) d3(t) L4], [randomConf.JointPosition 0.4]);
% end
% x_d = double([ myEEpose(1:3,4)               ; %ee position
%              rotm2eul(double(myDk(1:3,1:3)), 'ZYZ').']); %ee orientation    
% fprintf("Value of the Jacobian's determinant in the current configuration: \n%f\n\n", ...
%         vpa(numericJacobianDeterminant,4));
%---------------------------------------------


%% to go faster this is a almost singular configuration (det=0.000976)
%---------------------------------------------
% x_d = [0.0004, 0.0009, 0.5005, 1.1417, -1.5715, -0.0003].';
% almostSingConf = load("almostSingularConf.mat").randomConf;
% 
% numericJacobianDeterminant = subs(det(Ja_complete(1:3,1:3)), ...
%            [d1(t) theta2(t) d3(t) L4], [[almostSingConf.JointPosition] 0.4]);
% 
% fprintf("Almost singular goal coordinates: \nx = %f\ny = %f\nz = %f\n" + ...
%         "φ = %f\nϑ = %f\nψ = %f\n\n", x_d);
% fprintf("Value of the Jacobian's determinant in the current configuration: \n%f\n\n", ...
%         vpa(numericJacobianDeterminant,4));

%% 3) Start with different initial conditions

% myRobot.q0 = [0.05 pi/3 0.1].'; %joint starting conditions
fprintf("Position initial conditions: \nd1 = %.2f \ntheta2 = %.2f \nd3 = %.2f\n", [myRobot.q0]);


%Show the start and goal configurations on a plot 
startConf = homeConfiguration(robot);
for i=1:3 startConf(i).JointPosition = myRobot.q0(i); end
endConf = randomConf; %set to "randomConf" / "almostSingConf" depending on what has been chosen before

figure; subplot(1,2,1); show(robot,startConf); title("Starting configuration (" + mat2str(round([startConf.JointPosition],3)) +")");
subplot(1,2,2); show(robot, endConf); title("Goal configuration (" + mat2str(round([endConf.JointPosition],3)) +")");



%% Set the PD controller values
Kp = diag([30, 30, 30, 20,20, 20]);
% Kp = diag([1000, 900, 1000]); %increase the Kp values to see the overshoot
Kd = diag([20, 20, 20, 10, 10, 10]);


%Load and open the Simulink system
open_system('operationalSpacePDControl');
load_system('operationalSpacePDControl')
model = 'operationalSpacePDControl';

%Run the simulation and get the outputs
in = Simulink.SimulationInput(model);
out = sim(in);

