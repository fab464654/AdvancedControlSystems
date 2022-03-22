
clear; clc; close all;

%Add needed functions
addpath("../myFunctions/")


%Call the first assignment to compute the Jacobians and the transformation
%matrix between the two
assign1withoutPrints; clc;
assign3withoutPrints;
clearvars -except Ja_complete Jg T Ta T_phi B C G Dq %remove all the other variables

T0_to_ee = T; %we're considering Σ0

%Load "myRobot" struct
loadMyRobotStruct; %here are set the initial conditions
jointLimits = myRobot.jointLimits;

    
%% 1) Get 2 random feasible joint configurations and compute the direct
%%%   kinematics, to then have 2 feasible operational space points to reach

%Show the 3 configurations
figure; subplot(1,3,1); show(robot); title("Starting configuration ([0 0 0])")
for i=1:2
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
    if(i==1)
        subplot(1,3,2); show(robot, randomConf); %show the robot in the "desired joint configuration"
        title("First goal configuration (" + mat2str(round([randomConf.JointPosition],3)) +")");
        goalPose1_0 = goalPose_0; %determin first goal pose
    else
        subplot(1,3,3); show(robot, randomConf); %show the robot in the "desired joint configuration"
        title("Second goal configuration (" + mat2str(round([randomConf.JointPosition],3)) +")");
        goalPose2_0 = goalPose_0; %determin second goal pose
    end
end
    
startConf = homeConfiguration(robot);
for k=1:3 startConf(k).JointPosition = myRobot.q0(k); end
% T_start = getTransform(robot,startConf,"ee","base_link"); %wrong because wrt ∑base
T_start = double(subs(T, [L2,L3,L4,d1(t),theta2(t),d3(t)], [0.4,0.3,0.4,[startConf.JointPosition]])); %wrt ∑0

x0 = [ T_start(1:3, 4) ; %if q0_position = [0 0 0] --> x0_position = [0.5 0 0.2]
       rotm2eul(T_start(1:3,1:3), 'ZYZ').']; %x0_orientation


x_d = [x0 goalPose1_0 goalPose2_0]; %desired trajectory
    
%Uncomment to set a easy trajectory:
% goalPose1_0 = [0 , -0.1, 0.5, pi/2, -pi/2, 0].';
% goalPose2_0 = [0 , -0.1, 0.4, pi/2, -pi/2, 0].';
% x_d = [x0 goalPose1_0 goalPose2_0]; %desired trajectory

%% Set a constant x_d trajectory
x_d_const = [-0.139961;
              0.159418;
              0.559328;
             -0.850297;
             -1.570796;
             0       ];
         
 
    
Kp = 5*diag([100, 100, 100,50,50,50]);
% Kp = diag([1000, 1000, 1000, 1000, 1000, 1000]); %increase the Kp values to see the overshoot
Kd = diag([100, 100, 100, 50, 50, 50]);




%% Load "myRobot" struct
fprintf("Joint position initial conditions: \nd1 = %.2f \ntheta2 = %.2f \nd3 = %.2f\n", [myRobot.q0]);

%Load and open the Simulink system
open_system('operationalSpaceInverseDynamicsControl');
load_system('operationalSpaceInverseDynamicsControl')
model = 'operationalSpaceInverseDynamicsControl';

%Run the simulation and get the outputs
in = Simulink.SimulationInput(model);
out = sim(in);

