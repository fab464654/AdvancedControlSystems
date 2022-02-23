
clear; close all; clc;
 
%Add needed functions
addpath("../myFunctions/")

%Import URDF file
robot = importrobot('../URDF/PRP_2.urdf');

%Load the robot's information
loadMyRobotStruct;

%Show information about the structure
showdetails(robot)

%Show 3DOF PRP robot
config = homeConfiguration(robot);
%Set a specific configuration
config(1).JointPosition = 0.1;
config(2).JointPosition = pi/3;
config(3).JointPosition = -0.14;    
% showRobot(robot, config);

jointLimits = myRobot.jointLimits;

%% IMPORTANT: the user MUST choose the overall ref. wrt ∑0 or ∑base!!!
refFrame = "0";  % ∑0 = "0"; ∑base = "base"

%Initialize the DH table
syms L0 L2 L3 L4 d1(t) theta2(t) d3(t) t
if strcmp(refFrame, "0")
    DH_table = myRobot.DH_table_zero; 
elseif strcmp(refFrame, "base")
    DH_table = myRobot.DH_table_base; 
end

%Compute Direct and Inverse kinematics (+ Jacobians)
jointStructure = myRobot.jointStructure;

[T, T_all, Ja, Jg, ik_joints] = computeKinematics(DH_table, jointStructure, refFrame);

%To get LaTeX code for matrices
% fprintf("\nLaTeX equivalent code...\n")
% fprintf("--------------------------\n")
% fprintf('%s\n', getLatexEquation("A_1^0", T_all(:,:,1)));
% fprintf('%s\n', getLatexEquation("A_2^0", T_all(:,:,2)));
% fprintf('%s\n', getLatexEquation("A_3^0", T_all(:,:,3)));
% fprintf('%s\n', getLatexEquation("A_4^0", T));
% fprintf('%s\n', getLatexEquation("J_{g}", Jg));
% fprintf("--------------------------\n\n")



%Check the direct kinematics computation
if strcmp(refFrame, "0")
    DH_table = myRobot.DH_table_zero; 
elseif strcmp(refFrame, "base")
    DH_table = myRobot.DH_table_base; 
end
[toolbox_dk, my_dk, toolbox_ee_pose, my_ee_pose] = checkDirectKinematics(robot, config, T, refFrame);
fprintf("\n------------------\n");
fprintf("[CheckDirect] Checking the direct kinematics computation, considering "+...
        "the given joint configuration\n");
fprintf("------------------\n");
fprintf("[CheckDirect] Given joint parameters -> [%.3f %.3f %.3f]\n", config.JointPosition);
fprintf("[CheckDirect] Toolbox -> [%.3f %.3f %.3f]\n",  toolbox_ee_pose);
fprintf("[CheckDirect] Computed -> "); disp(vpa(my_ee_pose.', 3));
fprintf("------------------\n\n");



%Check the inverse kinematics computation
score = 0;
numTries = 2;
threshold = 0.1;
maxScore = 0; %accounts for multiple solutions to the ik problem

fprintf("\n------------------\n");
fprintf("[CheckInverse] Checking the inverse kinematics...\n");
for i=1:numTries
    fprintf("------------------\n");
    fprintf("[Number of try: %d] \n", i);
    %randomConf = randomConfiguration(robot); %doens't consider joint
    %limtis!!!
    randomConf = getRandomConfigurationRight(robot, jointLimits); %my function!
    
    [~, ~, goalPose, ~] = checkDirectKinematics(robot, randomConf, T, refFrame);
    my_joints = checkInverseKinematics(goalPose, ik_joints);

    %There could be multiple IK solutions
    for k=1:size(my_joints, 1) 
        %Recall the direct kinematics from the computed joint params with
        %the inverse kinematics
        
        %checking that the computed "my_joints" are inside the "jointLimits"
        if all(double(my_joints(k,:)) > jointLimits(:,1).') && all(double(my_joints(k,:)) < jointLimits(:,2).') 
            currentConf = homeConfiguration(robot); 
            currentConf(1).JointPosition = double(my_joints(k,1));
            currentConf(2).JointPosition = double(my_joints(k,2));
            currentConf(3).JointPosition = double(my_joints(k,3));

            [~, ~, computedPoseFromJointParams, ~] = checkDirectKinematics(robot, currentConf, T, refFrame);
            fprintf("[CheckInverse] Direct kinematics from the (%d) joint params -> [%.3f %.3f %.3f]\n", k, computedPoseFromJointParams);

            maxScore = maxScore + 1;
            if abs(goalPose - computedPoseFromJointParams) < [threshold threshold threshold]
                score = score + 1;
            end
        else %configuration without physical meaning (not doable)
            fprintf("[CheckInverse] The (%d) configuration isn't feasible!\n", k)
        end
    end   
    fprintf("\n");
end
fprintf("[CheckInverse] Score of the inverse kinematic computation: %d/%d\n", score, maxScore)
fprintf("------------------\n");


%Check the Geometric Jacobian
if strcmp(refFrame, "0")
    fprintf("\n------------------\n");
    fprintf("Considered Robot configuration -> [%.2f %.2f %.2f]\n", config.JointPosition);
    Jg_toolbox_base = getGeometricJacobianRight(robot, config, "ee");
    
    T_base_to_0 = myRobot.T_base_to_0_DH; %retrive (DH) transf. from ∑base to ∑0
    T_0_to_base = inv(T_base_to_0);
    R_0_to_base = T_0_to_base(1:3,1:3);
    T_jacobians = [R_0_to_base  , zeros(3)  ;
                   zeros(3)     , R_0_to_base    ];

    Jg_toolbox_zero = vpa(subs(T_jacobians*Jg_toolbox_base, [L0],[0.4]), 3);
    fprintf("\nToolbox Jg wrt ∑0:\n");
    disp(Jg_toolbox_zero)
    
    fprintf("Computed Jg wrt ∑0:\n");
    Jg_computed = vpa(subs(Jg, [L0 L2 L3 L4 d1(t) theta2(t) d3(t)], [0.4,0.4,0.3,0.4,config.JointPosition]), 5);
    disp(Jg_computed)
    fprintf("------------------\n");

elseif strcmp(refFrame, "base")
    fprintf("\n------------------\n");
    fprintf("Checking the Geometric Jacobian...\n");
    fprintf("Considered Robot configuration -> [%.2f %.2f %.2f]\n", config.JointPosition);
    Jg_toolbox = getGeometricJacobianRight(robot, config, "ee");
    fprintf("\nToolbox Jg:\n");
    disp(Jg_toolbox)

    fprintf("Computed Jg:\n");
    Jg_computed = vpa(subs(Jg, [L0 L2 L3 L4 d1(t) theta2(t) d3(t)], [0.4,0.4,0.3,0.4,config.JointPosition]), 5);
    disp(Jg_computed)
    fprintf("------------------\n");
end

%% Compute transformation matrix between Jg and Ja (base_link/∑0 -> ee frames)
%Formula: Jg = Ta(phi) * Ja_complete
if strcmp(refFrame, "0")
    R = T_all(1:3, 1:3, 3);
    R = eval(vpa(subs(R, [L0 L2 L3 L4 d1(t) theta2(t) d3(t)], [0.4,0.4,0.3,0.4,config.JointPosition]), 5));
    eulerAngles = rotm2eul(R,"ZYZ");

    phi   = eulerAngles(1);
    theta = eulerAngles(2);
    psi   = eulerAngles(3);

    T_phi = [0 -sin(phi) cos(phi)*sin(theta);
             0  cos(phi) sin(phi)*sin(theta);
             1     0     cos(theta)        ];

    Ta = [1 0 0     0 0 0     ;
          0 1 0     0 0 0     ;
          0 0 1     0 0 0     ;
          0 0 0   , T_phi(1,:);
          0 0 0   , T_phi(2,:);
          0 0 0   , T_phi(3,:)];

    Ja_complete = symSubsZeros(inv(Ta)*Jg, 0.00001);
    disp("Ja_complete (wrt ∑0) computed from inv(Ta)*Jg:"); disp(Ja_complete);
    % fprintf('%s\n', getLatexEquation("J_{a}", Ja_complete));
elseif strcmp(refFrame, "base")
    R = getTransform(robot,config,"ee","base_link");
    R = R(1:3, 1:3);
    eulerAngles = rotm2eul(R,"ZYZ");

    phi   = eulerAngles(1);
    theta = eulerAngles(2);
    psi   = eulerAngles(3);

    T_phi = [0 -sin(phi) cos(phi)*sin(theta);
             0  cos(phi) sin(phi)*sin(theta);
             1     0     cos(theta)        ];

    Ta = [1 0 0     0 0 0     ;
          0 1 0     0 0 0     ;
          0 0 1     0 0 0     ;
          0 0 0   , T_phi(1,:);
          0 0 0   , T_phi(2,:);
          0 0 0   , T_phi(3,:)];

    Ja_complete = symSubsZeros(inv(Ta)*Jg, 0.00001);
    disp("Ja_complete (wrt ∑base) computed from inv(Ta)*Jg:"); disp(Ja_complete);
    % fprintf('%s\n', getLatexEquation("J_{a}", Ja_complete));
end


