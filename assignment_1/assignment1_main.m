
clear; close all; clc;
 
%Add needed functions
addpath("../myFunctions/")

%Import URDF file
robot = importrobot('../URDF/PRP_2.urdf');

%Show information about the structure
showdetails(robot)

%Show 3DOF PRP robot
config = homeConfiguration(robot);
%Set a specific configuration
config(1).JointPosition = 0.1;
config(2).JointPosition = pi/3;
config(3).JointPosition = -0.14;    
% showRobot(robot, config);

jointLimits = [-0.2 0.2;
               -pi  pi ;
               -0.2 0.2];

%%
%Initialize the DH table
syms L0 L2 L3 L4 d1(t) theta2(t) d3(t) t
%L2 = length of Link2 = 0.4m
%L3 = length of Link3 = 0.3m
%L4 = length of Link4 = 0.4m
DH_table = [0   -pi/2   L0           -pi/2     ;
            0   0       d1(t)+L2/2   0         ;   
            0   pi/2    L3         theta2(t) ;
            0   -pi     d3(t)-L4/2   pi/2     ];

%Compute Direct and Inverse kinematics (+ Jacobians)
jointStructure = ["P","R","P"]; 

[T, T_all, Ja, Jg, ik_joints] = computeKinematics(DH_table, jointStructure);

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
[toolbox_dk, my_dk, toolbox_ee_pose, my_ee_pose] = checkDirectKinematics(robot, config, T);
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
    
    [~, ~, goalPose, ~] = checkDirectKinematics(robot, randomConf, T);
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

            [~, ~, computedPoseFromJointParams, ~] = checkDirectKinematics(robot, currentConf, T);
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


%% Compute transformation matrix between Jg and Ja (base_link -> ee frames)
%Formula: Jg = Ta(phi) * Ja_complete
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
disp("Ja_complete computed from inv(Ta)*Jg:"); disp(Ja_complete);
% fprintf('%s\n', getLatexEquation("J_{a}", Ja_complete));




