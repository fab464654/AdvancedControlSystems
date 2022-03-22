
%Add needed functions
addpath("../myFunctions/")

%Import URDF file
robot = importrobot('../URDF/PRP_2.urdf');

%Load the robot's information
loadMyRobotStruct;

config = homeConfiguration(robot);


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


