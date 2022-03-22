

%Import URDF file
robot = importrobot('../URDF/PRP_2_inertia.urdf');

syms m_Link2 m_Link3 m_Link4

%Compute baricentral and total inertia tensors (wrt base frame!)
% a = 0.4; b = 0.03; c = 0.03; %length / height / width
syms a_2 b_2 c_2
Ic_Link2 = diag( 1/12 * m_Link2 * [(a_2^2+b_2^2), (a_2^2+c_2^2), (b_2^2+c_2^2)]);
I_2 = Ic_Link2 + m_Link2 * ([0 0 -a_2/2]*[0 0 -a_2/2].'*eye(3)-[0 0 -a_2/2].'*[0 0 -a_2/2]);

syms a_3 b_3 h_3
% h = 0.3; a = 0.015; b = 0;   %length / outer radius / inner radius = 0 (not hollow body)
Ic_Link3 = diag( m_Link3 * [1/4*a_3^2 + 1/12*h_3^2, 1/4*a_3^2 + 1/12*h_3^2, 1/2*a_3^2]);
I_3 = Ic_Link3 - m_Link3 * ([0 0 -h_3/2]*[0 0 -h_3/2].'*eye(3)-[0 0 -h_3/2].'*[0 0 -h_3/2]);

syms a_4 b_4 c_4
% a = 0.4; b = 0.025; c = 0.025; %length / height / width
Ic_Link4 = diag( 1/12 * m_Link4 * [(a_4^2+b_4^2), (a_4^2+c_4^2), (b_4^2+c_4^2)]);
I_4 = Ic_Link4 + m_Link4 * ([0 0 +a_4/2]*[0 0 +a_4/2].'*eye(3)-[0 0 +a_4/2].'*[0 0 +a_4/2]);


%According to the theory: p_dot_Li = J_Li_P(q)*q_dot;
syms L0 L2 L3 L4 d1(t) theta2(t) d3(t) t

%Load useful PRP robot parameters
loadMyRobotStruct;

%IMPORTANT: the user MUST choose the overall ref. wrt ∑0 or ∑base!!!
startFrame = "0";  % ∑0 = "0"; ∑base = "base"

%Initialize the DH table and p_Li coordinates
if strcmp(startFrame, "0")
    DH_table = myRobot.DH_table_zero; 
    %Get all the transformation matrices
    [T, T_all] = computeDirectKinematics(DH_table);
  
    p_L2 = [0; 0; d1(t)];           %MANUALLY ASSIGN p_L2
    p_L3 = [0; 0; d1(t)+L2/2+L3/2]; %MANUALLY ASSIGN p_L3
    p_L4 = [-d3(t)*sin(theta2(t)); +d3(t)*cos(theta2(t)); L2/2+L3+d1(t)]; %MANUALLY ASSIGN p_L4
    %to verify p_L4:
    %vpa(subs(p_L4, [L0, L2, L3, d1(t), theta2(t), d3(t)], [0.4 0.4 0.3 config.JointPosition]))
    R_2 = T_all(1:3,1:3,1);
    R_3 = T_all(1:3,1:3,2);
    R_4 = T_all(1:3,1:3,3);
    
elseif strcmp(startFrame, "base")
    DH_table = myRobot.DH_table_base; 
    %Get all the transformation matrices
    [T, T_all] = computeDirectKinematics(DH_table);
    
    p_L2 = [d1(t); 0; L0]; %MANUALLY ASSIGN p_L2
    p_L3 = [d1(t)+L2/2+L3/2; 0; L0]; %MANUALLY ASSIGN p_L3
    p_L4 = [L2/2+L3+d1(t); -d3(t)*sin(theta2(t)); L0+d3(t)*cos(theta2(t))]; %MANUALLY ASSIGN p_L4
    %to verify p_L4:
    %vpa(subs(p_L4, [L0, L2, L3, d1(t), theta2(t), d3(t)], [0.4 0.4 0.3 config.JointPosition]))
    R_2 = T_all(1:3,1:3,2);
    R_3 = T_all(1:3,1:3,3);
    R_4 = T_all(1:3,1:3,4);
end


%Get partial Jacobians and kinetic energy of each link
syms Dd1(t) Dtheta2(t) Dd3(t) 
q_dot = [Dd1(t), Dtheta2(t), Dd3(t)].';

%------- Link 2 -------%
i = 2; %Link 2
[J_P_L2, J_O_L2] = getPartialJacobian(T_all, myRobot.jointStructure, p_L2, i, startFrame);
[T_L2, B_q2] = computeKineticEnergy(m_Link2, J_P_L2, J_O_L2, I_2, q_dot, R_2);


%------- Link 3 -------%
i = 3; %Link 3
[J_P_L3, J_O_L3] = getPartialJacobian(T_all, myRobot.jointStructure, p_L3, i, startFrame);
[T_L3, B_q3] = computeKineticEnergy(m_Link3, J_P_L3, J_O_L3, I_3, q_dot, R_3);


%------- Link 4 -------%
i = 4; %Link 4
[J_P_L4, J_O_L4] = getPartialJacobian(T_all, myRobot.jointStructure, p_L4, i, startFrame);
[T_L4, B_q4]  = computeKineticEnergy(m_Link4, J_P_L4, J_O_L4, I_4, q_dot, R_4);


%Compute the Potential energy of each link
if strcmp(startFrame, "base")
    g = [0 0 -9.81].';
elseif strcmp(startFrame, "0")
    g = [0 -9.81 0].';
end

%------- Link 2 -------%
U_L2 = computePotentialEnergy(m_Link2, g, p_L2);

%------- Link 3 -------%
U_L3 = computePotentialEnergy(m_Link3, g, p_L3);

%------- Link 4 -------%
U_L4 = computePotentialEnergy(m_Link4, g, p_L4);
%--------------------------------------



%% Computing the "total" entities

%Total computation of T and B(q)
T_total = simplify(T_L2 + T_L3 + T_L4);
B_q_total = simplify(B_q2 + B_q3 + B_q4);

%Total computation of U
U_total = U_L2 + U_L3 + U_L4;
%--------------------------------------







