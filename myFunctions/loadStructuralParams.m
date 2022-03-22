%--------------------------------------------------------------
%Set all the required params to evaluate symbolic expressions
%--------------------------------------------------------------

robotBodies = robot.Bodies;

m_Link2  = robotBodies{1,1}.Mass; %Link2 mass
m_Link3  = robotBodies{1,2}.Mass; %Link3 mass
m_Link4  = robotBodies{1,3}.Mass; %Link4 mass

a_2 = 0.4; b_2 = 0.03;  c_2 = 0.03;  %Link2
h_3 = 0.3; a_3 = 0.015; b_3 = 0;     %Link3
a_4 = 0.4; b_4 = 0.025; c_4 = 0.025; %Link4

L0 = 0.4; L2 = 0.4; L3 = 0.3; L4 = 0.4;

