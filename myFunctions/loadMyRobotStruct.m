%--------------------------------------------------------------
%Set all the required params to evaluate symbolic expressions
%--------------------------------------------------------------

robot = importrobot('../URDF/PRP_2_inertia.urdf');
robotBodies = robot.Bodies;

myRobot.jointStructure = ["P", "R", "P"];
myRobot.jointLimits = [-0.2 0.2;
                       -pi  pi ;
                       -0.2 0.2];
                   
syms L0 L2 L3 L4 d1(t) theta2(t) d3(t) m_Link2 m_Link3 m_Link4 t
%Transformation matrices from ∑base to ∑0 ACCORDING TO:
% - THE URDF FRAMES: myRobot.T_base_to_0_URDF 
% - THE DH CONVENTION FRAMES: myRobot.T_base_to_0_DH
myRobot.T_base_to_0_URDF =  [0, -1,  0, 0   ; %90° arond z-axis
                             1,  0,  0, 0   ;
                             0,  0,  1, 0.1 ; %0.1 transl. due to the placement of URDF ∑0
                             0,  0,  0, 1   ];
myRobot.T_base_to_0_DH = [ 0,  0, 1,  0;
                          1,  0, 0,  0;
                           0, 1, 0, L0;
                           0,  0, 0,  1];
                    
             %DH table:  a   alpha   d            theta
myRobot.DH_table_base = [0   pi/2   L0            pi/2     ;
                         0   0       d1(t)+L2/2   0         ;   
                         0   -pi/2    L3           theta2(t) ;
                         0   -pi     d3(t)-L4/2   -pi/2     ];
                                          
             %DH table:  a   alpha   d            theta
myRobot.DH_table_zero = [0   0       d1(t)+L2/2   0         ;   
                         0   -pi/2    L3           theta2(t) ;
                         0   -pi     d3(t)-L4/2   -pi/2     ];
                                
myRobot.m_Link2  = robotBodies{1,1}.Mass; %Link2 mass
myRobot.m_Link3  = robotBodies{1,2}.Mass; %Link3 mass
myRobot.m_Link4  = robotBodies{1,3}.Mass; %Link4 mass

myRobot.a_2 = 0.4; myRobot.b_2 = 0.03;  myRobot.c_2 = 0.03;  %Link2
myRobot.h_3 = 0.3; myRobot.a_3 = 0.015; myRobot.b_3 = 0;     %Link3
myRobot.a_4 = 0.4; myRobot.b_4 = 0.025; myRobot.c_4 = 0.025; %Link4

myRobot.L0 = 0.4; 
myRobot.L2 = 0.4; 
myRobot.L3 = 0.3; 
myRobot.L4 = 0.4;

myRobot.dof = 3;
% myRobot.q0 = [0 pi/2 0].'; %to check what happens on theta2
myRobot.q0 = [0 0 0].';
myRobot.Dq0 = [0 0 0].';

