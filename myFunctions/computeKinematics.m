%This script can be used to compute direct and inverse kinematics of
%my PRP manipulator
%Input: the DH table in the following form:
%   syms L0 L3 d1 theta2 d3 %d1, theta2, d3 joint parameters
%   DH_table = [a(i-1)  alpha(i-1)   d(i)  theta(i)  ; 
%               a(i-1)  alpha(i-1)   d(i)  theta(i)  ;
%               a(i-1)  alpha(i-1)   d(i)  theta(i)  ];
%
%Output: T, T_all, Ja, Jg matrices and InverseKinematics results

function [T, T_all, Ja, Jg, ik_joints] = computeKinematics(DH_table, jointStructure, startFrame)
    syms d1(t) theta2(t) d3(t) t %This is specific for my robot structure
    a     = DH_table(:,1);
    alpha = DH_table(:,2);
    d     = DH_table(:,3);
    theta = DH_table(:,4);

    fprintf("1) Computing direct kinematics...\n");
    T = eye(4); %4x4 initialization
    T_all = sym(cat(3,eye(4),eye(4),eye(4),eye(4))); %4x4x4, contains all transformations

    for i=1:size(theta, 1)    
        DH_matrix = [cos(theta(i)), -sin(theta(i))*cos(alpha(i)),  sin(theta(i))*sin(alpha(i)), a(i)*cos(theta(i));
                     sin(theta(i)),  cos(theta(i))*cos(alpha(i)), -cos(theta(i))*sin(alpha(i)), a(i)*sin(theta(i));
                           0      ,           sin(alpha(i))     ,           cos(alpha(i))     ,       d(i)        ;
                           0      ,               0             ,               0             ,        1         ];

        T = T*DH_matrix;  %update the total transformation 

        %Remove very small coefficient (according to the threshold)
        threshold = 0.00001;
        T = symSubsZeros(T, threshold);  

        T_all(:,:,i) = T;   %save the current transformation 
    end
    
    fprintf("[DirectKin] Final transformation matrix (direct kinematics):\n"); disp(T);
    fprintf("[DirectKin] Generic pose of the end effector:\n"); disp(T(1:3,4));

    fprintf("2) Computing the joint parameters, given px and py (inverse kinematics)...\n");
    syms p_x p_y p_z
    eq1 = p_x == T(1,4);
    eq2 = p_y == T(2,4);
    eq3 = p_z == T(3,4);

    [d1_sol, theta2_sol, d3_sol] = solve([eq1 eq2 eq3], [d1(t) theta2(t) d3(t)]); %solve the system
    ik_joints = [d1_sol, theta2_sol, d3_sol]; %to be returned once the function is called
    
    fprintf("[InverseKin] Solutions in the following form: [d1(t), theta2(t), d3(t)]\n");
    for i=1:size(ik_joints,1) %show the solution
        fprintf('%s\n', getLatexEquation("d_1(t)", simplify(d1_sol(i))));
        fprintf('%s\n', getLatexEquation("theta_2(t)", simplify(theta2_sol(i))));
        fprintf('%s\n', getLatexEquation("d_3(t)", simplify(d3_sol(i))));

        solution = char([simplify(d1_sol(i)), simplify(theta2_sol(i)), simplify(d3_sol(i))]);
        fprintf("[InverseKin] Solution n." + i + ": " + solution + "\n");  
    end
    
    fprintf("\n3) Computing the analytical Jacobian (linear part)...\n");
    p_e = T(1:3,4); %end effector's position  
    Ja(:,1) = diff(p_e,d1);
    Ja(:,2) = diff(p_e,theta2);
    Ja(:,3) = diff(p_e,d3);
    Ja = simplify(Ja);
    disp(Ja);
    %the above code is equivalent to:
    %jacobian(p_e)
    
    if strcmp(startFrame, "0")
        fprintf("\n4) Computing the Geometric Jacobian wrt ∑0...\n");
    elseif strcmp(startFrame, "base")
        fprintf("\n4) Computing the Geometric Jacobian wrt ∑base...\n");
    end
    
    %T_all(1) = T from ∑0/∑base to ∑1/∑0
    z0 = sym([0 0 1]');
    for i=1:3 %myRobot.dof = 3        
        if strcmp(startFrame, "0") 
            if i == 1 
                z = z0; %if we start from ∑base it's not correct
            else
                R_0_i_1 = T_all(1:3,1:3,i-1); %rotm from ∑i-1 to ∑i
                p_i_1 = T_all(1:3,4,i-1);     %p from ∑i-1 to ∑i
                p_i_1_e = p_e - p_i_1;
                z = R_0_i_1 * z0; 
            end
            if jointStructure(i) == "R" %revolute joint 
                Jg(1:3, i) = cross(z, p_i_1_e);
                Jg(4:6, i) = z;
            elseif jointStructure(i) == "P" %prismatic joint
                Jg(1:3, i) = z;
                Jg(4:6, i) = sym([0 0 0]');
            end
        elseif strcmp(startFrame, "base") 
            R_0_i = T_all(1:3,1:3,i);
            p_i = T_all(1:3,4,i);
            p_i_e = p_e - p_i;
            z = R_0_i * sym([0 0 1]');


            if jointStructure(i) == "R" %revolute joint 
                Jg(1:3, i) = cross(z, p_i_e);
                Jg(4:6, i) = z;
            elseif jointStructure(i) == "P" %prismatic joint
                Jg(1:3, i) = z;
                Jg(4:6, i) = sym([0 0 0]');
            end
        end
    end
    Jg = simplify(Jg);
    disp(Jg)
end