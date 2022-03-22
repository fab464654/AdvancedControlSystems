%This script can be used to compute direct and inverse kinematics of
%my PRP manipulator
%Input: the DH table in the following form:
%   syms L0 L3 d1 theta2 d3 %d1, theta2, d3 joint parameters
%   DH_table = [a(i-1)  alpha(i-1)   d(i)  theta(i)  ; 
%               a(i-1)  alpha(i-1)   d(i)  theta(i)  ;
%               a(i-1)  alpha(i-1)   d(i)  theta(i)  ];
%
%Output: T, T_all, Ja, Jg matrices 

function [T, T_all] = computeDirectKinematics(DH_table)
    %This is specific for my robot structure
    syms d1(t) theta2(t) d3(t) t L0 L2 L3 L4     
    a     = DH_table(:,1);
    alpha = DH_table(:,2);
    d     = DH_table(:,3);
    theta = DH_table(:,4);

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
end