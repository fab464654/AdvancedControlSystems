
clear; clc; close all;

%Add needed functions
addpath("../myFunctions/")

%Import URDF file
robot = importrobot('../URDF/PRP_2_inertia.urdf');

% sympref('FloatingPointOutput',false)

%Show information about the structure
showdetails(robot)

%Compute Kinetic and Potential energies
assign2withoutPrints %call the second assignment main script,
                     %with SYMBOLIC values!

syms q_dot(t) Dd1(t) Dtheta2(t) Dd3(t)
q_dot(t) = [Dd1(t), Dtheta2(t), Dd3(t)].';

%Compute the Lagrange function
L = T_total - U_total;

%Compute dL/d(q_dot)
dL_dq_dot = [diff(L, Dd1);
             diff(L, Dtheta2);
             diff(L, Dd3)];
dL_dq_dot = simplify(dL_dq_dot);
fprintf("--------------------------------------\n");
disp("dL_dq_dot:"); 
fprintf("--------------------------------------\n");
disp(dL_dq_dot);

%% TEST EQUALITY of expressions
%Equivalent to:
%dL_dq_dot = B_q_total * q_dot  
d_temp = B_q_total * q_dot;  %remove ";"
dL_dq_dot;                   %remove ";"
%%

%Compute d(dL_dq_dot)/dt
D_dL_dq_dot = diff(dL_dq_dot(t), t).';

%Needed to substitute diff(d3(t), t) with Dd3(t) etc..
D_dL_dq_dot = subs(D_dL_dq_dot, [diff(d1(t), t), diff(theta2(t), t), diff(d3(t), t)],...
                   [Dd1(t), Dtheta2(t), Dd3(t)]);
%Needed to substitute diff(Dd3(t), t) with DDd3(t) etc..
syms DDd1(t) DDtheta2(t) DDd3(t) t
D_dL_dq_dot = subs(D_dL_dq_dot, [diff(Dd1(t), t), diff(Dtheta2(t), t), diff(Dd3(t), t)],...
                [DDd1(t), DDtheta2(t), DDd3(t)]);
            
disp("--------------------------------------");
disp("D_dL_dq_dot:"); 
disp("--------------------------------------");
disp(D_dL_dq_dot.');

%Compute dL/d(q)
dL_dq = [diff(L, d1), diff(L, theta2), diff(L, d3)]; 
disp("--------------------------------------");
disp("dL_dq:"); 
disp("--------------------------------------");
disp(dL_dq.');


%% Solve the 3 Lagrange Equations:   
%Compute the 3 torques
tau = simplify( D_dL_dq_dot.' - dL_dq.' );
tau = formula(tau);
disp("--------------------------------------");
disp("The 3 Lagrange equations:")
disp("--------------------------------------");
disp(tau)              
    
%Compute the C matrix (useful for the assignment 4)
B = B_q_total; %to soften the notation
disp("--------------------------------------");
disp("B(q) matrix (from assignment 2):"); 
disp("--------------------------------------");
disp(B);

syms d1(t) theta2(t) d3(t) Dd1(t) Dtheta2(t) Dd3(t) t
q = [d1(t), theta2(t), d3(t)];
Dq = [Dd1(t), Dtheta2(t), Dd3(t)];

C_cycles = sym(zeros(3,3));
for i=1:3
    for j=1:3
        for k=1:3
            C_cycles(i,j) = C_cycles(i,j) + 1/2 * (  + diff(B(i,j), q(k)) ...
                                                     + diff(B(i,k), q(j)) ...
                                                     - diff(B(j,k), q(i))) * Dq(k);
        end        
    end
end
disp("--------------------------------------");
disp("C(q, q_dot) matrix (cycle computation):"); 
disp("--------------------------------------");
disp(C_cycles);

%Computation of the G matrix, would be the same extracting from "tau" the
%terms depending on "q"
G = [diff(U_total, d1); diff(U_total, theta2); diff(U_total, d3)];

disp("--------------------------------------");
disp("G(q) gravity term:"); 
disp("--------------------------------------");
disp(G); 


%% Get LaTeX equivalent code
% disp(getLatexEquation("C(q, \dot{q})", C_cycles))
% disp(getLatexEquation("G(q)", G))
% 
% 
% disp(getLatexEquation("\tau_{1}", tau(1,1)))
% disp(getLatexEquation("\tau_{2}", tau(2,1)))
% disp(getLatexEquation("\tau_{3}", tau(3,1)))
% 
% disp(getLatexEquation("L", collect(L, 981/100)))
% disp(getLatexEquation("", D_dL_dq_dot))
% disp(getLatexEquation("", dL_dq))
% 
% disp(getLatexEquation("B", B))







