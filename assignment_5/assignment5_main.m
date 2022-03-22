
clear; clc; close all;

%Add needed functions
addpath("../myFunctions/")
loadMyRobotStruct;

%Call the first assignment to compute the Jacobians and the transformation
%matrix between the two
assign1withoutPrints; clc;
assign3withoutPrints;
clearvars -except Ja_complete Jg Ta T_phi B C G %remove all the other variables
syms t L0 L2 L3 L4 m_Link2 m_Link3 m_Link4 real
syms d1(t) theta2(t) d3(t) Dd1(t) Dtheta2(t) Dd3(t) DDd1(t) DDtheta2(t) DDd3(t) ...
     x1(t) x2(t) x3(t) Dx1(t) Dx2(t) Dx3(t) DDx1(t) DDx2(t) DDx3(t) 

Ja = Ja_complete; %consider as analyitical jacobian the "complete" 6x3 one
q   = [d1(t), theta2(t), d3(t)].';        assumeAlso(q, 'real');
Dq  = [Dd1(t), Dtheta2(t), Dd3(t)].';     assumeAlso(Dq, 'real');
DDq = [DDd1(t), DDtheta2(t), DDd3(t)].';  assumeAlso(DDq, 'real');

% Dq = pinv(Ja)*Dx;               
DDx = Ja*DDq + diff(Ja, t)*Dq;
DDx = subs(DDx, [diff(d1(t), t), diff(theta2(t), t), diff(d3(t), t) ...
                 diff(Dd1(t), t), diff(Dtheta2(t), t), diff(Dd3(t), t)], ...
                [Dd1(t), Dtheta2(t), Dd3(t), DDd1(t), DDtheta2(t), DDd3(t)]);
DDx = simplify(DDx);

%Compute Ba
% Ba = inv(Ja*inv(B)*(Ja.')); %not simplified (returns an inf. matrix!)
Ba = (pinv(Ja).') * B * pinv(Ja);
Ba = subs(Ba, [conj(d1(t)),conj(theta2(t)),conj(d3(t)),conj(L4)], ... %why??
              [d1(t)      ,     theta2(t) ,      d3(t),     L4]);     %why??
Ba = simplify(Ba);

%Compute Ca*Dx
% Ca_Dx = Ba*Ja*inv(B)*C*Dq - Ba*diff(Ja, t)*Dq; %not simplified
Ca_Dx = pinv(Ja).'*C*Dq - Ba*diff(Ja, t)*Dq;
Ca_Dx = subs(Ca_Dx, [diff(d1(t), t), diff(theta2(t), t), diff(d3(t), t) ...
                     diff(Dd1(t), t), diff(Dtheta2(t), t), diff(Dd3(t), t)], ...
                    [Dd1(t), Dtheta2(t), Dd3(t), DDd1(t), DDtheta2(t), DDd3(t)]);
Ca_Dx = subs(Ca_Dx, [conj(d1(t)),conj(theta2(t)),conj(d3(t)),conj(L4)], ... %why??
                    [d1(t)      ,     theta2(t) ,      d3(t),     L4]);     %why??
Ca_Dx = simplify(Ca_Dx);

%Compute Ga
g = [0 -9.81 0].';

% Ga = Ba*Ja*inv(B)*g; %not simplified
Ga = pinv(Ja).'*g;
Ga = subs(Ga, [conj(d1(t)),conj(theta2(t)),conj(d3(t)),conj(L4)], ... %why??
              [d1(t)      ,     theta2(t) ,      d3(t),     L4]);     %why??
Ga = simplify(Ga);

%Compute u and u_e
syms f_x f_y f_z tau_x tau_y tau_z f_c_x f_c_y f_c_z tau_c_x tau_c_y tau_c_z

h_e = [f_x   f_y   f_z   tau_x   tau_y   tau_z].';
% %-- commands on f_y, tau_y and tau_z don't give any contribute --%
% h   = [f_c_x  0    f_c_z tau_c_x   0       0  ].'; 
%-- ∑0: commands on f_x, tau_y and tau_z don't give any contribute --%
h   = [0    f_c_y   f_c_z   0    0   tau_c_z ].'; 

u =   Ta.'*h;   %input command
u_e = Ta.'*h_e; %interaction with the environment


%Final expression: Ba(x)x+ Ca(x,Dx)Dx + Ga(x) = u - ue
leftTerm = Ba*DDx + Ca_Dx + Ga;

leftTerm = simplify(leftTerm);
leftTerm = symSubsZeros(leftTerm, 0.0001, 5);
rightTerm = symSubsZeros(u-u_e, 0.0001, 5);

dynamicEquation = leftTerm == rightTerm;
disp("The 6 equations retrieved from the Operational Space Dynamic model:")
disp(dynamicEquation)



%% Evaluation of a symbolic expression
%Import URDF file
robot = importrobot('../URDF/PRP_2_inertia.urdf');

loadStructuralParams; %load masses, distances...

f_c_x = 0; tau_c_x = 0; tau_c_y = 0; %those are zeros
f_c_y = 0;  f_c_z = 0; %set the command forces
tau_c_z = 0;           %set the command torque

fprintf("------- Evaluation of the equations -------\n\n")
jointValues = [0 0 0 0 0 0 0 0 0];
config = homeConfiguration(robot);
for i=1:3 config(i).JointPosition = jointValues(i); end
show(robot, config);

result = eval(subJointValues(dynamicEquation, jointValues)); %evaluates after substituting the jointValues
disp(result)

solutions = solve(result);
disp("The solutions are:");
fprintf("f_x = %.5f\n", solutions.f_x);
fprintf("f_y = %.5f\n", solutions.f_y);
fprintf("f_z = %.5f\n", solutions.f_z);
fprintf("tau_x = %.5f\n", solutions.tau_x);
fprintf("tau_y = %.5f\n", solutions.tau_y);
fprintf("tau_z = %.5f\n", solutions.tau_z);


disp("--------------------------")



%% LaTeX code...
fprintf("\nLaTeX equivalent code...\n")
fprintf("--------------------------\n")

% fprintf('%s\n', getLatexEquation("Ja", Ja));
% fprintf('%s\n', getLatexEquation("Ta", symSubsZeros(sym(Ta), 0.0001)));

syms m_Link2 m_Link3 m_Link4 L2 L3 L4 a_2 b_2 c_2 h_3 a_3 b_3 a_4 b_4 c_4  

Ba = subs(Ba, [m_Link2 m_Link3 m_Link4 L2 L3 L4 a_2 b_2 c_2 h_3 a_3 b_3 a_4 b_4 c_4] ...
            , [1 .5 .2 .4 .3 .4 .4 .03 .03 .3 .015 0 .4 .025 .025]);
Ba = symSubsZeros(Ba, 0.00001);
fprintf('%s\n', getLatexEquation("B_a(x)_{[1:6,1:2]}", Ba(1:6,1:2)));
fprintf('%s\n', getLatexEquation("B_a(x)_{[1:6,1:2]}", Ba(1:6,3:6)));
fprintf('%s\n', getLatexEquation("B_a(x)", Ba));

Ca_Dx = subs(Ca_Dx, [m_Link2 m_Link3 m_Link4 L2 L3 L4 a_2 b_2 c_2 h_3 a_3 b_3 a_4 b_4 c_4] ...
                  , [1 .5 .2 .4 .3 .4 .4 .03 .03 .3 .015 0 .4 .025 .025]);
Ca_Dx = symSubsZeros(Ca_Dx, 0.00001);

fprintf('%s\n', getLatexEquation("C_a\dot{x}", Ca_Dx));
fprintf('%s\n', getLatexEquation("G_a(x)", Ga));


fprintf("--------------------------\n")