
clear; clc; close all;

%Add needed functions
addpath("../myFunctions/")

%Import URDF file
robot = importrobot('../URDF/PRP_2_inertia.urdf');

%Call the assignment3 script
assign3withoutPrints  %get transformations from Σ0 !
CxDq = C*(Dq.');

%Load myRobotStruct
loadMyRobotStruct;

%Concatenate inertia matrices
I_all(:,:,1) = I_2; I_all(:,:,2) = I_3; I_all(:,:,3) = I_4; 

%Compute the G matrix
disp("Computing the gravity G matrix...");
showPrints = true;
q = [d1(t) theta2(t) d3(t)].';  %syms position col. vector    
Dq  = sym([0 0 0]).';           %syms velocity col. vector
DDq = sym([0 0 0]).';           %syms acceleration col. vector
g0 = sym([0 -9.81 0]).';  
DDp0_g0 = sym([0 0 0]).' - g0;      
G_NE = NE(q, Dq, DDq, DDp0_g0, I_all, T_all, showPrints);

%Compute the C*Dq vector
disp("Computing the C*Dq vector...");
showPrints = true;
q =  [d1(t) theta2(t) d3(t)].';     %syms position col. vector    
Dq = [Dd1(t) Dtheta2(t) Dd3(t)].';  %syms velocity col. vector    
DDq = sym([0 0 0]).';               %syms acceleration col. vector
g0 = sym([0 0 0]).';  
DDp0_g0 = sym([0 0 0]).' - g0;      
CxDq_NE = NE(q, Dq, DDq, DDp0_g0, I_all, T_all, showPrints);

%Compute the B matrix
disp("Computing the inertia B matrix...");
showPrints = false;
B_NE = sym(zeros(3));
q =  [d1(t) theta2(t) d3(t)].';     %syms position col. vector    
Dq  = sym([0 0 0]).';               %syms velocity col. vector 
g0 = sym([0 0 0]).';  
DDp0_g0 = sym([0 0 0]).' - g0;  
for k = 1:3
    DDq = sym([0 0 0]).';           %syms acceleration col. vector
    DDq(k) = 1;
    B_NE(:,k) = NE(q, Dq, DDq, DDp0_g0, I_all, T_all, showPrints);
end    

%Compute the tau torques
showPrints = false;
q =  [d1(t) theta2(t) d3(t)].';        %syms position col. vector    
Dq  = [Dd1(t) Dtheta2(t) Dd3(t)].';    %syms velocity col. vector 
DDq = [DDd1(t) DDtheta2(t) DDd3(t)].'; %syms acceleration col. vector
g0 = sym([0 -9.81 0]).';  
DDp0_g0 = sym([0 0 0]).' - g0;  
tau_NE = NE(q, Dq, DDq, DDp0_g0, I_all, T_all, showPrints);


%% To compare the resulting matrices
% clearvars -except B B_NE CxDq C CxDq_NE G G_NE tau tau_NE %remove all the other variables
B = expand(B); C = expand(C); G = expand(formula(G));

disp("------------------------------------------------------------");
disp("Inertia B matrix computed using the Lagrange's method"); disp(B)
disp("Inertia B matrix computed using the Newton-Euler's method"); disp(B_NE)
isequaln(B, B_NE) %to see if they're equal

disp("C(q,Dq)*Dq vector computed using the Lagrange's method"); disp(CxDq)
disp("C(q,Dq)*Dq vector computed using the Newton-Euler's method"); disp(CxDq_NE)
isequaln(CxDq, CxDq_NE) %to see if they're equal

disp("G(q) matrix computed using the Lagrange's method"); disp(G)
disp("G(q) matrix computed using the Newton-Euler's method"); disp(G_NE)
isequaln(G, G_NE) %to see if they're equal

disp("Torques computed using the Lagrange's method"); disp(tau)
disp("Torques computed using the Newton-Euler's method"); disp(tau_NE)
disp("------------------------------------------------------------");

fprintf("Tau1 Lagrange == Tau1 NE?  " + int2str(isequaln(expand(tau(1)), expand(tau_NE(1))))+ "\n")
fprintf("Tau2 Lagrange == Tau2 NE?  " + int2str(isequaln(expand(tau(2)), expand(tau_NE(2))))+ "\n")
fprintf("Tau3 Lagrange == Tau3 NE?  " + int2str(isequaln(expand(tau(3)), expand(tau_NE(3))))+ "\n")
disp("------------------------------------------------------------");

%% Latex code...
% disp(getLatexEquation("T^4_{e-e}", T_all(:,:,5)))
% disp(getLatexEquation("T^4_5", T4_5))


%% To compute matrices using Newton-Euler
function tau_NE = NE(q, Dq, DDq, DDp0, I_all, T_all, showPrints)

    %Set the initial conditions
    f_n_1 = sym([0 0 0]).';   %slides: f^{n+1}_{n+1}
    mu_n_1 = sym([0 0 0]).';  %slides: mu^{n+1}_{n+1}
    z0 = sym([0 0 1]).';
    omega0 = sym([0 0 0]).';
    Domega0 = sym([0 0 0]).';
    n = 3; %PRP robot's degrees of freedom
    jointStructure = ['P', 'R', 'P'];
    tau_NE = sym(zeros(3,1)); 
        
    %Initialize all the variables
    syms L2 L3 L4 d1(t) theta2(t) d3(t) m_Link2 m_Link3 m_Link4 t
    m_Linki = [m_Link2 m_Link3 m_Link4]; %symbolic masses

    r_C_Linki = [0   0   -L2/2  ; %slides: r^2_{2,C2}
                 0  L3/2   0    ; %slides: r^3_{3,C3}
                 0   0   -L4/2 ]; %slides: r^4_{4,C4}

    omega  = sym(zeros(3,3));  %3x3 matrix
    Domega = sym(zeros(3,3));  %3x3 matrix  
    DDp    = sym(zeros(3,3));  %3x3 matrix
    DDp_C  = sym(zeros(3,3));  %3x3 matrix

    %Compute all the transformation matrices that are required during the
    %Forward and Backward N.E. computation.
    %This property is used: T1_2 = inv(T0_1)*T0_2 (etc...)
    T0_1 = T_all(:,:,1);                                R0_1 = T0_1(1:3, 1:3);
    T1_2 = simplify( inv(T_all(:,:,1))*T_all(:,:,2) );  R1_2 = T1_2(1:3, 1:3);
    T2_3 = simplify( inv(T_all(:,:,2))*T_all(:,:,3) );  R2_3 = T2_3(1:3, 1:3);
    T3_4 = simplify( inv(T_all(:,:,3))*T_all(:,:,4) );  R3_4 = T3_4(1:3, 1:3);
 
    T_int = cat(3, T0_1, T1_2, T2_3, T3_4); %concatenate "intermidiate" transformations
    R_int = cat(3, R0_1, R1_2, R2_3, R3_4); %concatenate "intermidiate" rotation matrices

    %Compute the required translation vectors from a reference frame to another
    r0_1 = R0_1.'*T0_1(1:3, 4);   r1_2 = R1_2.'*T1_2(1:3, 4);
    r2_3 = R2_3.'*T2_3(1:3, 4);  
    r_int = cat(3, r0_1, r1_2, r2_3); %concatenate intermidiate translation vectors

    %% FORWARD Newton-Euler computation     
    if showPrints 
        disp("------------------------------")
        fprintf("Starting the Forward Newton-Euler routine...\n");
    end
    
    for i = 1:n %start FORWARD Newton-Euler computation
    
        if showPrints 
            fprintf("[FNE] Considering link %d ", i+1); 
            if jointStructure(i) == 'P'                
                fprintf("PRISMATIC joint\n");
            else
                fprintf("REVOLUTE joint\n");
            end
        end
        r_Ci = r_C_Linki(i,:).';   %col. vector from O(i) to C.o.M. Ci

        if i-1 == 0 %first iteration (prismatic in my case)
            omega(:,i)  = R_int(:,:,i).' * omega0;           
            Domega(:,i) = R_int(:,:,i).' * Domega0;

            %Dq = Dd1; DDq = DDd1
            Dd1 = Dq(i); DDd1 = DDq(i); %col. vectors

            DDp(:,i)    = R_int(:,:,i).' * DDp0 + cross(Domega(:,i), r_int(:,:,i)) + ...
                          cross(omega(:,i), cross(omega(:,i), r_int(:,:,i)));    
            DDp(:,i) = DDp(:,i) + R_int(:,:,i).' * DDd1 * z0 + cross(2*Dd1*omega(:,i), (R_int(:,:,i).'*z0));

            DDp_C(:,i) = DDp(:,i) + cross(Domega(:,i), r_Ci) + cross(omega(:,i), cross(omega(:,i), r_Ci));
        
        else %i-1 != 0
            omega(:,i)  = R_int(:,:,i).' * omega(:,i-1);           
            Domega(:,i) = R_int(:,:,i).' * Domega(:,i-1);

            DDp(:,i)    = R_int(:,:,i).' * DDp(:,i-1) + cross(Domega(:,i), r_int(:,:,i)) + ...
                          cross(omega(:,i), cross(omega(:,i), r_int(:,:,i)));

            %Dqi = Dtheta2; DDqi = DDtheta2  or  Dqi = Dd3; DDqi = DDd3 
            Dqi = Dq(i); DDqi = DDq(i); %col. vectors    
            if jointStructure(i) == 'R'     
                omega(:,i) = omega(:,i) + R_int(:,:,i).'*Dqi*z0;
                Domega(:,i) = Domega(:,i) + R_int(:,:,i).'*(DDqi*z0 + cross(Dqi*omega(:,i-1), z0));
            elseif jointStructure(i) == 'P'
                DDp(:,i) = DDp(:,i) + R_int(:,:,i).' * DDqi * z0 + cross(2*Dqi*omega(:,i), (R_int(:,:,i).'*z0));
            end    
            
            DDp_C(:,i) = DDp(:,i) + cross(Domega(:,i), r_Ci) + cross(omega(:,i), cross(omega(:,i), r_Ci));
            %---------------------------%
        end
        if showPrints fprintf("Finished the Forward Newton-Euler routine...\n"); end  
    end %end of the FORWARD Newton-Euler computation

    %% BACKWARD Newton-Euler computation
    if showPrints disp("Starting the Backward Newton-Euler routine..."); end
    
    f  = f_n_1;  %slides: f^{n+1}_{n+1}
    mu = mu_n_1; %slides: μ^{n+1}_{n+1}      
    for i = n:-1:1 %start BACKWARD Newton-Euler computation
        
        if showPrints 
            fprintf("[FNE] Considering link %d ", i+1); 
            if jointStructure(i) == 'P'                
                fprintf("PRISMATIC joint\n");
            else
                fprintf("REVOLUTE joint\n");
            end
        end        
 
        r_Ci = r_C_Linki(i,:).';   %vector from O(i) to C.o.M. Ci (slides: r^i_{i,Ci})

        %Compute the "f^i_i" force (R_int(:,:,i+1) = R^i_{i+1}})
        f_i = R_int(:,:,i+1) * f  +  m_Linki(i) * DDp_C(:,i); %slides: f^i_i

        %Compute the "μ^i_i" torque
        mu_i = -cross(f_i, (r_int(:,:,i) + r_Ci)) + R_int(:,:,i+1)*mu + cross(R_int(:,:,i+1)*f, r_Ci) + ...
               I_all(:,:,i)*Domega(:,i)    + cross(omega(:,i), I_all(:,:,i)*omega(:,i));
    
        if jointStructure(i) == 'P'
            tau_NE(i) = simplify( f_i.'  * R_int(:,:,i).' * z0 ); %neglecting motor and friction
        elseif jointStructure(i) == 'R'
            tau_NE(i) = simplify( mu_i.'  * R_int(:,:,i).' * z0 ); %neglecting motor and friction
        end
        
        f = f_i;   %UPDATE f
        mu = mu_i; %UPDATE mu
        %---------------------------%       
    end %end of the BACKWARD Newton-Euler computation
    if showPrints
        fprintf("Finished the Backward Newton-Euler routine...\n"); 
        fprintf("------------------------------\n\n")
    end
end

