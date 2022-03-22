function [J_P, J_O] = getPartialJacobian(T_all, jointStructure, p_Li, i, startFrame)
    %This function computes the Partial Jacobian, according to the theory
    %formulations
    
    n = 3; %number of links (i=1:n)
    
    if strcmp(startFrame, "base")
        for j=1:n        
            if j > (i-1) %according to the theory (beacause i=2 --> joint 1)
                J_O(:, j) = [0 0 0]';
                J_P(:, j) = [0 0 0]';
            else
                R_0_j_1 = T_all(1:3,1:3,j);      %R from ∑base to ∑j-1
                p_j_1 = T_all(1:3, 4, j);        %p(j-1)
                z_j_1 = R_0_j_1 * sym([0 0 1]'); %z(j-1)

                if jointStructure(j) == "R" %revolute joint 
                    J_P(1:3, j) = cross(z_j_1, (p_Li - p_j_1));
                    J_O(1:3, j) = z_j_1;
                elseif jointStructure(j) == "P" %prismatic joint
                    J_P(1:3, j) = z_j_1;
                    J_O(1:3, j) = sym([0 0 0]');
                end
            end
        end    
    elseif strcmp(startFrame, "0") 
        for j=1:n    
            if j > (i-1) %according to the theory (beacause i=2 --> joint 1)
                J_O(:, j) = sym([0 0 0]');
                J_P(:, j) = sym([0 0 0]');
            else
                if j == 1
                    R_0_j_1 = eye(3);      %R from ∑0 to ∑0
                    p_j_1 = sym([0 0 0]');        %p(j-1)
                    z_j_1 = R_0_j_1 * sym([0 0 1]');   %z(j-1)
                else
                    R_0_j_1 = T_all(1:3,1:3,j-1);      %R from ∑0 to ∑j-1 (MATLAB indices problem!)
                    p_j_1 = T_all(1:3, 4, j-1);        %p(j-1)
                    z_j_1 = R_0_j_1 * sym([0 0 1]');   %z(j-1)
                   
                end
                if jointStructure(j) == "R" %revolute joint 
                    J_P(1:3, j) = cross(z_j_1, (p_Li - p_j_1));
                    J_O(1:3, j) = z_j_1;
                elseif jointStructure(j) == "P" %prismatic joint
                    J_P(1:3, j) = z_j_1;
                    J_O(1:3, j) = sym([0 0 0]');
                end
            end
        end 
    end
    J_P = simplify(J_P);   
    J_O = simplify(J_O); 
    
    
end
    
    
    
    
%     %T_all(1) = T from ∑0/∑base to ∑1/∑0
%     z0 = sym([0 0 1]');
%     for i=1:3 %myRobot.dof = 3        
%         if strcmp(startFrame, "0") 
%             if i == 1 
%                 z = z0; %if we start from ∑base it's not correct
%             else
%                 R_0_i_1 = T_all(1:3,1:3,i-1); %rotm from ∑i-1 to ∑i
%                 p_i_1 = T_all(1:3,4,i-1);     %p from ∑i-1 to ∑i
%                 p_i_1_e = p_e - p_i_1;
%                 z = R_0_i_1 * z0; 
%             end
%             if jointStructure(i) == "R" %revolute joint 
%                 Jg(1:3, i) = cross(z, p_i_1_e);
%                 Jg(4:6, i) = z;
%             elseif jointStructure(i) == "P" %prismatic joint
%                 Jg(1:3, i) = z;
%                 Jg(4:6, i) = sym([0 0 0]');
%             end
%         elseif strcmp(startFrame, "base") 
%             R_0_i = T_all(1:3,1:3,i);
%             p_i = T_all(1:3,4,i);
%             p_i_e = p_e - p_i;
%             z = R_0_i * sym([0 0 1]');
% 
% 
%             if jointStructure(i) == "R" %revolute joint 
%                 Jg(1:3, i) = cross(z, p_i_e);
%                 Jg(4:6, i) = z;
%             elseif jointStructure(i) == "P" %prismatic joint
%                 Jg(1:3, i) = z;
%                 Jg(4:6, i) = sym([0 0 0]');
%             end
%         end
%     end


