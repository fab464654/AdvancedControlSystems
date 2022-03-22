function [toolbox_dk, my_dk, toolbox_ee_pose, my_ee_pose] = checkDirectKinematics(robot, myRobot, config, T, startFrame)
    %This function substitutes the given "config" joint parameters' values
    %inside the transformation matrix "T"
    syms L0 L2 L3 L4 d1(t) theta2(t) d3(t) t
    [q1, q2, q3] = config.JointPosition;
   
    %Important: ee and base_link must be reversed!!!
    if strcmp(startFrame, "0")
        toolbox_dk = getTransform(robot,config,"ee","base_link");
        T_base_to_0 = myRobot.T_base_to_0_DH; %retrive (DH) transf. from ∑base to ∑0
        T_0_to_base = subs(inv(T_base_to_0), L0, 0.4);        
        toolbox_dk = T_0_to_base*toolbox_dk;
    elseif strcmp(startFrame, "base")
        %IMPORTANT: - the above adjustment isn't required because my ∑base is
        %             equal to the URDF ∑base!!
        toolbox_dk = getTransform(robot,config,"ee","base_link");
        T_base_to_0 = myRobot.T_base_to_0_DH; %retrive (DH) transf. from ∑base to ∑0
        T_0_to_base = inv(T_base_to_0);
    end
    
    toolbox_ee_pose = toolbox_dk(1:3,4);
    
    my_dk = subs(T, [L0 L2 L3 L4 d1(t) theta2(t) d3(t)], [0.4,0.4,0.3,0.4, q1, q2, q3]);
    my_ee_pose = my_dk(1:3,4); 
end

