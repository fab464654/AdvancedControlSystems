function [toolbox_dk, my_dk, toolbox_ee_pose, my_ee_pose] = checkDirectKinematics(robot, config, T, startFrame)
    %This function substitutes the given "config" joint parameters' values
    %inside the transformation matrix "T"
    syms L0 L2 L3 L4 d1(t) theta2(t) d3(t) t
    [q1, q2, q3] = config.JointPosition;
   
    %Important: ee and base_link must be reversed!!!
    if strcmp(startFrame, "0")
        toolbox_dk = getTransform(robot,config,"ee","Link2");
        %IMPORTANT: - "getTransform" is a function of the robotics toolbox
        %           - it returns the transf. according to the URDF frames
        %             that DON'T follow the DH convention
        %           - the right ∑0 has a rotation wrt to the WRONG URDF ∑0
        %           - accounting for that, the results can be compared
        transf = [0, -1,  0, 0   ; %90° arond z-axis
                  1,  0,  0, 0   ;
                  0,  0,  1, 0.1 ; %0.1 transl. due to the placement of URDF ∑0
                  0,  0,  0, 1   ];
        toolbox_dk = transf * toolbox_dk;
    elseif strcmp(startFrame, "base")
        %IMPORTANT: - the above adjustment isn't required because my ∑base is
        %             equal to the URDF ∑base!!
        toolbox_dk = getTransform(robot,config,"ee","base_link");
    end
    
    toolbox_ee_pose = toolbox_dk(1:3,4);
    
    my_dk = subs(T, [L0 L2 L3 L4 d1(t) theta2(t) d3(t)], [0.4,0.4,0.3,0.4, q1, q2, q3]);
    my_ee_pose = my_dk(1:3,4); 
end

