function [toolbox_dk, my_dk, toolbox_ee_pose, my_ee_pose] = checkDirectKinematics(robot, config, T)
    %This function substitutes the given "config" joint parameters' values
    %inside the transformation matrix "T"
    syms L0 L2 L3 L4 d1(t) theta2(t) d3(t) t
    [q1, q2, q3] = config.JointPosition;
   
    %Important: ee and base_link must be reversed!!!
    toolbox_dk = getTransform(robot,config,"ee","base_link");
    toolbox_ee_pose = toolbox_dk(1:3,4);
    
    my_dk = subs(T, [L0 L2 L3 L4 d1(t) theta2(t) d3(t)], [0.4,0.4,0.3,0.4, q1, q2, q3]);
    my_ee_pose = my_dk(1:3,4); 
end

