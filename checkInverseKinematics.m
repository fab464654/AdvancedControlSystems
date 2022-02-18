function  my_joints = checkInverseKinematics(goalPose, ik_joints)
    %This function checks the correctness of the symbolic inverse kinematic
    %computation, using the checkDirectKinematics function.
    %
    %Input: robot object, goalPose (previously obtained by the checkDirectKinematics
    %       function), ik_joints (symbolic equations in which values are substituted)
    %
    %Output: joint angles/displacement to reach the given "goalPose"
    
    fprintf("[CheckInverse] Checking the inverse kinematics computation, considering "+...
            "the computed goal pose, starting from the given joint configuration");

    syms L0 L2 L3 L4 p_x p_y p_z
    
    %I'm considering that L0 = 0.4m; L2 = 0.4m, L3 = 0.3m; L4 = 0.4m
    %are the links' lengths according to my URDF structure!
    fprintf("\n[CheckInverse] Comparing given joint parameters with the computed ones:\n");
    fprintf("[CheckInverse] Given goal pose -> [%.3f %.3f %.3f]\n", goalPose);
 
    my_joints = subs(ik_joints, [L0 L2 L3 L4 p_x p_y p_z], [0.4,0.4,0.3,0.4,goalPose.']);
    
    for i=1:size(my_joints, 1)
        fprintf("[CheckInverse] Computed joint parameters (%d) -> [%.3f %.3f %.3f]\n", i, my_joints(i,:));
    end
end

