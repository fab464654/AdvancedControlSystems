function config = getRandomConfigurationRight(robot, jointLimits)
    config = homeConfiguration(robot);
    
    %Compute a random configuration, considering joint limits
    for i = 1:size(jointLimits, 1)
        config(i).JointPosition = jointLimits(i,1) + 2*jointLimits(i,2)*rand;
    end
  
end

