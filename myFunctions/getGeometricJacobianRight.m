function Jg_toolbox = getGeometricJacobianRight(myRobot, robot, config, startFrame, endFrame)
    %This function "corrects" the Geometric Jacobian, having linear
    %velocities on the upper part and angulars on the lower part and retrieves
    %Jacobian according to the start frame ∑0 or ∑base.
    
    Jg_toolbox_base = geometricJacobian(robot, config, endFrame);
    %Change the order of the linear/angular part
    for i=1:size(Jg_toolbox_base,2)
        temp = Jg_toolbox_base([1:3],i);
        Jg_toolbox_base([1:3],i) = Jg_toolbox_base([4:6],i);
        Jg_toolbox_base([4:6],i) = temp;
    end
    
    if strcmp(startFrame, "0")  %in this case convert for ∑0
        T_base_to_0 = myRobot.T_base_to_0_DH; %retrive (DH) transf. from ∑base to ∑0
        T_0_to_base = inv(T_base_to_0);
        R_0_to_base = T_0_to_base(1:3,1:3);
        T_jacobians = [R_0_to_base  , zeros(3)  ;
                       zeros(3)     , R_0_to_base    ];
        Jg_toolbox = T_jacobians*Jg_toolbox_base;
        Jg_toolbox(find(abs(Jg_toolbox)<0.0001)) = 0;
    elseif strcmp(startFrame, "base")
        Jg_toolbox = Jg_toolbox_base;
    end
    
    
    
    Jg_toolbox = vpa(Jg_toolbox, 5); %to reduce decimal digits
    
end