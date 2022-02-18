function Jg_toolbox = getGeometricJacobianRight(robot, config, frame)
    %This function "corrects" the Geometric Jacobian, having linear
    %velocities on the upper part and angulars on the lower part.
    Jg_toolbox = geometricJacobian(robot, config, frame);
 
    for i=1:size(Jg_toolbox,2)
        temp = Jg_toolbox([1:3],i);
        Jg_toolbox([1:3],i) = Jg_toolbox([4:6],i);
        Jg_toolbox([4:6],i) = temp;
    end
    
end