function DJa = get_DJa_matrix(q, Dq)  
  
    theta2 = q(2);
    d3 = q(3);
    Dtheta2 = Dq(2);
    Dd3 = Dq(3);
    L4 = 0.4;
    
    DJa = [0, - cos(theta2)*Dd3 - sin(theta2)*(L4/2 - d3)*Dtheta2, -cos(theta2)*Dtheta2;
           0,   cos(theta2)*(L4/2 - d3)*Dtheta2 - sin(theta2)*Dd3, -sin(theta2)*Dtheta2;
           0,                                                   0,                    0;
           0,                                                   0,                    0;
           0,                                                   0,                    0;
           0,                                                   0,                    0];
 
end

