function T0_ee = get_T0_ee_matrix(q)  

    d1 = q(1);
    theta2 = q(2);
    d3 = q(3);
    
    L2 = 0.4; L3 = 0.3; L4 = 0.4; 
    T0_ee = [0,    -cos(theta2),     sin(theta2),  sin(theta2)*(L4/2 - d3);
             0,    -sin(theta2),    -cos(theta2), -cos(theta2)*(L4/2 - d3);
             1,               0,               0,           L2/2 + L3 + d1;
             0,               0,               0,                       1];
end

