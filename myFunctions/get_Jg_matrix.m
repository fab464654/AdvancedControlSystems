function Jg = get_Jg_matrix(q)  
  
    theta2 = q(2);
    d3 = q(3);    
    L4 = 0.4;
    
    Jg = [0, cos(theta2)*(L4/2 - d3), -sin(theta2);
          0, sin(theta2)*(L4/2 - d3),  cos(theta2);
          1,                       0,            0;
          0,                       0,            0;
          0,                       0,            0;
          0,                       1,            0];   
end

