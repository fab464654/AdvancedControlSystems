function DJad = get_DJad_matrix(q, Dq, x_tilde)  
  
    %Compute time derivative of geometric jacobian w.r.t. to frame 0    
    theta2 = q(2);   d3 = q(3);    
    Dtheta2 = Dq(2); Dd3 = Dq(3);
    L4 = 0.4;
    DJg = [0, - cos(theta2)*Dd3 - sin(theta2)*(L4/2 - d3)*Dtheta2, -cos(theta2)*Dtheta2;
           0,   cos(theta2)*(L4/2 - d3)*Dtheta2 - sin(theta2)*Dd3, -sin(theta2)*Dtheta2;
           0,                                                   0,                    0;
           0,                                                   0,                    0;
           0,                                                   0,                    0;
           0,                                                   0,                   0];
        
    Rd = eul2rotm(-x_tilde(4:6).', 'ZYZ');
    phi_d_e = -x_tilde(4:6);
    
    %Compute T(phi_d_e) function of the euler angles of x_d
    T = [0, -sin(phi_d_e(1)), cos(phi_d_e(1))*sin(phi_d_e(2));
         0,  cos(phi_d_e(1)), sin(phi_d_e(1))*sin(phi_d_e(2));
         1,     0           , cos(phi_d_e(2))                ];

    Ta = [1 0 0   0 0 0 ;
          0 1 0   0 0 0 ;
          0 0 1   0 0 0 ;
          0 0 0 , T(1,:);
          0 0 0 , T(2,:);
          0 0 0 , T(3,:)];      
    
    DJad = pinv(Ta)*[Rd.'        zeros(3,3);
                     zeros(3,3)  Rd.'      ] * DJg;            
end

