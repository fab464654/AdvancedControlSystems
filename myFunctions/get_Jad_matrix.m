function Jad = get_Jad_matrix(q, x_tilde)  
  
    %Get geometric jacobian w.r.t. to frame 0    
    Jg = get_Jg_matrix(q); 
        
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
      
    Jad = pinv(Ta)*[Rd.'        zeros(3,3);
                     zeros(3,3)  Rd.'      ] * Jg;            
end

