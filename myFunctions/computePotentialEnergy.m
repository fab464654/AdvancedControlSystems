function U_Li = computePotentialEnergy(m_Li, g, p_Li)
    % This function computes the potential energy of a link, given all the
    % required parameters.
    %
    % p_Li = offset vector (3D) from the base ref. frame to the link's 
    %          center of mass. 
    
    
   
  
    U_Li = -m_Li * g.' * p_Li;
    
end

