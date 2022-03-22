function [T_Li, B_q] = computeKineticEnergy(m_Li, J_P, J_O, I_Li, q_dot, R_i)
    %This function computes the kinetic energy of a link, given all the
    %required parameters.
    
    B_q = m_Li    * (J_P.') * J_P  + ...
          J_O.'   * R_i     * I_Li * R_i.' * J_O ;   %corrected with R_i' 02/12/21 
    
    T_Li = simplify( 1/2 * q_dot.' * B_q * q_dot );
    
    %Equivalent all in one formula:
    %T_Li = 1/2 * m_Li    *  q_dot.' * J_P.' * J_P    * q_dot + ...
    %       1/2 * q_dot.' *  J_O.'   * R_i   *  I_Li  * J_O * q_dot;
end
 
