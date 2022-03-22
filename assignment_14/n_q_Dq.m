function n = n_q_Dq(Dq, q)
    
    C = get_C_matrix(q,Dq);
    G = get_G_matrix(q);
    
    n = C*Dq + G; %F = friction is neglected
end