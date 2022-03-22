function C = get_C_matrix(q,Dq)
    %obtained with the following code:
     
%     loadParameters;
%     syms d3_ Dd3_ Dtheta2_
%     eval(removeTimeDependency(C))
    Dtheta2 = Dq(2);
    Dd3 = Dq(3);
    d3 = q(3);
    
    C = [0,                 0,                0;
         0,        (Dd3*d3)/5,   (Dtheta2*d3)/5;
         0,   -(Dtheta2*d3)/5,                0];
end

