function G = get_G_matrix(q)
 %obtained with the following code:
     
%     loadParameters;
%     syms d3_ theta2_
%     eval(removeTimeDependency(formula(G)))
    theta2 = q(2);
    d3 = q(3);
    G = [                        0 ;
         -(981*d3*sin(theta2))/500;
             (981*cos(theta2))/500];
end

