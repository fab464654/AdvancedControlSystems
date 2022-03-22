function B = get_B_matrix(q)
    %This function substitutes the actual value of d3 inside the B matrix
    %obtained with the following code:
     
%     loadParameters;
%     syms d3_
%     eval(removeTimeDependency(B))

    d3 = q(3);
    B = [17/10,                                                   0,   0;
             0, d3^2/5 + 23095960089031723/7205759403792793600     ,   0;
             0,                                                   0, 1/5];
  
end

