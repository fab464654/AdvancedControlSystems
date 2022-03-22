function outMatrix = removeTimeDependency(matrix)
    
    syms t d1(t) theta2(t) d3(t) Dd1(t) Dtheta2(t) Dd3(t) DDd1(t) DDtheta2(t) DDd3(t) ...
         d1_ theta2_ d3_ Dd1_ Dtheta2_ Dd3_ DDd1_ DDtheta2_ DDd3_ ...
         L0 L2 L3 L4 m_Link2 m_Link3 m_Link4
    outMatrix = sym(zeros(size(matrix)));
    disp(outMatrix)
    for i=1:size(matrix,1)
        for j=1:size(matrix,2)
            outMatrix(i,j) = ...
            subs(matrix(i,j), [d1(t), theta2(t), d3(t), Dd1(t), Dtheta2(t), ...
                               Dd3(t), DDd1(t), DDtheta2(t), DDd3(t)], ...
                              [d1_, theta2_, d3_, Dd1_, Dtheta2_, Dd3_, DDd1_, DDtheta2_, DDd3_]);
        end
    end
end

