function result = subJointValues(expression, values)
    
    syms d1(t) theta2(t) d3(t) Dd1(t) Dtheta2(t) Dd3(t)
    syms DDd1(t) DDtheta2(t) DDd3(t) t
    
    symParams = [d1(t) theta2(t) d3(t) Dd1(t) Dtheta2(t) Dd3(t) DDd1(t) DDtheta2(t) DDd3(t)];
    
    result = subs(expression, symParams, values);
   
end

