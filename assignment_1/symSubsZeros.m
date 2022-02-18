function newMatrix = symSubsZeros(matrix, threshold)
    %This function is used to remove low coeffiecients from a symbolic
    %matrix and simplify the overall form. 
    %
    %Example: (10^-10*coeff1) + 1*coeff2 --> coeff2
    %
    %Input:  simbolic matrix, coefficients' threshold
    %Output: simplified matrix
    
    newMatrix = matrix;
    for i=1:length(matrix(:))        
        
        %Extract coefficients and variables of the "i" equation
        [coeff, var] = coeffs(matrix(i)); 
        numCoefficients = size(coeff, 2);
        
        %Look for small coefficients and substitute them with sym(0)
        for j=1:numCoefficients
            if abs(coeff(j)) < threshold
                coeff(j) = sym(0);
            end
        end

        %Build the simplified equation
        newEquation = sym(0);
        for k=1:numCoefficients
            newEquation = simplify(newEquation + coeff(k)*var(k));
        end
        
        %Update the old equation with the new simplified one
        newMatrix(i) = newEquation;
    end
end

