function newMatrix = symSubsZeros(matrix, threshold, roundCoeff)
    %This function is used to remove low coeffiecients from a symbolic
    %matrix and simplify the overall form. 
    %
    %Example: (10^-10*coeff1) + 1*coeff2 --> coeff2
    %
    %Input:  simbolic matrix, coefficients' threshold
    %Output: simplified matrix
    %
    %In this version the script works also for fractions of polynomials.
    %
    
    newMatrix = matrix;
    for i=1:length(matrix(:))      
        %If the expression is not a polynomial (fraction) I must treat Num.
        %and Den. separately!! 
        [N, D] = numden(matrix(i));
        [~, D_var] = coeffs(D);  
        
        if size(D_var(:)) < 2 %only 1 polynomial is present in "matrix(i)"  
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
                if nargin == 3 %includes the rounding factor
                    coeff(k) = round(coeff(k),roundCoeff);
                    %This check is needed because "round(1)=1.0" and "1.0"
                    %isn't simplified (or removed) by the "simplify" function!
                    if abs(coeff(k)) == 1.0 
                        coeff(k) = int64(coeff(k));
                    end
                end
                newEquation = simplify(newEquation + coeff(k)*var(k));
            end

            %Update the old equation with the new simplified one
            newMatrix(i) = newEquation;
            
        else %fraction of two polynomials
            if nargin == 3 %includes the rounding factor
                numerator = symSubsZeros(N, threshold, roundCoeff);
                denominator = symSubsZeros(D, threshold, roundCoeff);                
            else
                numerator = symSubsZeros(N, threshold);
                denominator = symSubsZeros(D, threshold);
            end
            newMatrix(i) = simplify(numerator / denominator);           
        end
    end
end

