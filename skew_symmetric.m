function V = skew_symmetric(v)
    % Ensure the input is a 3-element vector
    assert(length(v) == 3, 'Input vector must be of length 3');
    
    % Extract elements of the vector
    v1 = v(1);
    v2 = v(2);
    v3 = v(3);
    
    % Construct the skew-symmetric matrix
    V = [  0  -v3   v2;
          v3    0  -v1;
         -v2   v1    0];
end
