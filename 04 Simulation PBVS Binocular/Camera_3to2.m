% File name "Camera_3to2.m"

% Equation 2.4

function m = Camera_3to2(X_s,T_c_s)

    a = [800    0  800   0;
           0  800  800   0;
          0    0    1   0;];
                
    % Homogenizing 
    [r,c]=size(X_s);
    point_homo = zeros([r,c]+[1,0]);
    point_homo([1 2 3],:) = X_s;
    point_homo(4,:) = ones(1,c);
    
    % Equation 2.1
    pixel_homo = a*T_c_s*point_homo; 
    
    % De-homogenization
    m = zeros(size(pixel_homo)-[1 0]);
    m([1 2],:) = [pixel_homo(1,:)./pixel_homo(3,:);
                      pixel_homo(2,:)./pixel_homo(3,:);];
    
end