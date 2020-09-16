% File name "Calculate_Ls.m"

% Equation 2.13

function Ls = Calculate_Ls(s,z)

    Ls = zeros(size(s,1),6);
    
    for i=1:size(s,1)/2
        x = s(2*i-1,1);
        y = s(  2*i,1);
        
        Ls(2*i-1,1) = -1/z(1,i);
        Ls(2*i-1,2) = 0;
        Ls(2*i-1,3) = x/z(1,i);
        Ls(2*i-1,4) = x*y;
        Ls(2*i-1,5) = -(1+x^2);
        Ls(2*i-1,6) = y;
        
        Ls(  2*i,1) = 0;
        Ls(  2*i,2) = -1/z(1,i);
        Ls(  2*i,3) = y/z(1,i);
        Ls(  2*i,4) = 1+y^2;
        Ls(  2*i,5) = -x*y;
        Ls(  2*i,6) = -x;
    end
    
end