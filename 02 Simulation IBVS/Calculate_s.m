% File name "Calculate_s.m"

% Equation 2.8

function s = Calculate_s(m,a)

    s = zeros(numel(m),1);
    
    for i = 1:size(m,2)
        s(2*i-1,1) = (m(1,i)-a(1,3))/a(1,1);
        s(  2*i,1) = (m(2,i)-a(2,3))/a(2,2);
    end
    
end