% File name "Calculate_s.m"


function s = Calculate_s(T_e_star_e)

R = T_e_star_e([1 2 3],[1 2 3]);
p = T_e_star_e([1 2 3],4);

% exponential coordinate representation of rotation matrix
if (norm(R-eye(3)) < 1e-10) && (norm(p-[0 0 0]') < 1e-10)
    w = [0 0 0]';
    theta = 0;
elseif (norm(R-eye(3)) < 1e-10) && (~(norm(p-[0 0 0]') < 1e-10))
    w = [0 0 0]';
    theta = norm(p);
elseif (norm(trace(R)-(-1)) < 1e-10) && (norm(p-[0 0 0]') < 1e-10)
    w = (1/((2*(1+R(3,3)))^0.5)).*[R(1,3);R(2,3);1+R(3,3)];
    theta = pi;
elseif (norm(trace(R)-(-1)) < 1e-10) && (~(norm(p-[0 0 0]') < 1e-10))
    w = (1/((2*(1+R(3,3)))^0.5)).*[R(1,3);R(2,3);1+R(3,3)];
    theta = pi;
else
    theta = acos(0.5*(trace(R)-1));
    w_matrix = (1/(2*sin(theta))).*(R-R');
    w = [w_matrix(3,2) w_matrix(1,3) w_matrix(2,1)]';
end

thetau = theta*w';
t = p';
s = [t, thetau];

end

