% File name "PUMA_fkine.m"

function Tsb = PUMA_fkine(q)

B1 = [ 1  0  0        0   -0.9500         0]';
B2 = [ 0 -1  0  -0.9500         0         0]';
B3 = [ 0 -1  0  -0.5500         0         0]';
B4 = [ 0  0  1        0         0         0]';
B5 = [ 0 -1  0  -0.1500         0         0]';
B6 = [ 0  0  1        0         0         0]';

M = [0  0  1    0.95;
     0 -1  0    0;
     1  0  0    0.65;
     0  0  0      1;];

T1 = Matrix_exp(B1,q(1,1));
T2 = Matrix_exp(B2,q(1,2));
T3 = Matrix_exp(B3,q(1,3));
T4 = Matrix_exp(B4,q(1,4));
T5 = Matrix_exp(B5,q(1,5));
T6 = Matrix_exp(B6,q(1,6));

Tsb = M*T1*T2*T3*T4*T5*T6;

end

function T = Matrix_exp(S,theta)

T = eye(4);

w_matrix = [      0  -S(3,1)   S(2,1);
             S(3,1)        0  -S(1,1);
            -S(2,1)   S(1,1)        0;];

v = [S(4,1);S(5,1);S(6,1);];

Rot = eye(3)+sin(theta).*w_matrix+(1-cos(theta)).*(w_matrix^2);

Gv = (eye(3).*theta+(1-cos(theta)).*w_matrix+(theta-sin(theta)).*(w_matrix^2))*v;

T([1 2 3],[1 2 3]) = Rot;
T([1 2 3],4) = Gv;

end

