% File name "PUMA_ivkine.m"

function joint_velocity = PUMA_ivkine(V_b_e, joint_theta_current, joint_velocity_max)

B1 = [ 1  0  0        0   -0.9500         0]';  theta1 = joint_theta_current(1,1);
B2 = [ 0 -1  0  -0.9500         0         0]';  theta2 = joint_theta_current(1,2);
B3 = [ 0 -1  0  -0.5500         0         0]';  theta3 = joint_theta_current(1,3);
B4 = [ 0  0  1        0         0         0]';  theta4 = joint_theta_current(1,4);
B5 = [ 0 -1  0  -0.1500         0         0]';  theta5 = joint_theta_current(1,5);
B6 = [ 0  0  1        0         0         0]';  theta6 = joint_theta_current(1,6);

Jb6 = B6;
Jb5 = Matrix_Ad(Matrix_exp(B6,-theta6))*B5;
Jb4 = Matrix_Ad(Matrix_exp(B6,-theta6)*Matrix_exp(B5,-theta5))*B4;
Jb3 = Matrix_Ad(Matrix_exp(B6,-theta6)*Matrix_exp(B5,-theta5)*Matrix_exp(B4,-theta4))*B3;
Jb2 = Matrix_Ad(Matrix_exp(B6,-theta6)*Matrix_exp(B5,-theta5)*Matrix_exp(B4,-theta4)*Matrix_exp(B3,-theta3))*B2;
Jb1 = Matrix_Ad(Matrix_exp(B6,-theta6)*Matrix_exp(B5,-theta5)*Matrix_exp(B4,-theta4)*Matrix_exp(B3,-theta3)*Matrix_exp(B2,-theta2))*B1;

Jb = [Jb1 Jb2 Jb3 Jb4 Jb5 Jb6];

q_derivation = zeros(6,1);
q_derivation([1 2 3],1) = V_b_e([4 5 6],1);
q_derivation([4 5 6],1) = V_b_e([1 2 3],1);

joint_velocity = pinv(Jb)*q_derivation;
joint_velocity = joint_velocity';

if max(joint_velocity) > joint_velocity_max 
    joint_velocity = joint_velocity * (joint_velocity_max/max(joint_velocity));
end

end

function Ad_T = Matrix_Ad(T)

R = T([1 2 3],[1 2 3]);
p = T([1 2 3],4);

p_matrix = [      0  -p(3,1)   p(2,1);
             p(3,1)        0  -p(1,1);
            -p(2,1)   p(1,1)        0;];

Ad_T = zeros(6,6);
Ad_T([1 2 3],[1 2 3]) = R;
Ad_T([4 5 6],[1 2 3]) = p_matrix*R;
Ad_T([4 5 6],[4 5 6]) = R;

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