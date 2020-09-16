% File name "PUMA_ivkine.m"

function joint_theta_updated = PUMA_ikine(T_s_e_current,T_s_e_target,joint_theta_current,joint_velocity_max,frequency)

T = pinv(T_s_e_current) * T_s_e_target;
[S,theta] = Matrix_log(T);
V_b_e = S.*theta.*(frequency);

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

q_derivation = V_b_e;

joint_velocity = pinv(Jb)*q_derivation;
joint_velocity = joint_velocity';

if max(joint_velocity) > joint_velocity_max 
    joint_velocity = joint_velocity * (joint_velocity_max/max(joint_velocity));
end

joint_theta_updated = joint_theta_current + joint_velocity*(1/frequency);

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

function [S,theta] = Matrix_log(T)


R = T([1 2 3],[1 2 3]);
p = T([1 2 3],4);


    if (norm(R-eye(3)) < 1e-10) && (norm(p-[0 0 0]') < 1e-10)
        w = [0 0 0]';
        v = [0 0 0]';
        theta = 0;
    elseif (norm(R-eye(3)) < 1e-10) && (~(norm(p-[0 0 0]') < 1e-10))
        w = [0 0 0]';
        theta = norm(p);
        v = p/norm(p);
    elseif (norm(trace(R)-(-1)) < 1e-10) && (norm(p-[0 0 0]') < 1e-10)
        w = (1/((2*(1+R(3,3)))^0.5)).*[R(1,3);R(2,3);1+R(3,3)];
        theta = pi;
        v = [0 0 0]';
    elseif (norm(trace(R)-(-1)) < 1e-10) && (~(norm(p-[0 0 0]') < 1e-10))
        w = (1/((2*(1+R(3,3)))^0.5)).*[R(1,3);R(2,3);1+R(3,3)];
        theta = pi;
        w_matrix = [      0  -w(3,1)   w(2,1);
                     w(3,1)        0  -w(1,1);
                    -w(2,1)   w(1,1)        0;];
        v = ((1/theta).*eye(3)-0.5*w_matrix+(1/theta-0.5*cot(theta/2))*((w_matrix)^2))*p;
    elseif (~(norm(trace(R)-(-1)) < 1e-10)) && (norm(p-[0 0 0]') < 1e-10)
        theta = acos(0.5*(trace(R)-1));
        w_matrix = (1/(2*sin(theta))).*(R-R');
        w = [w_matrix(3,2) w_matrix(1,3) w_matrix(2,1)]';
        v = [0 0 0]';
    else
        theta = acos(0.5*(trace(R)-1));
        w_matrix = (1/(2*sin(theta))).*(R-R');
        w = [w_matrix(3,2) w_matrix(1,3) w_matrix(2,1)]';
        v = ((1/theta).*eye(3)-0.5*w_matrix+(1/theta-0.5*cot(theta/2))*((w_matrix)^2))*p;
    end

S = zeros(6,1);
S([1 2 3],1) = w;
S([4 5 6],1) = v;

end