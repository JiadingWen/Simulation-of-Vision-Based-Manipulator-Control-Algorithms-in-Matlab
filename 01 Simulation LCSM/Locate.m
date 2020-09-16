% File name "Locate.m"

% Solve Equation 1.8

% input m_c1_t  pixel coordinates of target point in camera 1 image
% input m_c2_t  pixel coordinates of target point in camera 2 image
% input x_c1_t  vector x of camera 1
% input x_c2_t  vector x of camera 2

% output X_s_t  coordinates of this point (or these points) in fixed frame

function X_s_t= Locate(m_c1_t,m_c2_t,x_c1,x_c2)

% Equation 1.8
A = [x_c1(1)-x_c1(9)*m_c1_t(1) x_c1(2)-x_c1(10)*m_c1_t(1) x_c1(3)-x_c1(11)*m_c1_t(1);
     x_c1(5)-x_c1(9)*m_c1_t(2) x_c1(6)-x_c1(10)*m_c1_t(2) x_c1(7)-x_c1(11)*m_c1_t(2);
     x_c2(1)-x_c2(9)*m_c2_t(1) x_c2(2)-x_c2(10)*m_c2_t(1) x_c2(3)-x_c2(11)*m_c2_t(1);
     x_c2(5)-x_c2(9)*m_c2_t(2) x_c2(6)-x_c2(10)*m_c2_t(2) x_c2(7)-x_c2(11)*m_c2_t(2);];

b = [m_c1_t(1)-x_c1(4);
     m_c1_t(2)-x_c1(8);
     m_c2_t(1)-x_c2(4);
     m_c2_t(2)-x_c2(8);];

% Equation 1.9
X_s_t = pinv(A)*b;

end