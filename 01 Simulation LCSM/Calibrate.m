%File name "Calibrate.m"

% Solve Equation 1.6

% input X_s spatial coordinates of 6 calibrating points in space frame
% input m   pixel coordinates of 6 calibrating points 

% output x  see Equation 1.6

function x = Calibrate(m,X_s)

% Equation 1.6
A = [X_s(1,1) X_s(2,1) X_s(3,1) 1        0        0        0 0 -X_s(1,1)*m(1,1) -X_s(2,1)*m(1,1) -X_s(3,1)*m(1,1);
            0        0        0 0 X_s(1,1) X_s(2,1) X_s(3,1) 1 -X_s(1,1)*m(2,1) -X_s(2,1)*m(2,1) -X_s(3,1)*m(2,1);
     X_s(1,2) X_s(2,2) X_s(3,2) 1        0        0        0 0 -X_s(1,2)*m(1,2) -X_s(2,2)*m(1,2) -X_s(3,2)*m(1,2);
            0        0        0 0 X_s(1,2) X_s(2,2) X_s(3,2) 1 -X_s(1,2)*m(2,2) -X_s(2,2)*m(2,2) -X_s(3,2)*m(2,2);
     X_s(1,3) X_s(2,3) X_s(3,3) 1        0        0        0 0 -X_s(1,3)*m(1,3) -X_s(2,3)*m(1,3) -X_s(3,3)*m(1,3);
            0        0        0 0 X_s(1,3) X_s(2,3) X_s(3,3) 1 -X_s(1,3)*m(2,3) -X_s(2,3)*m(2,3) -X_s(3,3)*m(2,3);
     X_s(1,4) X_s(2,4) X_s(3,4) 1        0        0        0 0 -X_s(1,4)*m(1,4) -X_s(2,4)*m(1,4) -X_s(3,4)*m(1,4);
            0        0        0 0 X_s(1,4) X_s(2,4) X_s(3,4) 1 -X_s(1,4)*m(2,4) -X_s(2,4)*m(2,4) -X_s(3,4)*m(2,4);
     X_s(1,5) X_s(2,5) X_s(3,5) 1        0        0        0 0 -X_s(1,5)*m(1,5) -X_s(2,5)*m(1,5) -X_s(3,5)*m(1,5);
            0        0        0 0 X_s(1,5) X_s(2,5) X_s(3,5) 1 -X_s(1,5)*m(2,5) -X_s(2,5)*m(2,5) -X_s(3,5)*m(2,5);
     X_s(1,6) X_s(2,6) X_s(3,6) 1        0        0        0 0 -X_s(1,6)*m(1,6) -X_s(2,6)*m(1,6) -X_s(3,6)*m(1,6);
            0        0        0 0 X_s(1,6) X_s(2,6) X_s(3,6) 1 -X_s(1,6)*m(2,6) -X_s(2,6)*m(2,6) -X_s(3,6)*m(2,6);];

b =  [m(1,1);
      m(2,1);
      m(1,2);
      m(2,2);
      m(1,3);
      m(2,3);
      m(1,4);
      m(2,4);
      m(1,5);
      m(2,5);
      m(1,6);
      m(2,6);];

% Equation 1.7
x = pinv(A)*b;

end