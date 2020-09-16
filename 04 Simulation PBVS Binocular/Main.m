%% Header
% File name "PBVS_Binocular.m"
% This code only tested on Matlab R2018b with Robotics Toolbox version 10.3

clear
clc
close all

%% Configuration Visualization

% 6R PUMA type robot arm and Body frame (Using Robotics Toolbox)
figure(1)
title('Configuration Visualization')
L(1) = Link([ 000*pi/180  0.65000 0.00000   -090*pi/180    0    000*pi/180]); 
L(2) = Link([ 000*pi/180  0.15000 0.40000    000*pi/180    0    000*pi/180]); 
L(3) = Link([ 000*pi/180 -0.15000 0.00000   -090*pi/180    0   -090*pi/180]); 
L(4) = Link([ 000*pi/180  0.40000 0.00000    090*pi/180    0    000*pi/180]); 
L(5) = Link([ 000*pi/180  0.00000 0.00000   -090*pi/180    0    000*pi/180]); 
L(6) = Link([ 000*pi/180  0.15000 0.00000    000*pi/180    0    000*pi/180]); 
PUMA = SerialLink(L,'name','PUMA');
theta_initial = [045*pi/180  -75*pi/180  075*pi/180  000*pi/180  090*pi/180  000*pi/180];
PUMA.plot(theta_initial,'scale',0.3);

% Space frame
Ts = [1 0 0 0;
      0 1 0 0;
      0 0 1 0;
      0 0 0 1;];
hold on
trplot(Ts,'thick',2,'frame','s','length',0.25,'rgb')


% Task space
xlim([-0.4 1.4])
ylim([-0.4 1.4])
zlim([-0.4 1.4])
length = 1;
height = 0.5; 
x_length = 0:length/10:length;
y_length = 0:length/10:length;
[x_grid,y_grid] = meshgrid(x_length,y_length);
mesh(x_grid,y_grid,height.*ones(11,11));

% Camera frame
% Camera frame is coincident with the body frame
% Do not need visualization

%% Camera calibration

% intrinsic parameter matric of camera
a = [800    0  800   0;
       0  800  800   0;
       0    0    1   0;];


%% Set Target point & 4 interest points & Object frame

% Target point
X_s_t_set = [0.35, 0.35, height + 0.1]';

% Place target point in task space
plot3(X_s_t_set(1,1),X_s_t_set(2,1),X_s_t_set(3,1),'rx')

% 4 interest points in task space
X_s_i_set = [X_s_t_set(1,1)-0.05  X_s_t_set(1,1)-0.05  X_s_t_set(1,1)+0.05   X_s_t_set(1,1)+0.05;
             X_s_t_set(2,1)-0.05  X_s_t_set(2,1)+0.05  X_s_t_set(2,1)-0.05   X_s_t_set(2,1)+0.05;
             X_s_t_set(3,1)-0.1   X_s_t_set(3,1)-0.1   X_s_t_set(3,1)-0.1    X_s_t_set(3,1)-0.1;];

% Place 4 interest points in task space
figure(1)
line(X_s_i_set(1,[1 2]),X_s_i_set(2,[1 2]),X_s_i_set(3,[1 2]),'color','r');
line(X_s_i_set(1,[2 4]),X_s_i_set(2,[2 4]),X_s_i_set(3,[2 4]),'color','r');
line(X_s_i_set(1,[4 3]),X_s_i_set(2,[4 3]),X_s_i_set(3,[4 3]),'color','r');
line(X_s_i_set(1,[3 1]),X_s_i_set(2,[3 1]),X_s_i_set(3,[3 1]),'color','r');
plot3(X_s_i_set(1,1),X_s_i_set(2,1),X_s_i_set(3,1),'ro')
plot3(X_s_i_set(1,2),X_s_i_set(2,2),X_s_i_set(3,2),'ro')
plot3(X_s_i_set(1,3),X_s_i_set(2,3),X_s_i_set(3,3),'ro')
plot3(X_s_i_set(1,4),X_s_i_set(2,4),X_s_i_set(3,4),'ro')

% Object frame
T_s_o_set = [-1  0  0      X_s_t_set(1,1)
              0  1  0      X_s_t_set(2,1)
              0  0 -1  X_s_t_set(3,1)-0.1
              0  0  0                   1];
      
% Place object frame in task space
trplot(T_s_o_set,'thick',2,'frame','o','length',0.25,'rgb')

%%  Desired Parameter

% Set desired pixel coordinates of 4 interest points
m_c_i_star = [ 400  400 1200 1200;
              1200  400 1200  400;];

% Plot desired position of 4 interest points in camera image
figure(2)
title('Camera Image')
xlim([0 1600])
ylim([0 1600])
hold on
plot(m_c_i_star(1,1),m_c_i_star(2,1),'b*')
plot(m_c_i_star(1,2),m_c_i_star(2,2),'b*')
plot(m_c_i_star(1,3),m_c_i_star(2,3),'b*')
plot(m_c_i_star(1,4),m_c_i_star(2,4),'b*')
line(m_c_i_star(1,[1 2]),m_c_i_star(2,[1 2]),'color','b');
line(m_c_i_star(1,[2 4]),m_c_i_star(2,[2 4]),'color','b');
line(m_c_i_star(1,[4 3]),m_c_i_star(2,[4 3]),'color','b');
line(m_c_i_star(1,[3 1]),m_c_i_star(2,[3 1]),'color','b');

% Set desired pose of end-effector relative to object frame
T_o_e_star = [ 1  0  0    0;
               0  1  0    0;
               0  0  1 -0.1;
               0  0  0    1;];

% Plot desired pose of end-effector relative to object frame
figure(1)
trplot(T_s_o_set * T_o_e_star,'thick',2,'frame','e*','length',0.25,'rgb')

% Equation 4.20
T_e_star_e_star = [ 1  0  0  0;
                    0  1  0  0;
                    0  0  1  0;
                    0  0  0  1;];

% Calculate desired visual feature 
s_star = Calculate_s(T_e_star_e_star);

%% Control Parameter

% Upper limit of iteration times
iteration_limit = 1000;

% Termination conditions of iteration
error_max = 0.001;

% Upper limit of angular velocity of robot manipulator joints (rad/s)
joint_velocity_max = 0.5;

% Cameras report 25 images per second
Hz = 25;

% Current angles of robot manipulator joints
theta_current = theta_initial;

% Current pose of robot end-effector
T_s_e_current = PUMA_fkine(theta_current);

% Set Lambda
Lambda = 10;

for i = 1:iteration_limit
    
    %% Error (i.e. distance between current position and desired position of end-effector)
    X_s_e_current = T_s_e_current([1 2 3],4);
    X_s_e_desired = X_s_t_set;
    error = norm(X_s_e_current - X_s_e_desired)^0.5;
    figure(4)
    title('Error')
    hold on
    plot(i,error,'k.')
    disp(i)
    if error < error_max
         break
    end

    %% Binocular Vision System 
    
    % Get pixel coordinates of 4 interest points in camera 1
    T_c1_s = inv(T_s_e_current);  %equation 4.3
    m_c1_i = Camera_3to2(X_s_i_set,T_c1_s); %equation 4.4
    
    % Plot current position of 4 interest points in camera image
    figure(2)
    plot(m_c1_i(1,1),m_c1_i(2,1),'r.')
    plot(m_c1_i(1,2),m_c1_i(2,2),'r.')
    plot(m_c1_i(1,3),m_c1_i(2,3),'r.')
    plot(m_c1_i(1,4),m_c1_i(2,4),'r.')
    if i == 1
        line(m_c1_i(1,[1 2]),m_c1_i(2,[1 2]),'color','r');
        line(m_c1_i(1,[2 4]),m_c1_i(2,[2 4]),'color','r');
        line(m_c1_i(1,[4 3]),m_c1_i(2,[4 3]),'color','r');
        line(m_c1_i(1,[3 1]),m_c1_i(2,[3 1]),'color','r');
    end
    
    % Pose relationship bewteen two cameras
    b = 0.05;
    T_c1_c2 = [1  0  0  b;
               0  1  0  0;
               0  0  1  0;
               0  0  0  1;];
    T_c2_c1 = [1  0  0 -b;
               0  1  0  0;
               0  0  1  0;
               0  0  0  1;];
     
    % Pose of camera 2
    T_c2_s = T_c2_c1 * T_c1_s;
    
    % Get pixel coordinates of 4 interest points in camera 2
    m_c2_i = Camera_3to2(X_s_i_set,T_c2_s);
    
    % Calculate z_c_i of each target point
    z_c1_i = Calculate_z_c(m_c1_i,m_c2_i,a,b);
    
    % Get spatial coordinates of 4 interest points in camera 1
    X_c1_i = zeros(3,size(m_c1_i,2));
    for j = 1:size(m_c1_i,2)
        X_c1_i(1,j) = (m_c1_i(1,j)-a(1,3))/a(1,1).*z_c1_i(1,j);
        X_c1_i(2,j) = (m_c1_i(2,j)-a(2,3))/a(2,2).*z_c1_i(1,j);
        X_c1_i(3,j) = z_c1_i(1,j);
    end
    
    %% Current pose of end-effector
    
    % Calculate spatial coordinates of interest points in object frame
    X_o_i = [ 0.05  0.05 -0.05 -0.05;
             -0.05  0.05 -0.05  0.05;
                 0     0     0     0;];
    
    % equation 4.12
    T_c1_o = [X_c1_i; 1 1 1 1;] * pinv([X_o_i; 1 1 1 1;]);
    
    % equation 4.13
    T_c1_o([1 2 3],3) = cross(T_c1_o([1 2 3],1),T_c1_o([1 2 3],2));
    
    % equation 4.16
    T_e_o = T_c1_o;
    
    % equation 4.17
    T_o_e = pinv(T_e_o);
    
    
    %% Visual Feature & Visual Feature Error
    
    % Equation 4.18
    T_e_star_e = inv(T_o_e_star) * T_o_e;
    
    % Calculate visual feature
    s = Calculate_s(T_e_star_e);
    
    % Calculate visual feature error
    e = s - s_star;
    
    % Plot visual feature error
    figure(3)
    title('Visual Feature Error')
    hold on
    plot(i,e,'k.')
    
    %% Spatial Velocity Calculation
    
    % Equation 4.24
    p = s(1, [1 2 3]);
    thetau = s(1, [4 5 6]);
    
    V_b_e([1 2 3],1) = -Lambda.*(T_e_star_e([1 2 3],[1 2 3])'*p');
    V_b_e([4 5 6],1) = -Lambda.*thetau';
    
    %% Inverse Velocity Kinematics of Robot Manipulator
    
    % Angular velocities of robot manipulator joints
    theta_deria = PUMA_ivkine(V_b_e, theta_current, joint_velocity_max);
    
    % Update angles of robot manipulator joints
    theta_current = theta_current + theta_deria*(1/Hz);
    
    % Update pose of robot end-effector
    T_s_e_current = PUMA_fkine(theta_current);
    
    % Plot new pose of end-effector
    figure(1)
    PUMA.plot(theta_current,'scale',0.3);
    
    % Plot Trajectory of end effector
    Trajectory(:,i) = X_s_e_current;
    if i > 1
        plot3(Trajectory(1,[i-1 i]),Trajectory(2,[i-1 i]),Trajectory(3,[i-1 i]),'color','k','LineWidth',1)
    end
    
end








