%% Header
% File name "LCSM.m"
% This code only tested on Matlab R2018b with Robotics Toolbox version 10.3

clear
clc
close all

%% Configuration Visualization

% 6R PUMA type robot manipulator and Body frame (Using Robotics Toolbox)
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
T_s = [1 0 0 0;
      0 1 0 0;
      0 0 1 0;
      0 0 0 1;];
hold on
trplot(T_s,'thick',2,'frame','s','length',0.25,'rgb')

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
M4 = [1 0 0 1;
      0 1 0 0;
      0 0 1 height + 0.4;
      0 0 0 1;];
agx = 000*pi/180;
M3 = [      1         0         0       0;
            0  cos(agx) -sin(agx)       0;
            0  sin(agx)  cos(agx)       0;
            0         0         0       1;];
agy = -135*pi/180;
M2 = [cos(agy)        0   sin(agy)       0;
            0         1          0       0;
     -sin(agy)        0   cos(agy)       0;
            0         0          0       1;];
agz = 000*pi/180;
M1 = [cos(agz) -sin(agz)         0       0;
      sin(agz)  cos(agz)         0       0;
            0         0          1       0;
            0         0          0       1;];
agx = -30*pi/180;
M0 = [      1         0         0       0;
            0  cos(agx) -sin(agx)       0;
            0  sin(agx)  cos(agx)       0;
            0         0         0       1;];
T_s_c1 = M4*M3*M2*M1*M0;
trplot(T_s_c1,'thick',2,'frame','c1','length',0.25,'rgb')
M4 = [1 0 0 0;
      0 1 0 1;
      0 0 1 height + 0.4;
      0 0 0 1;];
agx = 135*pi/180;
M3 = [      1         0         0       0;
            0  cos(agx) -sin(agx)       0;
            0  sin(agx)  cos(agx)       0;
            0         0         0       1;];
agy = 000*pi/180;
M2 = [cos(agy)        0   sin(agy)       0;
            0         1          0       0;
     -sin(agy)        0   cos(agy)       0;
            0         0          0       1;];
agz = 090*pi/180;
M1 = [cos(agz) -sin(agz)         0       0;
      sin(agz)  cos(agz)         0       0;
            0         0          1       0;
            0         0          0       1;];
agx = 030*pi/180;
M0 = [      1         0         0       0;
            0  cos(agx) -sin(agx)       0;
            0  sin(agx)  cos(agx)       0;
            0         0         0       1;];
T_s_c2 = M4*M3*M2*M1*M0;
trplot(T_s_c2,'thick',2,'frame','c2','length',0.25,'rgb')

%% Camera calibration

% spatial coordinates of 6 calibrating points
X_s = [0.2  0.4  0.6  0.2  0.4  0.6
       0.2  0.2  0.2  0.6  0.6  0.6
      height height height height height height];
plot3(X_s(1,:),X_s(2,:),X_s(3,:),'rx')

% pixel coordinates of 6 calibrating points 
m_c1 = Camera1_3to2(X_s);
m_c2 = Camera2_3to2(X_s);

% plot camera image
figure(2)
hold on
title('Camera 1 Image')
xlim([0 1600])
ylim([0 1600])
plot(m_c1(1,:),m_c1(2,:),'rx')
figure(3)
hold on
title('Camera 2 Image')
xlim([0 1600])
ylim([0 1600])
plot(m_c2(1,:),m_c2(2,:),'rx')

% Calibrate camera
x_c1 = Calibrate(m_c1,X_s);
x_c2 = Calibrate(m_c2,X_s);

%% Target point location

% Set target point
X_s_t_set = [0.35, 0.35, height]';

% Plot target point
figure(1)
plot3(X_s_t_set(1,:),X_s_t_set(2,:),X_s_t_set(3,:),'ro')

% pixel coordinates of target point
m_c1_t = Camera1_3to2(X_s_t_set);
m_c2_t = Camera2_3to2(X_s_t_set);

% plot target point in camera image
figure(2)
plot(m_c1_t(1,:),m_c2_t(2,:),'ro')
figure(3)
plot(m_c1_t(1,:),m_c2_t(2,:),'ro')

% locate target point
X_s_t = Locate(m_c1_t,m_c2_t,x_c1,x_c2);

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

% equation 1.1
T_s_e_target = T_s_e_current;
T_s_e_target([1 2 3],4) = X_s_t;

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
    
    %% Inverse kinematics
    
    % update angles of robot manipulator joints
    theta_current = PUMA_ikine(T_s_e_current,T_s_e_target,theta_current,joint_velocity_max,Hz);
    
    % update pose of robot end-effector
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






