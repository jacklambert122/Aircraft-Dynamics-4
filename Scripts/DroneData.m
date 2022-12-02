%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Author: Jack Lambert

% Purpose: This code takes in the experimental data from the rolling
% spiders data and plots the results for the copter translation and
% rotation over time with respect to the controls in which we set the gains
% for
% Date Modefied: 2/18/18
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

load('./Data/RSdata_Drone09_1405.mat')

%% Motor Commands
timeMotor = rt_motor.time(:);
Motor1 = (rt_motor.signals.values(:,1)*13840.4).^(1/2); % Motor Rotation Rate [Rad/s]
Motor2 = (abs(rt_motor.signals.values(:,2)*13840.4)).^(1/2); % Motor Rotation Rate [Rad/s]
Motor3 = (rt_motor.signals.values(:,3)*13840.4).^(1/2); % Motor Rotation Rate [Rad/s]
Motor4 = (abs(rt_motor.signals.values(:,4)*13840.4)).^(1/2); % Motor Rotation Rate [Rad/s]

%% Translation

% Position
timeest = rt_estim.time(:);
xdata = rt_estim.signals.values(:,1); % X-Position [m]
ydata = rt_estim.signals.values(:,2); % Y-Position [m]
zdata = rt_estim.signals.values(:,3); % Z-Position [m]

% Velocity 
Vx = rt_estim.signals.values(:,7); % X-Velocity [m/s]
Vy = rt_estim.signals.values(:,8); % Y-Velocity [m/s]
Vz = rt_estim.signals.values(:,9); % Z-Velocity [m/s]

%% Rotation

% Attitude
yaw = rt_estim.signals.values(:,4); % [Rad]
pitch = rt_estim.signals.values(:,5); % [Rad]
roll = rt_estim.signals.values(:,6); % [Rad]

% Angular Rates
p = rt_estim.signals.values(:,10); % Body Fixed frame rotation about x-axis [Rad/s]
q = rt_estim.signals.values(:,11); % Body Fixed frame rotation about y-axis[Rad/s]
r = rt_estim.signals.values(:,12); % Body Fixed frame rotation about z-axis [Rad/s]

%% Plots of Translation vs. Time in Correlation to Motor Controls

% Motor Controls
figure(1)
subplot(3,1,1)
plot(timeMotor,Motor1)
hold on
plot(timeMotor,Motor2)
plot(timeMotor,Motor3)
plot(timeMotor,Motor4)
title('Motor Controls vs. Time')
xlabel('Time [s]')
ylabel('Motor Rotation Rate [rad/s]')
legend('Motor 1',' |Motor 2|','Motor 3','|Motor 4|')

% Position
subplot(3,1,2,'replace')
plot(timeest,xdata)
hold on
plot(timeest,ydata)
plot(timeest,zdata)
hold off
title('Position vs. Time')
ylabel('Position [m]')
xlabel('Time [s]')
legend('X-Position','Y-Position','Z-Position')

% Velocity
subplot(3,1,3,'replace')
plot(timeest,Vx)
hold on
plot(timeest,Vy)
plot(timeest,Vz)
hold off
title('Velocity vs. Time')
ylabel('Velocity [m]')
xlabel('Time [s]')
legend('X-Velocity','Y-Velocity','Z-Velocity')

%% Plots of Rotation vs. Time in Correlation to Motor Controls

% Motor Controls
figure(2)
subplot(3,1,1)
plot(timeMotor,Motor1)
hold on
plot(timeMotor,Motor2)
plot(timeMotor,Motor3)
plot(timeMotor,Motor4)
title('Motor Controls vs. Time')
xlabel('Time [s]')
ylabel('Motor Rotation Rate [rad/s]')
legend('Motor 1',' |Motor 2|','Motor 3','|Motor 4|')

% Attitude
subplot(3,1,2,'replace')
plot(timeest,roll)
hold on
plot(timeest,pitch)
plot(timeest,yaw)
hold off
title('Attitude vs. Time')
ylabel('Orientation [rad]')
xlabel('Time [s]')
legend('Roll','Pitch','Yaw')

% Anglar Velocity in Body fixed Frame
subplot(3,1,3,'replace')
plot(timeest,p)
hold on
plot(timeest,q)
plot(timeest,r)
hold off
title('Angular Velocity vs. Time')
ylabel('Angular Velocity [rad/s]')
xlabel('Time [s]')
legend('\omega_{x}','\omega_{y}','\omega_{z}')

