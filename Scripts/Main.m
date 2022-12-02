%% Author: Jack Lambert
% University of Colorado Engineering Department
% 
% Purpose:  This codes purpose is to call the linear, non-linear, and
% initial condition functions in order to plot the simulated state
% variables for the different cases. This code calls the linear and non-
% linear functions using ODE45, which computes the differential equations
% for each of the state variables. This code plots the linear and
% non-linear cases state variables separately with respect to time, then
% compares them directly
% Date Modefied: 2/18/18
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ODE45 Variable Allocation
%                     x = z(1); % N - location
%                     dy = z(2); % E - location
%                     dz = z(3); % -D - location
%                     u = z(4); % u - component of velocity
%                     v = z(5); % v compenent of velocity
%                     w = z(6); % w component of velocity
%                     % Rotational Motion
%                     phi = z(7); % Attitude Euler Angles
%                     theta = z(8); % Attitude Euler Angles
%                     psi = z(9); % Attitude Euler Angles
%                     p = z(10); % Angular velocity about the x-axis [rad/s]
%                     q = z(11); % Angular Velocity about the y-axis [rad/s]
%                     r = z(12); % Angular Velocity about the z-axis [rad/s]
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Inital Conditions:
%                     i = 1 ----> +5 [deg] Bank
%                     i = 2 ----> +5 [deg] Pitch
%                     i = 3 ----> +5 [deg] Roll Rate
%                     i = 4 ----> +5 [deg] Pitch Rate
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all;
%% Initial Condition function
[conditionL,conditionNL] = InitialConditions();
% Time for ODE45 to numerically integrate over (Linear barely Varies)
timeNL = [0:0.001:10];
timeL = [0 10];
%% For title as conditions are change
string = ["+5 [deg] Bank","+5 [deg] Pitch","+0.1 [rad/s] Roll Rate",...
    "+0.1 [rad/s] Pitch Rate"];
%% Non Linear By Itself
for i = 1:4
    % Calling ODE45 
    [tNL,zNL] = ode45('NonLinear',timeNL,conditionNL{i});
    % Plotting Conditions
    figure
    % U_E vs time
    subplot(8,1,1)
    plot(tNL ,zNL(:,4),'Linewidth',1)
    tit = sprintf('%s %s','State Variable of a Quad-Copter w/',string(i));
    title(tit)
    ylabel('u_E [m/s]')
   
    % V_E vs time
    subplot(8,1,2)
    plot(tNL ,zNL(:,5),'Linewidth',1)
    ylabel('v_E [m/s]')
    
    % W_E vs time
    subplot(8,1,3)
    plot(tNL ,zNL(:,6),'Linewidth',1)
    ylabel('w_E [m/s]')
    
    % p vs time
    subplot(8,1,4)
    plot(tNL ,zNL(:,10),'Linewidth',1)
    ylabel('p [rad/s]')
    
    % q vs time
    subplot(8,1,5)
    plot(tNL ,zNL(:,11),'Linewidth',1)
    ylabel('q [rad/s]')
    
    % r vs time
    subplot(8,1,6)
    plot(tNL ,zNL(:,12),'Linewidth',1)
    ylabel('r [rad/s]')
    
    % Phi vs time
     subplot(8,1,7)
    plot(tNL ,zNL(:,7),'Linewidth',1)
    ylabel('\phi [rad]')
    
    % Theta vs time
    subplot(8,1,8)
    plot(tNL ,zNL(:,8),'Linewidth',1)
    ylabel('\theta [rad]')
    xlabel('Time [s]')
end

%% Linear By Itself
for i = 1:4
    % Calling ODE45 
    [tL,zL] = ode45('Linear',timeL,conditionL{i});
    
    % Plotting Conditions
    figure
    % U_E vs time
    subplot(8,1,1)
    plot(tL ,zL(:,4),'Linewidth',1)
    tit = sprintf('%s %s','State Variable of a Quad-Copter w/',string(i));
    title(tit)
    ylabel('u_E [m/s]')
    
    % V_E vs time
    subplot(8,1,2)
    plot(tL ,zL(:,5),'Linewidth',1)
    ylabel('v_E [m/s]')
    
    % W_E vs time
    subplot(8,1,3)
    plot(tL ,zL(:,6),'Linewidth',1)
    ylabel('w_E [m/s]')
    
    % p vs time
    subplot(8,1,4)
    plot(tL ,zL(:,10),'Linewidth',1)
    ylabel('p [rad/s]')
    
    % q vs time
    subplot(8,1,5)
    plot(tL ,zL(:,11),'Linewidth',1)
    ylabel('q [rad/s]')
    
    % r vs time
    subplot(8,1,6)
    plot(tL ,zL(:,12),'Linewidth',1)
    ylabel('r [rad/s]')
    
    % Phi vs time
     subplot(8,1,7)
    plot(tL ,zL(:,7),'Linewidth',1)
    ylabel('\phi [rad]')
    
    % Theta vs time
    subplot(8,1,8)
    plot(tL ,zL(:,8),'Linewidth',1)
    ylabel('\theta [rad]')
    xlabel('Time [s]')

end
%% Trajectory Linear vs Non-Linear
for i = 1:4
    [tNL,zNL] = ode45('NonLinear',timeNL,conditionNL{i});
    [tL,zL] = ode45('Linear',timeL,conditionL{i});
    
    figure
    plot3(zNL(:,1),zNL(:,2),-zNL(:,3),'-o')
    hold on
    plot3(zL(:,1),zL(:,2),-zL(:,3),'-o')
    tit = sprintf('%s %s','Trajectory of Quad-Copter w/',string(i));
    title(tit)
    xlabel('N Displacement [m]')
    ylabel('E Displacement [m]')
    zlabel('-D Displacement [m]')
    legend('Non-Linear','Linear')
    axis equal
end

%% Linear vs. Non-Linear ( State Variables vs Time)
for i = 1:4
    % Calling ODE45 
    [tL,zL] = ode45('Linear',timeL,conditionL{i});
    [tNL,zNL] = ode45('NonLinear',timeNL,conditionNL{i});
    
    % Plotting Conditions
    figure
    % U_E vs time
    subplot(8,1,1)
    plot(tL ,zL(:,4),'.','Linewidth',1)
    tit = sprintf('%s %s','State Variable of a Quad-Copter w/',string(i));
    title(tit)
    hold on
    plot(tNL ,zNL(:,4),'Linewidth',1)
    hold off
    ylabel('u_E [m/s]')
    legend('Linear','Non-Linear')
    
    % V_E vs time
    subplot(8,1,2)
    plot(tL ,zL(:,5),'.','Linewidth',1)
    hold on
    plot(tNL ,zNL(:,5),'Linewidth',1)
    hold off
    ylabel('v_E [m/s]')
    
    % W_E vs time
    subplot(8,1,3)
    plot(tL ,zL(:,6),'.','Linewidth',1)
    hold on
    plot(tNL ,zNL(:,6),'Linewidth',1)
    hold off
    ylabel('w_E [m/s]')

    % p vs time
    subplot(8,1,4)
    plot(tL ,zL(:,10),'.','Linewidth',1)
    hold on
    plot(tNL ,zNL(:,10),'Linewidth',1)
    hold off
    ylabel('p [rad/s]')
    
    % q vs time
    subplot(8,1,5)
    plot(tL ,zL(:,11),'.','Linewidth',1)
    hold on
    plot(tNL ,zNL(:,11),'Linewidth',1)
    hold off
    ylabel('q [rad/s]')
    
    % r vs time
    subplot(8,1,6)
    plot(tL ,zL(:,12),'.','Linewidth',1)
    hold on
    plot(tNL ,zNL(:,12),'Linewidth',1)
    hold off
    ylabel('r [rad/s]')
    
    % Phi vs time
     subplot(8,1,7)
    plot(tL ,zL(:,7),'.','Linewidth',1)
    hold on
    plot(tNL ,zNL(:,7),'Linewidth',1)
    hold off
    ylabel('\phi [rad/s]')
    
    % Theta vs time
    subplot(8,1,8)
    plot(tL ,zL(:,8),'.','Linewidth',1)
    hold on
    plot(tNL ,zNL(:,8),'Linewidth',1)
    hold off
    ylabel('\theta [rad/s]')
    xlabel('time [s]')
    
end