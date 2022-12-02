%% Author: Jack Lambert

% Purpose: This code gives the intial conditions for both the linear and
% non-linear models for ode45 to compute the diffeential equations. This
% function then plots the the trajectories and attitide for the linear and
% non-linear models with the variations in thier inital conditions whoch
% are set in an inital conditions matrix and iterated through for each case
% Date Modefied: 2/12/18
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Initial Condition function
[conditionL,conditionNL] = IntialConditions();
%% Devaitations for Non Linear Case
string = ["+5 [deg] Bank","+5 [deg] Pitch","+5 [deg] Azimuth",...
    "+0.1 [rad/s] Roll Rate", "+0.1 [rad/s] Pitch Rate", "+0.1 [rad/s] Yaw Rate"];

for i = 1:4
    [tNL,zNL] = ode45('NonLinear',[0 5],conditionNL{i});
    [tL,zL] = ode45('NonLinearTest',[0 5],conditionNL{i});
    figure(i)
    plot3(zNL(:,1),zNL(:,2),-zNL(:,3),'-o')
    hold on
    plot3(zL(:,1),zL(:,2),-zL(:,3),'-o')
    tit = sprintf('%s %s','Trajectory of Quad-Copter w/',string(i));
    title(tit)
    xlabel('N Displacement [m]')
    ylabel('E Displacement [m]')
    zlabel('-D Displacement [m]')
    legend('Non-Linear','Non-Linear New')
    axis equal
end
%% Plotting Rotation Differences
for i = 1:4
    [tNL,zNL] = ode45('NonLinear',[0 2],conditionNL{i});
    [tL,zL] = ode45('NonLinearTest',[0 2],conditionL{i});
    tit = sprintf('%s %s','Trajectory of Quad-Copter w/',string(i));
    if i == 1 
        % Plots Changes in Phi vs. Time
        figure
        plot(tNL,zNL(:,7),'LineWidth',1)
        hold on
        plot(tL,zL(:,7),'LineWidth',1)
        hold off
        title(tit)
        xlabel('time [s]')
        ylabel('Bank [rad]')
        legend('Non-Linear','Test')
        
    elseif i == 2
        % Plots Changes in theta versus time
        figure
        plot(tNL,zNL(:,8),'LineWidth',1)
        hold on
        plot(tL,zL(:,8),'LineWidth',1)
        hold off
        title(tit)
        xlabel('time [s]')
        ylabel('Pitch [rad]')
        legend('Non-Linear','Test')
        
    elseif i == 3
        % Plots Changes in Roll Rate vs time
        figure
        plot(tNL,zNL(:,10),'LineWidth',1)
        hold on
        plot(tL,zL(:,10),'LineWidth',1)
        hold off
        title(tit)
        xlabel('time [s]')
        ylabel('Roll Rate [rad/s]')
        legend('Non-Linear','Test')
        
        figure
        plot(tNL,zNL(:,7),'-o')
        hold on
        plot(tL,zL(:,7),'LineWidth',1)
        hold off
        title(tit)
        xlabel('time [s]')
        ylabel('Bank [rad]')
        legend('Non-Linear','Test')


    elseif i == 4
        % Plots changes in Pitch versus time
        figure
        plot(tNL,zNL(:,11),'-o')
        hold on
        plot(tL,zL(:,11),'LineWidth',1)
        hold off
        title(tit)
        xlabel('time [s]')
        ylabel('Pitch rate [rad/s]')
        legend('Non-Linear','Test')
        
        figure
        plot(tNL,zNL(:,8),'-o')
        hold on
        plot(tL,zL(:,8),'LineWidth',1)
        hold off
        title(tit)
        xlabel('time [s]')
        ylabel('Pitch [rad]')
        legend('Non-Linear','Test')
        
    
    end
  
end


