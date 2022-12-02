%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Author: Jack Lambert

% Purpose: This function gives ODE45 all the differntial equations to
% iterate through over each time step for the Linear Model
% Date Modefied: 2/18/18
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [dydt] = Linear(t,y)
%% Constants
mass = 68/1000; % [kg]
L_arm = 6/100; % [m]
eta = 1*10^(-3); % Aerodynamic Coefficient for drag [N /(m/s)^2]
zeta = 3*10^(-3);  % Aerodynamic Coefficient for drag [N /(m/s)^2]
alpha = 2*10^(-6); % Aerodynamic Coefficient for drag [N /(rad/s)^2]
beta = 1*10^(-6); % Aerodynamic Coefficient for drag [N /(rad/s)^2]
Ix = 6.8*10^(-5); % MOI about x-axis [kg*m^2]
Iy = 9.2*10^(-5); % MOI about x-axis [kg*m^2]
Iz = 1.35*10^(-4); % MOI about x-axis [kg*m^2]
R = sqrt(2)/2*L_arm; % Distance to COG [m]
k = 0.0024; % [m]
g = 9.81; % [m/s^2]
%% Derivatives to be Integrated
% Translational Motion
delta_xE = y(1); % N - location
delta_yE = y(2); % E - location
delta_zE = y(3); % -D - location
delta_u = y(4); % u - component of velocity
delta_v = y(5); % v compenent of velocity
delta_w = y(6); % w component of velocity
% Rotational Motion
delta_phi = y(7); % Bank [rad]
delta_theta = y(8); % Pitch [rad]
delta_psi = y(9); % Azimuth [rad]
delta_p = y(10); % Roll Rate [rad/s]
delta_q = y(11); % Pitch Rate [rad/s]
delta_r = y(12); % Yaw Rate [rad/s]
%% Adding Feedback Control to Attitude

% Control Constants
k1 = 0.00136; % Derivative control constant for pitch Rate
k2 = 0.0106; % Proportional control constant for phi
k3 = 0.00184; % Derivative control constant for roll Rate
k4 = 0.0144;  % Proportional control constant for theta
k5 = 0.0012; % Derivative Control of yaw rate

% Controls needed for Rotation Control
bL = [(-k1*delta_p - k2*delta_phi), (-k3*delta_q - k4*delta_theta), (-k5*delta_r), 0];

% Geometry of quadcopter
AL = [-R R R -R;
    -R -R R R;
    -k k -k k;
    -1 -1 -1 -1];

% Computing forces based on Control gains and Forces
delta_f = AL\bL';

%% Translation Equations

% Transformation matrix expanded for position in inertial to position in
% body
dydt(1) = delta_u*cos(delta_theta)*cos(delta_psi)+ delta_v*(sin(delta_phi)...
    *sin(delta_theta)*cos(delta_psi)-cos(delta_phi)*sin(delta_psi))...
    + delta_w*(cos(delta_phi)*sin(delta_theta)*cos(delta_psi)+...
    sin(delta_phi)*sin(delta_psi)); % X_E [m]
dydt(2) = delta_u*cos(delta_theta)*sin(delta_psi)+ delta_v*(sin(delta_phi)...
    *sin(delta_theta)*sin(delta_psi)+cos(delta_phi)*cos(delta_psi))...
    + delta_w*(cos(delta_phi)*sin(delta_theta)*sin(delta_psi)-...
    sin(delta_phi)*cos(delta_psi)); % Y_E [m]
dydt(3) = -delta_u*sin(delta_theta)+delta_v*sin(delta_phi)*cos(delta_theta)...
    +delta_w*cos(delta_phi)*cos(delta_theta); % Z_E [m]

dydt(4) = -g*delta_theta; % u_E [m/s]
dydt(5) = g*delta_phi; % v_E [m/s]
dydt(6) = AL(4,:)*delta_f*(1/mass); % w_E [m/s]

%% Moments to Rotations
dydt(7) = delta_p; % Phi [rad]
dydt(8) = delta_q; % Theta [rad]
dydt(9) = delta_r; % Psi [rad]
dydt(10) = AL(1,:)*delta_f*(1/Ix); % p [rad/s]
dydt(11) = AL(2,:)*delta_f*(1/Iy); % q [rad/s]
dydt(12) = AL(3,:)*delta_f*(1/Iz); % r [rad/s]

dydt = dydt';