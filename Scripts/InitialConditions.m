%% Author: Jack Lambert

% Purpose: This code gives the intial conditions for both the linear and
% non-linear models for ode45 to compute the diffeential equations. This
% codes purpose is to organize
% Date Modefied: 2/18/18
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [conditionL,conditionNL] = InitialConditions()
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Linear Case
% Pertubations
delta_xE = [0 0 0 0]; % N - location [m]
delta_yE = [0 0 0 0]; % E - location [m]
delta_zE = [0 0 0 0]; % -D - location [m]
delta_u = [0 0 0 0]; % u - component of velocity [m/s]
delta_v = [0 0 0 0]; % v compenent of velocity [m/s]
delta_w = [0 0 0 0]; % w component of velocity [m/s]
delta_phi = [5*(pi/180) 0 0 0]; % Phi Euler Angle [rad]
delta_theta = [0 5*(pi/180) 0 0]; % Theta Euler Angle [rad]
delta_psi = [0 0 0 0]; % Psi Euler Angle [rad]
delta_p = [0 0 0.1 0]; % Angular velocity about the x-axis [rad/s]
delta_q = [0 0 0 0.1]; % Angular Velocity about the y-axis [rad/s]
delta_r = [0 0 0 0]; % Angular Velocity about the z-axis [rad/s]
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Initial Conditions for Non-Linear Case
c1 = [0 0 0 0]; % N - location [m]
c2 = [0 0 0 0]; % E - location [m]
c3 = [0 0 0 0]; % -D - location [m]
c4 = [0 0 0 0]; % u - component of velocity [m/s]
c5 = [0 0 0 0]; % v compenent of velocity [m/s]
c6 = [0 0 0 0]; % w component of velocity [m/s]
c7 = [5*(pi/180) 0 0 0]; % Phi Euler Angle [rad]
c8 = [0 5*(pi/180) 0 0]; % Theta Euler Angle [rad]
c9 = [0 0 0 0]; % Psi Euler Angle [rad]
c10 = [0 0 0.1 0]; % Roll Rate [rad/s]
c11 = [0 0 0 0.1]; % Pitch rate [rad/s]
c12 = [0 0 0 0]; % Yaw Rate [rad/s]
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Running through each of the Cases for the Variations
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Linear Case
for i = 1:4
    conditionL{i}= [(delta_xE(i)) (delta_yE(i)) (delta_zE(i))...
        (delta_u(i)) (delta_v(i)) (delta_w(i))...
        (delta_phi(i)) (delta_theta(i)) (delta_psi(i))...
        (delta_p(i)) (delta_q(i)) (delta_r(i))]; 
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Non-Linear Case
for i = 1:4
    conditionNL{i}= [c1(i) c2(i) c3(i) c4(i) c5(i) c6(i) c7(i)...
        c8(i) c9(i) c10(i) c11(i) c12(i)]; 
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%