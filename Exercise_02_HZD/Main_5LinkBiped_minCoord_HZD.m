% *************************************************************************
%
%    Script: Main_5LinkBiped_minCoord_HZD
%
% Main script for exercise_02 (incomplete).
%
% This script demonstrates the concept of a hybrid zero dynamics controller
% that is employed to the model of a minimal coordinate model of a 5-link
% walker.
%
% INPUT:  - NONE
% OUTPUT: - NONE
%
%   C. David Remy remy@inm.uni-stuttgart.de
%   Matlab R2018
%   4/22/2020
%   v21
%
% Based on the paper:
%  'A MATLAB Framework For Gait Creation', 2011, C. David Remy, Keith
%  Buffinton, and Roland Siegwart,  International Conference on
%  Intelligent Robots and Systems, September 25-30, San Francisco, USA 
%
%   See also SYMBOLICCOMPUTATIONOFEOM, FLOWMAP, JUMPMAP, JUMPSET,
%   TIMESTEPPING, HYBRIDDYNAMICS

%% Initial Setup
% Make a clean sweep:
clear all
close all
clc
ChangeMe = 0;
% Define the system and add all necessary folders to the path:
% Define the model:
modelName = '5LinkBiped_minCoord';
% Define the controller:
controllerName = 'HZD_incomplete'; 
% Move one folder level up and include the library folders
% "Library\Shared", as well as the desired model and controller folders
% from "Library\Controllers" and "Library\Models" into the path.  Make
% sure that you downloaded these from the lecture pages and that your
% local folder structure matches the structure given there. 
currentDir = pwd;
cd('..');
if (~exist('Library','dir'))
    error('Could not include all necessary library files. Please download the library folder and make sure that you run this file from the lecture folder')
end
% Reset the MATLAB search path to its default value:
path(pathdef);
% Add to the path all shared files:
if isunix()
    path(genpath('Library/Shared'), path);
    path(genpath(['Library/Models/',modelName]), path);
    path(genpath(['Library/Controllers/',controllerName]), path);
else
    path(genpath('Library\Shared'), path);
    path(genpath(['Library\Models\',modelName]), path);
    path(genpath(['Library\Controllers\',controllerName]), path);
end
% Go back to original directory:
cd(currentDir);


%% Set up initial conditions

% Set the default initial conditions:
% Positions:
q1  =  0.1;  % [rad] angle of the stance shank wrt vertical (0 = orthog to the ground, >0 is ccw)
q2  = -0.1;  % [rad] angle of the stance shank wrt the stance thigh (0 = straight knee, >0 is ccw)
q3  =  0.3;  % [rad] angle of the stance thigh wrt the main body (0 = parallel to mb, >0 is ccw)
q4  = -0.2;  % [rad] angle of the swing thigh wrt the main body (0 = parallel to mb, >0 is ccw)
q5  = -0.4;  % [rad] angle of the swing shank wrt the swing thigh (0 = straight knee, >0 is ccw)
% Velocities:
dq1_dt = -0.2;  % [rad/sqrt(l_0/g)] ... angular velocity thereof
dq2_dt =  0;    % [rad/sqrt(l_0/g)] ... angular velocity thereof
dq3_dt = -0.3;  % [rad/sqrt(l_0/g)] ... angular velocity thereof
dq4_dt =  0;    % [rad/sqrt(l_0/g)] ... angular velocity thereof
dq5_dt =  0;    % [rad/sqrt(l_0/g)] ... angular velocity thereof
% Assemble into positions and rates vectors:
q0     = [q1, q2, q3, q4, q5].';
dq_dt0 = [dq1_dt, dq2_dt, dq3_dt, dq4_dt, dq5_dt].';

% The initial conditions defined above are not consistent with the virtual
% constraints. Let's find some initial conditions that are.  To do so we
% use our function h_D. 
%%%% CODE 1.5.2 complete the following %%%%
% First we need to know our phase
th     = [-1, 1/2, 0, 0, 0] * q0;
dth_dt = [-1, 1/2, 0, 0, 0] * dq_dt0;
[hD, dhD_dth, ddhD_ddth] = targetEvolution(th);
dhD_dt = dhD_dth*dth_dt; % compute the time-derivative of h_D

% Use these as initial conditions:
q0     = [q0(1);hD]; % ChangeMe
dq_dt0 = [dq_dt0(1);dhD_dt]; % ChangeMe
%%%% End 1.5.2 %%%%

% No discrete states:
z0 = [];

% System parameters are specific to the model and remain fixed:
p = systParam([]);


%% Define and run simulation:
% Simulations options:
simOptions.tSTART = 0;                       % initial time
simOptions.tMAX   = 15;                      % maximum simulation time
simOptions.graphOUTPUT = systGraphics(q0,p); % graphics output class (set to [] for no animation)
set(gcf,'Units','Normalized','OuterPosition',[0.0,0,1.0,1.0]);
simOptions.dtGraphOUT  = 0.1;                % time step between updates of the graphical output
simOptions.dtOUT  = [];                      % time step for numerical output (set it to empty for solver's default output)
simOptions.simulateSingleStride = false;     % Do a continuous simulation

% Controller function handle:
simOptions.controller = @Controller_HZD;

% Run event-based simulation:
[t, q, dqdt, z] = HybridDynamics(q0, dq_dt0, z0, p, simOptions);

% Visualize the motion:
figure('Name','HZD Walker: Leg Angles','WindowStyle','docked')
grid on; hold on; box on;
plot(t,q(2:5,:)');
legend({'$q_2$','$q_3$','$q_4$','$q_5$'}, 'interpreter','latex');

% Reconstruct theta and dthetadt for each timestep from the trajectories q,
% dqdt:
%%%% CODE 3.1.3 complete this ****
theta_actual     = ChangeMe;
dtheta_dt_actual = ChangeMe;

% Run the zeroDynamics function to get predictions of the phase
[t_predicted, theta_predicted, dtheta_dt_predicted] = zeroDynamics(theta_actual(1),dtheta_dt_actual(1));

figure('Name','HZD Walker: Zero Dynamics','WindowStyle','docked')
grid on; hold on; box on;
plot(theta_actual,dtheta_dt_actual,'.-');
xlabel('theta'); ylabel('dtheta_dt');
plot(theta_predicted, dtheta_dt_predicted,'r--');
%%%% End 3.1.3

%%%% CODE 3.1.4 complete this ****
% Reconstruct the trajectory for the error y = (q_a - h_d) from your
% simulated data.

y = ChangeMe;

figure('Name','HZD Walker: Constraints','WindowStyle','docked')
grid on; hold on; box on;
plot(t,y);
legend({'$y_1$','$y_2$','$y_3$','$y_4$'}, 'interpreter','latex');
%%%% End 3.1.1

figure('Name','HZD Walker: Desired and actual joint angles','WindowStyle','docked')
grid on; hold on; box on;
plot(t,q_a);
plot(t,H_d');
legend({'$q_2$','$q_3$','$q_4$','$q_5$','$h_{d,1}$','$h_{d,2}$','$h_{d,3}$','$h_{d,4}$'}, 'interpreter','latex');
