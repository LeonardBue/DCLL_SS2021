% *************************************************************************
%
%    Script: Main_7LinkBiped_floatBase_StaticLocomotion
%
% One of the main scripts for lecture_06.
%
% This script demonstrates a statically stable, kinematically controlled
% walking motion of a floating base model of a 7-link walker.
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
% Define the system and add all necessary folders to the path:
% Define the model:
modelName = '7LinkBiped_floatBase';
% Define the controller:
controllerName = 'StaticLocomotion'; 
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


%% Define initial states and parameters:
% Set the initial conditions:
% Positions:
x      = -0.1;  % [l] horizontal main body position
y      =  0.85; % [l] vertical main body position
phi    =  0;    % [rad] angle of the main body with the vertical (0 = orthog to the ground, >0 is ccw)
alphaL =  0.61; % [rad] angle of the left thigh wrt the main body (0 = parallel to mb, >0 is ccw)
alphaR =  0.47; % [rad] angle of the right thigh wrt the main body (0 = parallel to mb, >0 is ccw)
betaL  = -1.29; % [rad] angle of the left shank wrt the left thigh (0 = straight knee, >0 is ccw)
betaR  = -1.25; % [rad] angle of the right shank wrt the right thigh (0 = straight knee, >0 is ccw)
gammaL =  2.18; % [rad] angle of the left foot wrt the left shank (0 = straight ankle, >0 is ccw)
gammaR =  2.35; % [rad] angle of the right foot wrt the right shank (0 = straight ankle, >0 is ccw)
% Velocities:
dx      =  0;  % [sqrt(l_0*g)] ... velocity thereof
dy      =  0;  % [sqrt(l_0*g)] ... velocity thereof
dphi    =  0;  % [rad/sqrt(l_0/g)] ... angular velocity thereof
dalphaL =  0;  % [rad/sqrt(l_0/g)] ... angular velocity thereof
dalphaR =  0;  % [rad/sqrt(l_0/g)] ... angular velocity thereof
dbetaL  =  0;  % [rad/sqrt(l_0/g)] ... angular velocity thereof
dbetaR  =  0;  % [rad/sqrt(l_0/g)] ... angular velocity thereof
dgammaL =  0;  % [rad/sqrt(l_0/g)] ... angular velocity thereof
dgammaR =  0;  % [rad/sqrt(l_0/g)] ... angular velocity thereof
% Assemble into positions and rates vectors:
q0    = [x, y, phi, alphaL, alphaR, betaL, betaR, gammaL, gammaR].';
dqdt0 = [dx, dy, dphi, dalphaL, dalphaR, dbetaL, dbetaR, dgammaL, dgammaR].';

% Discrete states:
% At the moment, the controller is only used with the time-stepping
% solver, hence no discrete states:
z0 = [];

% System parameters are specific to the model and remain fixed:
p = systParam([]);


%% Show the kinematic trajectories of the feet:
DrawKinematicTrajectories()
AnimateKinematicTrajectories()
%% TODO:  INCLUDE THESE AS SUBFUNCTIONS

%% Define and run simulation:
% Simulations options:
simOptions.tSTART = 0;                       % initial time
simOptions.tMAX   = 25;                      % maximum simulation time
simOptions.graphOUTPUT = systGraphics(q0,p); % graphics output class (set to [] for no animation)
set(gcf,'Units','Normalized','OuterPosition',[0.0,0,0.5,1.0]);
simOptions.dtGraphOUT  = 0.1;                % time step between updates of the graphical output
simOptions.dtOUT  = [];                      % time step for numerical output (set it to empty for solver's default output)
simOptions.dt = 0.001;                       % size of the time step
simOptions.simulateSingleStride = false;     % Do a continuous simulation

% Controller function handle:
simOptions.controller = @Controller_StaticLocomotion;

% Run simulation with time-stepping:
[t, q, dqdt, z] = TimeStepping(q0, dqdt0, z0, p, simOptions);

%% Display results
figure('Name','Seven Link Walker: Generalized coordinates of walking','Units','Normalized','OuterPosition',[0.0,0.5,0.5,0.5]);
grid on; hold on; box on;
plot(t,q');
legend({'$x$','$y$','$\phi$','$\alpha_L$','$\alpha_R$','$\beta_L$','$\beta_R$','$\gamma_L$','$\gamma_R$'}, 'interpreter','latex');

figure('Name','Seven Link Walker: Generalized speeds of walking','Units','Normalized','OuterPosition',[0.0,0.0,0.5,0.5]);
grid on; hold on; box on;
plot(t,dqdt');
legend({'$\dot{x}$','$\dot{y}$','$\dot{\phi}$','$\dot{\alpha}_L$','$\dot{\alpha}_R$','$\dot{\beta}_L$','$\dot{\beta}_R$','$\dot{\gamma}_L$','$\dot{\gamma}_R$'}, 'interpreter','latex');

figure('Name','Seven Link Walker: Contact Flags of walking','Units','Normalized','OuterPosition',[0.5,0.5,0.5,0.5]);
grid on; hold on; box on;
plot(t,z');
legend({'$Ankle_L$','$Ankle_R$','$Toe_L$','$Toe_R$'}, 'interpreter','latex');
axis([0,25,-0.5,1.5])

% To show the controller effort, we have to re-evaluate the controller
% at each timestep:
% Initialize torque vector:
u = q; 
for i=1:size(t,2)
    u(:,i) = Controller_StaticLocomotion(t(i), q(:,i), dqdt(:,i), z(:,i), p);
end
figure('Name','Seven Link Walker: Torques of walking','Units','Normalized','OuterPosition',[0.5,0.0,0.5,0.5]);
grid on; hold on; box on;
plot(t,u');
legend({'$\tau_x$','$\tau_y$','$\tau_\phi$','$\tau_{\alpha_L}$','$\tau_{\alpha_R}$','$\tau_{\beta_L}$','$\tau_{\beta_R}$','$\tau_{\gamma_L}$','$\tau_{\gamma_R}$'}, 'interpreter','latex');
axis([0,25,-0.5,0.5])