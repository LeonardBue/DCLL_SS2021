% *************************************************************************
%
%    Script: Main_SLIP_Raibert
%
% One of the main scripts for lecture_06.
%
% This script demonstrates the principle of Raibert's hopping controller.
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
%   See also FLOWMAP, JUMPMAP, JUMPSET, HYBRIDDYNAMICS

%% Initial Setup
% Make a clean sweep:
clear all
close all
clc
% Define the system and add all necessary folders to the path:
% Define the model:
modelName = 'SLIP';
% Define the controller:
controllerName = 'Raibert'; 
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
x =  0;    % [l] horizontal main body position
y =  1.095;% [l] vertical main body position
% Velocities:
dx =  1.1;% [sqrt(l_0*g)] ... velocity thereof
dy =  0;  % [sqrt(l_0*g)] ... velocity thereof
% Assemble into positions and rates vectors:
q0    = [x, y].';
dqdt0 = [dx, dy].';

% Discrete states:
phase  = 0; % The phase of the model  (flight after apex = 0, stance = 1, flight before apex = 2)
contPt = 0; % [l_0]  Horizontal position of the last ground contact
z0 = [phase, contPt]';

% System parameters are specific to the model and remain fixed:
p = systParam([]);


%% Define and run simulation:
% Simulations options:
simOptions.tSTART = 0;                       % initial time
simOptions.tMAX   = 10;                      % maximum simulation time
simOptions.graphOUTPUT = systGraphics(q0,p); % graphics output class (set to [] for no animation)
set(gcf,'Units','Normalized','OuterPosition',[0.0,0,0.5,1.0]);
simOptions.dtGraphOUT  = 0.1;                % time step between updates of the graphical output
simOptions.dtOUT  = [];                      % time step for numerical output (set it to empty for solver's default output)
simOptions.simulateSingleStride = false;     % Do a continuous simulation

% Controller function handle:
simOptions.controller = @Controller_Raibert;

% Run event-based simulation:
[t, q, dqdt, z] = HybridDynamics(q0, dqdt0, z0, p, simOptions);

%% Repeat over a longer duration without animation to show how the motion stabilizes:
simOptions.tMAX   = 100;  % maximum simulation time
simOptions.graphOUTPUT = [];    % graphics output class (set to [] for no animation)
% Run event-based simulation:
[t, q, dqdt, z] = HybridDynamics(q0, dqdt0, z0, p, simOptions);

%% Display results
figure('Name','SLIP model: dx and y of forward hopping with Raibert''s controller','Units','Normalized','OuterPosition',[0.5,0.5,0.5,0.5]);
grid on; hold on; box on;
plot(t,dqdt(1,:)');
plot(t,q(2,:)');
legend({'$\dot{x}$','$y$'}, 'interpreter','latex');
axis([0, 100, 0, 3])
drawnow();

% To show the controller effort, we have to re-evaluate the controller
% at each timestep.  Note that in Raibert's controller, this evaluation
% is done only once per stride at apex transit.
% Initialize output vector:
angAtt = t; 
for i=1:size(t,2)
    angAtt(:,i) = Controller_Raibert(t(i), q(:,i), dqdt(:,i), z(:,i), p);
end
figure('Name','SLIP model: angle of attack with Raibert''s controller','Units','Normalized','OuterPosition',[0.5,0.0,0.5,0.5]);
grid on; hold on; box on;
plot(t,angAtt);
legend({'angle of attack'}, 'interpreter','latex');

%% Do a theoretical analysis of stability:
% Find periodic motion:
% Use the initial velocity from before as initial guess for a periodic
% trajectory:
dxINIT = dx;
pCYC = p;  % use system parameters as before
zCYC = z0; % discrete states remain unchanged

% Some options for numerical root-search:
numOPTS = optimset('Algorithm','levenberg-marquardt',... 
                   'Display','iter',...
                   'MaxFunEvals',1000,...
                   'MaxIter',500,...
                   'TolFun',1e-10,...
                   'TolX',1e-10);

% Solve the non-linear equation defined in 'residual()'. The residual
% function runs a one-step simulation of the model using provided
% intinial states and parameters and returns the change in velocity dx
% at the end of the step. fsolve finds initial velocity such that this
% change is 0: 
dxCYC = fsolve(@(dx)residual(y,dx,zCYC,pCYC), dxINIT, numOPTS);
qCYC    = [0; y];
dqdtCYC = [dxCYC; 0];

% The stability of the linearized system is evaluated by computing the
% Floquet multipliers (Eigenvalues of the Monodromy matrix):
[eigenValuesCYC,eigenVectorsCYC] = floquet(qCYC, dqdtCYC, zCYC, pCYC);
disp([newline, 'Floquet analysis of the solution: ']);
disp('...found the following Eigenvalues:');
disp(eigenValuesCYC);
% If the solution is stable all eigenvalues must be inside the unit
% circle: 
if all(abs(eigenValuesCYC)<1)
    disp('Solution is stable')
else
    disp('Solution is unstable')
end
disp('...found the following Eigenvectors:');
disp(eigenVectorsCYC);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Helper functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% This function simulates one step of the model and returns the
% velocity residual of the periodicity constraint: P(x)-x
% INPUT:    yIN    -- initial apex height;
%           dxIN   -- initial apex speed;
%           zIN    -- initial discrete state;
%           p      -- vector of system parameters;
%           angAtt -- angle-of-attack.
% OUTPUT:   r -- change in apex velocity after one step.

function r_ = residual(yIN_, dxIN_, zIN_, p_)
    % Form the initial state vectors:
    qIN_    = [0; yIN_];
    dqdtIN_ = [dxIN_; 0];

    % Define simulation options:
    simOptions_.tSTART = 0;                            % initial time
    simOptions_.tMAX = 5;                              % maximum simulation time
    simOptions_.graphOUTPUT = [];                      % no graphical output
    simOptions_.controller = @Controller_Raibert;      % handle to the controller function
    simOptions_.dtOUT = [];                            % time step for numerical output (set it to empty for solver's default output)
    simOptions_.simulateSingleStride = true;           % Simulate only a single stride

    % Run one step simulation:
    [t_, q_, dqdt_, z_] = HybridDynamics(qIN_, dqdtIN_, zIN_, p_, simOptions_);

    % Return velocity deviation after one step:
    dxOUT_ = dqdt_(1,end);
    r_ = dxOUT_ - dxIN_;
end

% This function computes Floquet multipliers and eigenvectors of a SLIP
% model periodic solution.
% INPUT:    qCYC    -- initial CoM positions at apex, [x; y];
%           dqdtCYC -- initial CoM velocities at apex, [dx; dy];
%           pCYC    -- vector of system parameters.
% OUTPUT:   eigenValuesCYC  -- eigenvalues of the solution;
%           eigenVectorsCYC -- eigenvectors of teh solution.
% 
function [eigenValuesCYC_, eigenVectorsCYC_] = floquet(qCYC_, dqdtCYC_, zCYC_, pCYC_)
    % Define simulation options:
    simOptions_.tSTART = 0;                            % initial time
    simOptions_.tMAX = 5;                              % maximum simulation time
    simOptions_.graphOUTPUT = [];                      % no graphical output
    simOptions_.controller = @Controller_Raibert;      % handle to the controller function
    simOptions_.dtOUT = [];                            % time step for numerical output (set it to empty for solver's default output)
    simOptions_.simulateSingleStride = true;           % Simulate only a single stride

    % Small disturbance to compute Floquet multipliers:
    disturbance_ = 1e-6;
    % Initialize the Jacobian (Monodromy) matrix:
    J_ = zeros(3);

    for j_ = 1:3
        % The x-position does not stay the same after a step, so we do
        % not perturb it in calculation of the Floquet multipliers:
        % Perturbation in the positive direction:
        distVecPLUS_ = zeros(4,1);
        distVecPLUS_(j_+1) = disturbance_;
        [tPLUS_, qPLUS_, dqdtPLUS_, zPLUS_] = HybridDynamics(qCYC_+distVecPLUS_(1:2), dqdtCYC_+distVecPLUS_(3:4), zCYC_, pCYC_, simOptions_);
        % Perturbation in the negative direction:
        distVecMINUS_ = zeros(4,1);
        distVecMINUS_(j_+1) = -disturbance_;
        [tMINUS_, qMINUS_, dqdtMINUS_, zMINUS_] = HybridDynamics(qCYC_+distVecMINUS_(1:2), dqdtCYC_+distVecMINUS_(3:4), zCYC_, pCYC_, simOptions_);
        % Monodromy matrix (using central difference derivative estimate):
        J_(:,j_) = [qPLUS_(2,end)-qMINUS_(2,end); dqdtPLUS_(:,end)-dqdtMINUS_(:,end)] ./ (2*disturbance_);
    end
    % Conpute the eigenvalues and eigenvectors of the Monodromy matrix:
    [eigenVectorsCYC_, D_] = eig(J_);
    eigenValuesCYC_  = diag(D_);
end