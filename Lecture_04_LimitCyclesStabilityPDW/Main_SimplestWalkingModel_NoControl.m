% *************************************************************************
%
%    Script: Main_SimplestWalkingModel_NoControl
%
% Main script for lecture_04.
%
% This MATLAB script runs simulations of the simplest walking model in 2D,
% finds its periodic motions, and analyzes their stability.
%
% In particular, it sets up the framework and include all necessary files,
% folders, and variables.
% Then it is used to:
% - (a) run a basic simulations while creating animations;
% - (b) find a periodic walking motion;
% - (c) conduct a first order stability analysis of the periodic motion;
% - (d) conduct a parameter study on slope angle;
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
modelName = 'SimplestWalkingModel';
% Define the controller:
controllerName = 'NoControl'; 
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
theta  =  0.2;  % [rad] stance leg angle
phi    =  0.4;  % [rad] angle between the legs
% Velocities:
dtheta = -0.2;  % [rad/sqrt(l/g)] ... angular velocity thereof
dphi   =  0;    % [rad/sqrt(l/g)] ... angular velocity thereof
% Assemble into positions and rates vectors:
q0    = [theta, phi].';
dqdt0 = [dtheta, dphi].';

% Discrete states:
z0 = [];

% System parameters are specific to the model and remain fixed:
p = systParam([]);


%% Define and run simulation:
% Simulations options:
simOptions.tSTART = 0;                       % initial time
simOptions.tMAX   = 5;                       % maximum simulation time
simOptions.graphOUTPUT = systGraphics(q0,p); % graphics output class (set to [] for no animation)
set(gcf,'Units','Normalized','OuterPosition',[0.0,0,0.5,1.0]);
simOptions.dtGraphOUT  = 0.1;                % time step between updates of the graphical output
simOptions.dtOUT  = [];                      % time step for numerical output (set it to empty for solver's default output)
simOptions.simulateSingleStride = true;      % Simulate only a single stride

% Controller function handle:
simOptions.controller = @Controller_NoControl;

%% (a) Run event-based simulation to simulate a single stride with graphical
% output:     
[t, q, dqdt, z] = HybridDynamics(q0, dqdt0, z0, p, simOptions);

% Display results
figure('Name','Simplest Walker: single stride of a non-periodic motion','Units','Normalized','OuterPosition',[0.5,0.5,0.5,0.5]);
grid on; hold on; box on;
plot(t,q');
plot(t,dqdt');
legend({'$\theta$','$\phi$','$\dot{\theta}$','$\dot{\phi}$'}, 'interpreter','latex');
drawnow();

%% (b) Create a periodic gait
% For a passive dynamic walker, the problem of gait creation is simply
% to find periodic initial conditions, i.e. states in which the model
% is started and that are reached again at the end of one step.
yINIT = [q0; dqdt0]; % initial guess for the solution
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
yCYC = fsolve(@(yTRY)residual(yTRY,zCYC,pCYC), yINIT, numOPTS);
qCYC    = yCYC(1:end/2);
dqdtCYC = yCYC(end/2+1:end);

% Run a one-step simulation to see that the solution is indeed
% periodic, and initial and final states are identical: 
[t, q, dqdt, z] = HybridDynamics(qCYC, dqdtCYC, zCYC, pCYC, simOptions);
disp([newline, 'Deviation after one step: ']);
disp(num2str([q(:,end)-qCYC; dqdt(:,end)-dqdtCYC]));

% Display results
figure('Name','Simplest Walker: single stride of a periodic motion','Units','Normalized','OuterPosition',[0.5,0.0,0.5,0.5]);
grid on; hold on; box on;
plot(t,q');
plot(t,dqdt');
legend({'$\theta$','$\phi$','$\dot{\theta}$','$\dot{\phi}$'}, 'interpreter','latex');
drawnow();


%% (c) Stability analysis
% The stability of the linearized system is evaluated by computing the
% Floquet multipliers (Eigenvalues of the Monodromy matrix):
[eigenValuesCYC,eigenVectorsCYC] = floquet(qCYC, dqdtCYC, zCYC, pCYC);
disp([newline, 'Floquet analysis of the solution: ']);
disp('...found the following Eigenvalues:');
disp(eigenValuesCYC);
% If the solution is stable all eigenvalues must be inside the unit circle:
if all(abs(eigenValuesCYC)<1)
    disp('Solution is stable')
else
    disp('Solution is unstable')
end
disp('...found the following Eigenvectors:');
disp(eigenVectorsCYC);


%% (d) Extended parameter study
% The influence of the slope gamma on the stability is analyzed, and
% displayed in a root locus plot.  

% Set up the parameter range that will be studied:
N        = 21;                    % # of Samples
gammas   = linspace(0.01,0.02,N); % Vector of slope angles gamma
eVals    = zeros(4,N);            % To store the computed eigenvalues for each parameter
qCYCs    = zeros(2,N);            % To store the computed periodic angles
dqdtCYCs = zeros(2,N);            % To store the computed periodic velocities
for i = 1:N
    disp(' ');
    disp('******************************');
    disp([' Step ',num2str(i),': gamma = ',num2str(gammas(i))]);
    disp(' ');
    % Recycle the last peridic solution as initial guess.  This improves
    % convergence
    yINIT = yCYC;
    % Adapt the slope angle value: 
    pCYC = gammas(i);

    % Find a periodic solution:
    yCYC = fsolve(@(yTRY)residual(yTRY,zCYC,pCYC), yINIT);
    qCYCs(:,i)    = yCYC(1:end/2);
    dqdtCYCs(:,i) = yCYC(end/2+1:end);

    % Compute the Floquet-multipliers
    eigenValuesCYC = floquet(qCYCs(:,i), dqdtCYCs(:,i), zCYC, pCYC);

    % Store the multipliers:
    eVals(:,i) = eigenValuesCYC;
end

% prepare root locus plot:
figure('Name','Simplest Walker: Root Locus Plot - Slope variation','Units','Normalized','OuterPosition',[0.5,0.0,0.5,1.0]);
hold on; grid on; box on;
% plot the unit circle
plot(sin(linspace(0,2*pi)),cos(linspace(0,2*pi)),'r')
axis equal
% plot the individual eigenvalues:
caxis([min(gammas),max(gammas)])
map = jet(N);
for i = 1:N
    plot(real(eVals(:,i)),imag(eVals(:,i)),'Color',map(i,:), 'Marker','o','MarkerSize',6,'LineStyle','none');
end
% Graph annotation:
colormap(map)
colorbar();
xlabel('Re \lambda')
ylabel('Im \lambda') 
title(sprintf('Simplest walking model: \n eigenvalue root locus for variation in slope'));
axis([-2.5,1.5,-1.5,1.5]);
drawnow();
% Save the figure to image files:
fig = gcf; set(fig, 'PaperType','<custom>', 'PaperSize',[20,10], 'PaperPosition', [0,0,20,10])
print(fig,'-r600','-djpeg','Figure1.jpg');

%% (e) Show examples of a stable (gamma = 0.01) and unstable (gamma = 0.02) solution:
% Update simulation options:
simOptions.tMAX   = 50;                      % maximum simulation time
simOptions.simulateSingleStride = false;     % Run continuous simulation
% (e-1) stable:
pCYC    = gammas(1);
qCYC    = qCYCs(:,1)+[1e-4;0];  % And add a small disturbance
dqdtCYC = dqdtCYCs(:,1);
[t, q, dqdt, z] = HybridDynamics(qCYC, dqdtCYC, zCYC, pCYC, simOptions);
figure('Name','Simplest Walker: multiple strides of a stable periodic motion','Units','Normalized','OuterPosition',[0.5,0.5,0.5,0.5]);
grid on; hold on; box on;
plot(t,q');
plot(t,dqdt');
legend({'$\theta$','$\phi$','$\dot{\theta}$','$\dot{\phi}$'}, 'interpreter','latex');
% (e-2) unstable:
pCYC    = gammas(N);
qCYC    = qCYCs(:,N)+[1e-4;0];  % And add a small disturbance
dqdtCYC = dqdtCYCs(:,N);
[t, q, dqdt, z] = HybridDynamics(qCYC, dqdtCYC, zCYC, pCYC, simOptions);
figure('Name','Simplest Walker: multiple strides of an unstable periodic motion','Units','Normalized','OuterPosition',[0.5,0.0,0.5,0.5]);
grid on; hold on; box on;
plot(t,q');
plot(t,dqdt');
legend({'$\theta$','$\phi$','$\dot{\theta}$','$\dot{\phi}$'}, 'interpreter','latex');


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Helper functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% This function simulates one step of the model and returns the
% velocity residual of the periodicity constraint: P(x)-x
% INPUT:    yIN    -- initial continuous state;
%           zIN    -- initial discrete state;
%           p      --  parameter values;
% OUTPUT:   r -- change in state after one step.

function r_ = residual(yIN_, zIN_, p_)
    % Extract positions and velocities:
    qIN_    = yIN_(1:end/2);
    dqdtIN_ = yIN_(end/2+1:end);

    % Define simulation options:
    simOptions_.tSTART = 0;                            % initial time
    simOptions_.tMAX = 5;                              % maximum simulation time
    simOptions_.graphOUTPUT = [];                      % no graphical output
    simOptions_.controller = @Controller_NoControl;    % handle to the controller function
    simOptions_.dtOUT = [];                            % time step for numerical output (set it to empty for solver's default output)
    simOptions_.simulateSingleStride = true;           % Simulate only a single stride

    % Run one step simulation:
    [t_, q_, dqdt_, z_] = HybridDynamics(qIN_, dqdtIN_, zIN_, p_, simOptions_);

    % Return velocity deviation after one step:
    yOUT_ = [q_(:,end); dqdt_(:,end)];
    r_ = (yOUT_-yIN_);
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
    simOptions_.controller = @Controller_NoControl;    % handle to the controller function
    simOptions_.dtOUT = [];                            % time step for numerical output (set it to empty for solver's default output)
    simOptions_.simulateSingleStride = true;           % Simulate only a single stride

    % Small disturbance to compute finite differences:
    disturbance_ = 1e-6;
    % Initialize the Jacobian:
    J_ = zeros(4);

    for j_ = 1:4
        % Perturbation in the positive direction:
        distVecPLUS_ = zeros(4,1);
        distVecPLUS_(j_) = disturbance_;
        [tPLUS_, qPLUS_, dqdtPLUS_, zPLUS_] = HybridDynamics(qCYC_+distVecPLUS_(1:2), dqdtCYC_+distVecPLUS_(3:4), zCYC_, pCYC_, simOptions_);
        % Perturbation in the negative direction:
        distVecMINUS_ = zeros(4,1);
        distVecMINUS_(j_) = -disturbance_;
        [tMINUS_, qMINUS_, dqdtMINUS_, zMINUS_] = HybridDynamics(qCYC_+distVecMINUS_(1:2), dqdtCYC_+distVecMINUS_(3:4), zCYC_, pCYC_, simOptions_);
        % Jacobian (using central difference derivative estimate):
        J_(:,j_) = [qPLUS_(:,end)-qMINUS_(:,end); dqdtPLUS_(:,end)-dqdtMINUS_(:,end)] ./ (2*disturbance_);
    end
    % Compute the eigenvalues and eigenvectors of the Jacobian:
    [eigenVectorsCYC_, D_] = eig(J_);
    eigenValuesCYC_  = diag(D_);
end