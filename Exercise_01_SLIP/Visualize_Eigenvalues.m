% *************************************************************************
%
%    Script: Visualize_Eigenvalues
%
% Auxilliary script for exercise_01.
%
% This MATLAB script runs a periodic simulation of a simple SLIP (Spring
% Loaded Inverted Pendulum) model in 2D and visualizes the stability about
% this orbit.
%
% The script is using the "pause()" function to step through different
% steps of the visualization.
%
% INPUT:  - NONE
% OUTPUT: - NONE
%
%   C. David Remy remy@inm.uni-stuttgart.de
%   Matlab R2018
%   6/23/2020
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
controllerName = 'PassiveSLIP'; 
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


%% Define periodic states and parameters:
% Set periodic initial conditions:
% Positions:
x =  0;    % [l] horizontal main body position
y =  1.2;  % [l] vertical main body position
% Velocities:
dx =  2.5; % [sqrt(l_0*g)] ... velocity thereof
dy =  0;   % [sqrt(l_0*g)] ... velocity thereof
% Assemble into positions and rates vectors:
qCYC    = [x, y].';
dqdtCYC = [dx, dy].';

% Discrete states:
phase  = 0; % The phase of the model  (flight after apex = 0, stance = 1, flight before apex = 2)
contPt = 0; % [l_0]  Horizontal position of the last ground contact
zCYC = [phase, contPt].';

% System parameters are specific to the model and (apart from the angle of
% attack) remain fixed: 
pCYC = systParam([]);
% Fix the angle-of-attack to previously found value that gives periodic
% motion: 
angAtt = 0.586104935917454;
pCYC(5) = angAtt;

%% Define and run simulation:
% Simulations options:
simOptions.tSTART = 0;                       % initial time
simOptions.tMAX   = 5;                       % maximum simulation time
simOptions.graphOUTPUT = systGraphics(qCYC,pCYC); % graphics output class (set to [] for no animation)
set(gcf,'Units','Normalized','OuterPosition',[0.0,0,0.5,0.5]);
simOptions.dtGraphOUT  = 0.1;                % time step between updates of the graphical output
simOptions.dtOUT  = [];                      % time step for numerical output (set it to empty for solver's default output)
simOptions.simulateSingleStride = true;      % Simulate only a single stride.

% Controller function handle:
simOptions.controller = @Controller_PassiveSLIP;

%% Run event-based simulation for periodic orbit:
[t, q, dqdt, z] = HybridDynamics(qCYC, dqdtCYC, zCYC, pCYC, simOptions);
disp('press a key to continue');pause();

% Display the basic limit cycle with some characteristic surfaces:
figure('Name','Orbits of the SLIP model','Units','Normalized','OuterPosition',[0.5,0.0,0.5,1.0]);
grid on; hold on; box on;
axis equal
axis([0.5,1.5,2,3,-1,1])
view(3)
xlabel('$y$', 'interpreter','latex');
ylabel('$\dot{x}$', 'interpreter','latex');
zlabel('$\dot{y}$', 'interpreter','latex');
lighting PHONG
camlight right
% Plot starting point and orbit
plot3(q(2,1), dqdt(1,1), dqdt(2,1),'k*');
plot3(q(2,:), dqdt(1,:), dqdt(2,:),'k','LineWidth',2);
disp('press a key to continue');pause();
% Plot poincare section
surf([0.5,1.5],[2,3],[0,0;0,0],'FaceColor',[0,0,1],'FaceAlpha',0.3);
disp('press a key to continue');pause();
% Plot surface of constant kinetic and gravitational energy E (excluding
% the energy stored in the spring (which is also a function of the discrete
% state z)
E = qCYC(2) + 0.5*dqdtCYC(1)^2 + 0.5*dqdtCYC(2)^2;
[dX,dY] = meshgrid(2.4:.01:2.7, -1:.01:1);
Y = E - 0.5*dX.^2 - 0.5*dY.^2;
surf(Y,dX,dY,'FaceColor',[1,0,0],'FaceAlpha',0.3,'EdgeColor','none')
disp('press a key to continue');pause();

% Stability analysis
% The stability of the linearized system is evaluated by computing the
% Floquet multipliers (Eigenvalues of the Monodromy matrix):
[eigenValuesCYC,eigenVectorsCYC] = floquet(qCYC, dqdtCYC, zCYC, pCYC);
% For visualization it is nicer to flip the sign of the EVs (which is
% mathematically irrelevant):
eigenVectorsCYC = -eigenVectorsCYC; 
% Display numerical results:
disp([newline, 'Floquet analysis of the solution: ']);
disp('...found the following Eigenvalues:');
disp(eigenValuesCYC);
% If the solution is stable all eigenvalues must be inside the unit
% circle: 
if all(abs(eigenValuesCYC)<1)
    disp('Solution is stable')
elseif any(abs(eigenValuesCYC)>1)
    disp('Solution is unstable')
else 
    disp('Solution is marginally stable')
end
disp('...found the following Eigenvectors:');
disp(eigenVectorsCYC);
disp('press a key to continue');pause();

% Show eigenvectors
pbaspect manual
yCYC_ = [qCYC(2); dqdtCYC(1); dqdtCYC(2)];
arrow3(yCYC_.',yCYC_.'+eigenVectorsCYC(:,1).'*0.3,'r')
arrow3(yCYC_.',yCYC_.'+eigenVectorsCYC(:,2).'*0.3,'g')
arrow3(yCYC_.',yCYC_.'+eigenVectorsCYC(:,3).'*0.3,'b')
pbaspect auto
disp('press a key to continue');pause();

% Show disturbed limit cycles:
simOptions.graphOUTPUT = [];   % no graphical output for the reruns
% EV 3 (0)
yDIST = [0;yCYC_+eigenVectorsCYC(:,3)*0.05];
qDIST = yDIST(1:2);
dqdtDIST = yDIST(3:4);
[t, q, dqdt, z] = HybridDynamics(qDIST, dqdtDIST, zCYC, pCYC, simOptions);
plot3(q(2,1), dqdt(1,1), dqdt(2,1),'b*');
plot3(q(2,:), dqdt(1,:), dqdt(2,:),'b','LineWidth',3);
disp('press a key to continue');pause();
% Zoom in
axis([yCYC_(1)-0.1,yCYC_(1)+0.1,yCYC_(2)-0.1,yCYC_(2)+0.1,yCYC_(3)-0.1,yCYC_(3)+0.1]);
disp('press a key to continue');pause();
% EV 2 (~0.7)
yDIST = [0;yCYC_+eigenVectorsCYC(:,2)*0.05];
qDIST = yDIST(1:2);
dqdtDIST = yDIST(3:4);
[t, q, dqdt, z] = HybridDynamics(qDIST, dqdtDIST, zCYC, pCYC, simOptions);
plot3(q(2,1), dqdt(1,1), dqdt(2,1),'g*');
plot3(q(2,:), dqdt(1,:), dqdt(2,:),'g','LineWidth',3);
disp('press a key to continue');pause();
% EV 1 (1)
yDIST = [0;yCYC_+eigenVectorsCYC(:,1)*0.05];
qDIST = yDIST(1:2);
dqdtDIST = yDIST(3:4);
[t, q, dqdt, z] = HybridDynamics(qDIST, dqdtDIST, zCYC, pCYC, simOptions);
plot3(q(2,1), dqdt(1,1), dqdt(2,1),'r*');
plot3(q(2,:), dqdt(1,:), dqdt(2,:),'r','LineWidth',3);


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
    simOptions_.controller = @Controller_PassiveSLIP;  % handle to the controller function
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

