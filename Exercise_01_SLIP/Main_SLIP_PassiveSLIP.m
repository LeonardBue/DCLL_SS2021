% *************************************************************************
%
%    Script: Main_SLIP_PassiveSLIP
%
% Main script for exercise_01 (incomplete).
%
% This MATLAB script runs simulations of a simple SLIP (Spring Loaded 
% Inverted Pendulum) model in 2D, finds its periodic motions, and analyzes
% their stability.
%
% In particular, it sets up the framework and include all necessary files,
% folders, and variables.
% Then it is used to:
% - (a) run a basic simulations while creating animations;
% - (b) find a periodic walking motion;
% - (c) conduct a first order stability analysis of the periodic motion;
% - (d) analyze the reaction to disturbances of zero energy;
% - (e) compare motions inside and outside the basin of attraction;
% - (f) perform a continuation study to find a branch of periodic solutions
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

% ************************************
% ************************************
% TASK 0: RUN THIS CODE SECTION TO INITIALIZE THE FRAMEWORK
% ************************************
% ************************************

% Make a clean sweep:
clear all
close all
clc
CHANGEME = 0;
% Define the system and add all necessary folders to the path:
% Define the model:
modelName = 'SLIP_incomplete';
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


%% Define initial states and parameters:
% Set the initial conditions:
% Positions:
x =  0;    % [l] horizontal main body position
y =  1.2;  % [l] vertical main body position
% Velocities:
dx =  0;  % [sqrt(l_0*g)] ... velocity thereof
dy =  0;  % [sqrt(l_0*g)] ... velocity thereof
% Assemble into positions and rates vectors:
q0    = [x, y].';
dqdt0 = [dx, dy].';

% Discrete states:
phase  = 0; % The phase of the model  (flight after apex = 0, stance = 1, flight before apex = 2)
contPt = 0; % [l_0]  Horizontal position of the last ground contact
z0 = [phase, contPt].';

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
simOptions.simulateSingleStride = false;     % Do a continuous simulation

% Controller function handle:
simOptions.controller = @Controller_PassiveSLIP;

%% (a) Run event-based simulation for different conditions:

% ************************************
% ************************************
% TASK 5-1: RUN THIS TO CHECK IF YOUR DYNAMICS WORK CORRECTLY
% ************************************
% ************************************

% Hopping in place
[t, q, dqdt, z] = HybridDynamics(q0, dqdt0, z0, p, simOptions);

% Display results
figure('Name','SLIP model: y and dy of in-place hopping','Units','Normalized','OuterPosition',[0.5,0.0,0.5,1.0]);
grid on; hold on; box on;
plot(t, [q(2,:).',dqdt(2,:).']);
legend({'$y$','$\dot{y}$'}, 'interpreter','latex');
drawnow();

% Let's add some forward motion to the initial state:
dx = 1;
dqdt0 = [dx, dy]';
[t, q, dqdt, z] = HybridDynamics(q0, dqdt0, z0, p, simOptions);

% ************************************
% ************************************
% TASK 5-2: GET A ROUGHLY PERIODIC MOTION WITH dx=1
% ************************************
% ************************************

% Since this is obviously not periodic, we need to change the angle of
% attack:
angAtt = 15*pi/180;
p(5) = angAtt;
[t, q, dqdt, z] = HybridDynamics(q0, dqdt0, z0, p, simOptions);

% We will solve for a periodic solution numerically below, but first
% try to manually change angAtt and see how close you can get to a
% periodic motion.

%% (b) Create a periodic gait
% (i) Find the right forward speed
% For the SLIP model, the problem of gait creation can be defined in
% two ways.  One is to find periodic initial conditions that match the
% given angle-of-attack. That is, we look for states in which the model
% is started and that are reached again at the end of one stride.  It
% turns out that it is sufficent to only alter the forward velocity. 

% ************************************
% ************************************
% TASK 6-1: EXECUTE THE CODE IN THIS SECTION AND CHECK IF IT CREATES A
% PERIODIC TRAJECTORY
% ************************************
% ************************************

% Use the initial velocity from before as initial guess for a periodic
% trajectory:
dxINIT = dx;
% Fix the angle-of-attack:
angAtt = pi/8;
p(5) = angAtt;
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
dxCYC = fsolve(@(dx)residual(y,dx,zCYC,pCYC,angAtt), dxINIT, numOPTS);
qCYC    = [0; y];
dqdtCYC = [dxCYC; 0];

% Run a one-step simulation to see that the solution is indeed periodic, 
% and initial and final states are identical: 
simOptions.simulateSingleStride = true; % Simulate only a single stride
[t, q, dqdt, z] = HybridDynamics(qCYC, dqdtCYC, zCYC, pCYC, simOptions);
% Display state deviation after one step:
disp([newline, 'Deviation after one step: ']);
disp(num2str([q(:,end)-qCYC; dqdt(:,end)-dqdtCYC]));
% Note that the first number is non-zero. It corresponds to the change in
% the CoM horizontal position after one step which does not have to be
% periodic.

%% (ii) Find the right angle of attack
% In the second approach, we define a forward speed and search for the
% correct angle-of-attack that corresponds to periodic motion.

% ************************************
% ************************************
% TASK 6-2: COMPLETE THE CODE IN THIS SECTION TO FIND THE CORRECT
% ANGLE-OF-ATTACK FOR A FIXED VELOCITY dx
% ************************************
% ************************************

% Fix the speed:
dx = 2.5;
% Make initial guess of the periodic angle-of-attack:
angAttINIT = pi/8;

% Solve the non-linear equation defined in 'residual()', this time with
% respect to the angle-of-attack:
angAttCYC = fsolve( @(angAtt)residual(y,dx,zCYC,pCYC,angAtt), angAttINIT, numOPTS);
disp(['The required angle-of-attack of the periodic gait is: ',num2str(angAttCYC)]);

% Run a one-step simulation to see that the solution is indeed periodic, 
% and initial and final states are identical: 
qCYC    = [0; y];
dqdtCYC = [dx; 0];
zCYC    = [0; 0];
pCYC = p;
pCYC(5) = angAttCYC;
simOptions.simulateSingleStride = true; % Simulate only a single stride
[t, q, dqdt, z] = HybridDynamics(qCYC, dqdtCYC, zCYC, pCYC, simOptions);
% Display state deviation after one step:
disp([newline, 'Deviation after one step: ']);
disp(num2str([q(:,end)-qCYC; dqdt(:,end)-dqdtCYC]));


%% (c) Stability analysis

% ************************************
% ************************************
% TASK 7-1: COMPLETE THE NESTED FUNCTION floquet AND RUN THIS SECTION
% TO CHECK STABILITY OF THE SOLUTION
% ************************************
% ************************************

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
elseif any(abs(eigenValuesCYC)>1)
    disp('Solution is unstable')
else
    disp('new periodic gate')
    % What is the situation we haven't covered? And what does it mean?
end
disp('...found the following Eigenvectors:');
disp(eigenVectorsCYC);
disp('What do these eigenvector/eigenvalue pairs mean, physically?')
% Post to the forum your interpretation of the eigenvector/eigenvalue
% pair. Hint: try comparing several periodic orbits, both stable and
% unstable. Are there any eigenvalues that don't change? What do these
% mean?

%% (d) Variation on constant energy level:
% We now vary the forward velocity dx and the hopping height y, such that
% the total energy of the system does not change.

% ************************************
% ************************************
% TASK 9-1: CREATE A GAIT WITH dx=2.5 USING THE CODE IN SECTION (c-ii);
% TASK 9-2: COMPLETE THE CODE BELOW TO CREATE A SET OF INITIAL
% CONDITIONS WITH THE SAME ENERGY LEVEL AND SIMULATE ONE STEP FOR EACH
% OF THEM; 
% TASK 9-3: PLOT A FIRST ORDER RETURN MAPS
% ************************************
% ************************************

% Extract individual states and parameters:
[x,y,dx,dy] = contStates(qCYC, dqdtCYC);
[g,l_0,m_0,k,~] = systParam(pCYC);
% Compute the nominal amount of energy of the corresponding gait:
E_tot = m_0*g*y + 1/2*m_0*(dx^2 + dy^2); % CHANGEME;

% Number of sample points (of different disturbances):
n = 50;
% Variations in forward velocity:
dx_in = linspace(2.3, 2.6, n);
% Energy-invariant change in hopping height:
y_in  = (E_tot - 0.5*dx_in.^2)/g; % CHANGEME;

% Supress the animations for now:
simOptions.graphOUTPUT = [];

% Initialize the vectors of deviations after one step:
dx_out = zeros(1,n);
y_out = zeros(1,n);
% For each considered disturbance, run a one-step simulation and save the
% velocity and height deviations at the end of the step:
for i = 1:n
    disp(['Iteration: ',num2str(i)])
    qIN = qCYC;
    qIN(2) = y_in(i);
    dqdtIN = dqdtCYC;
    dqdtIN(1) = dx_in(i);
    [t, q, dqdt, z] = HybridDynamics(qIN, dqdtIN, zCYC, pCYC, simOptions);
    y_out(i)  = q(2,end);
    dx_out(i) = dqdt(1,end);
end

% Plot the first order return map of the hopper as a function of dx:
figure('Name','SLIP model: 1st order return map for speed','Units','Normalized','OuterPosition',[0.0,0.5,0.5,0.5]);
hold on; grid on; box on
plot(dx_in, dx_out)
% All periodic solutions are on this line:
line(dx_in, dx_in, 'color', 'k')
axis equal
axis tight
xlabel('dx before stride')
ylabel('dx after stride')

% The same figure for hopping height y; the two figures are equivalent:
figure('Name','SLIP model: 1st order return map for height','Units','Normalized','OuterPosition',[0.0,0.0,0.5,0.5]);
hold on; grid on; box on
plot(y_in, y_out)
% All periodic solutions are on this line:
line(y_in, y_in, 'color', 'k')
axis equal
axis tight
xlabel('y before stride')
ylabel('y after stride')

% ************************************
% ************************************
% TASK 10: EVALUATE THE CODE BELOW WITH TWO DIFFERENT INITIAL
% VELOCITIES dx 
% ************************************
% ************************************

%% (e) Compare a couple of steps that are started closely to the second 
%% (stable) fixed point for a starting point inside and outside the 
%% basin of attraction:  
% (i) Choose initial conditions that start inside the basin of
% attraction. Read the return map to find a velocity that should be
% inside the basin. Make sure to choose a height that corresponds to
% the same energy level!
qIN = [qCYC(1), CHANGEME].';
dqdtIN = [CHANGEME, dqdtCYC(2)].';
zIN = zCYC;
simOptions.tMAX = 100;      % Simulation duration
simOptions.simulateSingleStride = false;     % Do a continuous simulation
[t, q, dqdt, z] = HybridDynamics(qIN, dqdtIN, zIN, pCYC, simOptions);
figure('Name','SLIP model: y and dy of a motion that converges to stable solution','Units','Normalized','OuterPosition',[0.5,0.5,0.5,0.5]);
grid on; hold on; box on;
plot(t, [q(2,:).',dqdt(2,:).']);
legend({'$y$','$\dot{y}$'}, 'interpreter','latex');
% (ii) Now choose one that is just outside the basin of attraction.
qIN = [qCYC(1), CHANGEME].';
dqdtIN = [CHANGEME, dqdtCYC(2)].';
zIN = zCYC;
simOptions.tMAX = 55;      % Simulation duration
simOptions.simulateSingleStride = false;     % Do a continuous simulation
[t, q, dqdt, z] = HybridDynamics(qIN, dqdtIN, zIN, pCYC, simOptions);
figure('Name','SLIP model: y and dy of a motion that fails','Units','Normalized','OuterPosition',[0.5,0.5,0.5,0.0]);
grid on; hold on; box on;
plot(t, [q(2,:).',dqdt(2,:).']);
legend({'$y$','$\dot{y}$'}, 'interpreter','latex');


%% (f) Perform a continuation study
% (i) Periodic solutions for larger hopping heights.

% ************************************
% ************************************
% TASK 11-1: RUN THE CODE IN THIS SECTION, OBSERVE A SET OF SOLUTIONS
% AUTOMATICALLY FOUND
% ************************************
% ************************************

% Consider the same periodic solution we just found and analyzed. We
% now want to find all possible solutions with the same
% angle-of-attack. To do this, we are going to perform a continuation
% study. That is, given one known solution, we will use it as the
% initial guess to find a different solution nearby.

% We start with the solution we already know:
qCYC_start = qCYC;
dqdtCYC_start = dqdtCYC;
angAtt = pCYC(5);

% Define a vector of apex heights for which we are looking for periodic motions:
d = 0.05;   % Small step in height
solution.y = qCYC_start(2):d:5;
% Initialize the corresponding vector of forward speeds (these are to be
% determined):
solution.dx = nan(size(solution.y));

% Use our starting point as the initial guess for finding the first
% solution in the continuation procedure:
dxINIT = dqdtCYC_start(1);

% Prepare to display results:
figure('Name','SLIP model: dx and y of periodic solutions');
xlabel('Forward velocity, dx');
ylabel('Height at apex, y');
grid on; hold on; box on;
solution.graph = line(solution.dx, solution.y, 'marker','.', 'markersize',15, 'color','blue');

% In the loop below, we go through all specified apex heights and find a 
% corresponding periodic solution for each of them. At each iteration, we
% use the last found solution as the initial guess for finding the next
% solution nearby.
for i = 1:length(solution.y)
    % The hopping height for which we want to find the next solution:
    y = solution.y(i);
    % Call the root-search function:
    dxCYC = fsolve(@(dx)residual(y,dx,zCYC,pCYC,angAtt), dxINIT, numOPTS);

    % Store and display the result:
    solution.dx(i) = dxCYC;
    set(solution.graph, 'xdata',solution.dx, 'ydata',solution.y);
    drawnow;
    % Initialize the search for the next iteration:
    dxINIT = dxCYC;
end


%% (ii) Periodic solutions for smaller hopping heights.
% We now want to use the same continuation routine to find periodic
% solutions with smaller hopping heights.

% ************************************
% ************************************
% TASK 11-2: COMPLETE THE CODE BELOW TO FIND A SET OF SOLUTIONS FOR
% DECREASING HOPPING HEIGHTS
% ************************************
% ************************************

% Vector of heights for which we are looking for periodic motions:
solution.y = CHANGEME;
% Initialize the corresponding vector of forward speeds (these are to
% be determined):
solution.dx = CHANGEME;

% Again, we use the same starting point as the initial guess for
% finding the first solution in the continuation procedure:
dxINIT = dqdtCYC_start(1);

% initialize the graphics for the new search:
solution.graph = line(solution.dx, solution.y, 'marker','.', 'markersize',15, 'color','red');

% In the loop below, we go through all specified apex heights and try to 
% find a corresponding periodic solution for each of them. At each 
% iteration, we use the last found solution as the initial guess for 
% finding the next solution nearby.
for i = 1:length(solution.y)
    % The hopping height for which we want to find the next solution:
    y = solution.y(i);
    % Call the root-search function:
    dxCYC = fsolve(@(dx)residual(y,dx,zCYC,pCYC,angAtt), dxINIT, numOPTS);

    % Store and display the result:
    solution.dx(i) = dxCYC;
    set(solution.graph, 'xdata',solution.dx, 'ydata',solution.y);
    drawnow;
    % Initialize the search for the next iteration:
    dxINIT = dxCYC;
end

% However, we can notice that the root-finder is not able to find periodic
% solutions after a few iterations. This is because we used a fixed value
% by which we decreased the desired apex height at each iteration -- but
% solutions do not exist for too small heights (or have very large speeds
% and are thus far from the initial guess). We resolve this issue below.


%% (iii) Continuation with linear extrapolation of solutions

% We want to set up the continuation routine so as to move at each
% iteration not by a small fixed height (or speed) -- but along the branch
% of solutions. To do this, we have to compute the initial guess at each
% iteration by extrapolation from prevously found solutions.

% ************************************
% ************************************
% TASK 12: COMPLETE THE CODE BELOW TO IMPLEMENT CONTINUATION AND
% DISCOVER THE SOLUTION BRANCH
% ************************************
% ************************************

% Number of solutions to search (# of iterations in the continuation):
N = 200;
% Initialize the solution vectors for heights and speeds:
solution.y = nan(1,N);
solution.dx = nan(1,N);
% Here, we start from a large-height solution point (this is the solution
% at which we stopped the continuation in (g-i)):
solution.y(1) = 4.9514; 
solution.dx(1) = 2.3942;
angAtt = 0.5861;
pCYC(5) = angAtt;
% and we want to move in the direction of decreasing heights.

% Prepare to display results:
figure('Name','SLIP model: dx and y of periodic solutions');
xlabel('Forward velocity, dx');
ylabel('Height at apex, y');
grid on; hold on; box on;
solution.graph = line(solution.dx, solution.y, 'marker','.', 'markersize',15, 'color','blue');


% First, find one nearby solution. This is needed only once to
% initialize the continuation procedure.
yINIT  = solution.y(1) - 0.05;   % A small deviation in the apex height
dxINIT = solution.dx(1);
% Call the root-search function:
solutionCYC = fsolve(CHANGEME, CHANGEME, numOPTS);

% Store and display the result:
solution.y(2)  = solutionCYC(1);
solution.dx(2) = solutionCYC(2);
set(solution.graph, 'xdata',solution.dx, 'ydata',solution.y);
drawnow;

% In the loop below, we move along the solution branch to find the
% specified total number of solutions. At each iteration, we use the 
% previous two solutions to generate an initial guess for finding the next 
% solution along the branch.
for i = 3:N
    % Initial guess for the next solution:
    yINIT  = CHANGEME;
    dxINIT = CHANGEME;
    % Call the root-search function:
    solutionCYC = fsolve(CHANGEME,  CHANGEME, numOPTS);

    % Note: The root-search above does not guarantee that the new solution
    % is a fixed desired distance d away from the previous solution (as in
    % the algorithm we discussed in class). Instead, the initial guess for
    % the root-search is generated that maintains the distance between the
    % last two solutions. Hence, the resulting new solution is usually (but
    % not necessarily) at approximately the same distance from the previous
    % solution.

    % Store and display the result:
    solution.y(i)  = solutionCYC(1);
    solution.dx(i) = solutionCYC(2);
    set(solution.graph, 'xdata',solution.dx, 'ydata',solution.y);
    drawnow;
end


%% (iv) Check stability along the branch of solutions

% ************************************
% ************************************
% TASK 13: EXECUTE THIS SECTION TO CHECK STABILITY OF THE SOLUTIONS
% ALONG THE BRANCH
% ************************************
% ************************************

% The stability of the linearized system is evaluated by computing the
% Floquet multipliers (eigenvalues of the Monodromy matrix)
for i = 1:N
    qCYC    = [0; solution.y(i)];
    dqdtCYC = [solution.dx(i); 0];
    % The eigenvalues and eigenvectors are computed numerically:
    eigenValuesCYC = floquet(qCYC, dqdtCYC, zCYC, pCYC);

    % Two of the eigenvalues are always 0 and 1. To assess stability, we
    % need to identify the other, non-unit and non-zero eigenvalue.
    % This eliminates the zero eigenvalue:
    [~,ind] = min(abs(eigenValuesCYC));
    eigenValuesCYC = eigenValuesCYC([1:ind-1,ind+1:end]);
    % This selects the non-unit of the two remaining eigenvalues:
    [~,ind] = max(abs(eigenValuesCYC-1));
    solution.eigenValue(i) = eigenValuesCYC(ind);
end

% plot the individual eigenvalues:
figure('Name','SLIP model: dx and y of periodic solutions','WindowStyle','docked');
grid on; hold on; box on;
xlabel('Forward velocity, dx');
ylabel('Height at apex, y');
% Define the color map of the figure:
caxis([min(solution.eigenValue), max(solution.eigenValue)])
map = jet(N);
for i = 1:N
    % We want to color the i-th solution with the color corresponding to
    % its eigenvalue. We identify the index of this color in the colormap,
    % by linearly mapping the vector of all eigenvalues to the interval
    % [1,N]. Then, the color index is:
    iMap = 1 + round( (N-1) * (solution.eigenValue(i)-min(solution.eigenValue)) / (max(solution.eigenValue)-min(solution.eigenValue)) );
    if abs(solution.eigenValue(i)) < 1  % stable solutions
        plot(solution.dx(i), solution.y(i), 'Color',map(iMap,:), 'Marker','o','MarkerSize',10,'LineStyle','none');
    else    % unstable solutions
        plot(solution.dx(i), solution.y(i), 'Color',map(iMap,:), 'Marker','x','MarkerSize',6,'LineStyle','none');
    end
end
% Graph annotation:
colormap(map)
colorbar();

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

function r_ = residual(yIN_, dxIN_, zIN_, p_, angAtt_)
    % Form the initial state vectors:
    qIN_    = [0; yIN_];
    dqdtIN_ = [dxIN_; 0];
    % Update the angle-of-attack in the parameter vector:
    p_(5) = angAtt_;

    % Define simulation options:
    simOptions_.tSTART = 0;                            % initial time
    simOptions_.tMAX = 5;                              % maximum simulation time
    simOptions_.graphOUTPUT = [];                      % no graphical output
    simOptions_.controller = @Controller_PassiveSLIP;  % handle to the controller function
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


% BONUS question: this implementation of floquet analysis relies on
% numerical finite differences, which is notoriously brittle.
% How could the Floquet analysis be made more reliable?
% Post your ideas in the forum, extra points for implementing them.
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

% ************************************
% ************************************
% TASK 7-1: COMPLETE THE CODE BELOW TO COMPUTE THE MONODROMY MATRIX
% ************************************
% ************************************

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
        J_(:,j_) = [qPLUS_(2,end)-qMINUS_(2,end);  dqdtPLUS_(2,end) - dqdtMINUS_(2, end); zPLUS_(2,end) - zMINUS_(2,end)] ./ (2*disturbance_);
    end
    % Conpute the eigenvalues and eigenvectors of the Monodromy matrix:
    [eigenVectorsCYC_, D_] = eig(J_);
    eigenValuesCYC_  = diag(D_);
end