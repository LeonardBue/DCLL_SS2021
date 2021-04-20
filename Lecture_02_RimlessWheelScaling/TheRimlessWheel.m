% *************************************************************************
%
%   [] = TheRimlessWheel()
%
% Simulation of a rimless wheel
%
% INPUT:    - None
% OUTPUT:   - None
%
%   C. David Remy remy@inm.uni-stuttgart.de
%   Matlab R2018
%   4/22/2020
%   v21

function TheRimlessWheel()
    %% Initial Setup
    % Make a clean sweep:
    clear all
    close all
    clc
    %% Set up the model:
    % Model parameters:
    m     = 1;           % total mass [units don't matter, could be kg]
    l     = 1;           % leg length [units don't matter, could be m]
    g     = 1;           % gravity [units don't matter, could be m/s^2]
    r_g   = 0.1;         % normalized radius of gyration [units of l]
    alpha = 22.5*pi/180; % leg angle [rad] 
    gamma = 0.1;         % slope of the ramp [rad]

    % Simulation parameters
    nSteps        = 20;     % number of steps to simulate
    theta_0_PLUS  = -alpha; % initial leg angle [rad]
    thetaD_0_PLUS = +0.5;    % initial angular velocity [rad/s]

    %% Simulation
    % Set up output, solver, initial conditions:
    t      = [];
    theta  = [];
    thetaD = [];
    options = odeset('Events',@myEventsFcn, 'AbsTol', 1e-8, 'RelTol', 1e-8);
    y_i_PLUS = [theta_0_PLUS; thetaD_0_PLUS];
    t_PLUS = 0;
    % Simulate nSteps steps:
    for i = 1:nSteps
        % 1) Integrate the rimless wheel dynamics until an event is triggered
        [t_part, y_part, te, ye, ie] = ode45(@ode, [t_PLUS,t_PLUS+20], y_i_PLUS, options);
        if (isempty(ie))
            warning('No event triggered after 20 units of time')
        end
        % Store results of integration
        t = [t;t_part];
        theta  = [theta; y_part(:,1)];
        thetaD = [thetaD;y_part(:,2)];
        % 2) Compute transfer of support
        theta_i_MINUS  = y_part(end,1);
        thetaD_i_MINUS = y_part(end,2);
        theta_i_PLUS  = -theta_i_MINUS;
        thetaD_i_PLUS = (cos(2*alpha)+r_g^2)/(1+r_g^2)*thetaD_i_MINUS;
        % Store results
        t = [t;t_part(end)];
        theta  = [theta; theta_i_PLUS];
        thetaD = [thetaD;thetaD_i_PLUS];
        % 3) Repeat with new initial conditions
        y_i_PLUS = [theta_i_PLUS; thetaD_i_PLUS];
        t_PLUS = t_part(end);
    end
    % Plot results
    figure('Name','Motion of the Rimless wheel','Units','Normalized','OuterPosition',[0,0,0.5,1])
    grid on; hold on; box on;
    plot(t,theta)
    plot(t,thetaD)
    ylabel('leg angle, $\theta$, angular rate, $\dot{\theta}$', 'interpreter','latex');
    xlabel('time [normalized]');
    
    %% Compute step-to-step transitions for different intial speeds
    % This does the same as version _v01, just that it is using numerical
    % integratio instead of an energy-based transfer.
    % Set up array of initial speeds:
    n = 200;
    thetaD_min = -1;
    thetaD_max = +1;
    thetaD_START = linspace(thetaD_min,thetaD_max,n);
    thetaD_END   = zeros(1,n);
    options = odeset('Events',@myEventsFcn, 'AbsTol', 1e-8, 'RelTol', 1e-8);
    for i = 1:n
        if (thetaD_START(i) >0)
            theta_i_PLUS  = -alpha; % initial leg angle [rad]
        else
            theta_i_PLUS  = +alpha; % initial leg angle [rad]
        end
        thetaD_i_PLUS = +thetaD_START(i); % initial angular velocity [rad/s]
        % Simulate
        y_i_PLUS = [theta_i_PLUS; thetaD_i_PLUS];
        % 1) Integrate the rimless wheel dynamics until an event is triggered
        [~, y, ~, ~, ie] = ode45(@ode, [0,20], y_i_PLUS, options);
        if (isempty(ie))
            warning('No event triggered after 20 units of time')
        end
        % 2) Compute transfer of support
        % (Note: we are only intersted in velocities)
        thetaD_i1_MINUS = y(end,2);
        thetaD_i1_PLUS = (cos(2*alpha)+r_g^2)/(1+r_g^2)*thetaD_i1_MINUS;
        % Store result:
        thetaD_END(i) = thetaD_i1_PLUS;
    end
    % Plot results
    figure('Name','Step-to-step transfer of the rimless wheel','Units','Normalized','OuterPosition',[0.5,0,0.5,1])
    grid on; hold on; box on;
    plot(thetaD_START, thetaD_END,'r.');
    plot([thetaD_min,thetaD_max],[thetaD_min,thetaD_max],'k');
    xlabel('initial speed, $\dot{\theta}^+_i$', 'interpreter','latex'); ylabel('next-step speed, $\dot{\theta}^+_{i+1}$', 'interpreter','latex');
    axis equal;
    
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
%     Helper functions                                                    %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    

    % Right hand side of ODE
    function dydt_ = ode(t_,y_)
        % extract states
        theta_  = y_(1);
        thetaD_ = y_(2);
        % compute acceleration
        thetaDD_ = g/l*sin(theta_+ gamma)/(1 + r_g^2);
        % combine into states
        dydt_ = [thetaD_; thetaDD_];
    end

    % Event function
    function [value, isterminal, direction] = myEventsFcn(t_,y_)
        % extract state
        theta_     = y_(1);
        value      = [theta_ - alpha; 
                      theta_ + alpha];
        isterminal = [1; 
                      1];
        direction  = [+1;
                      -1];
    end
end