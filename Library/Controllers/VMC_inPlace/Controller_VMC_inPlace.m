% *************************************************************************
%
%   tau = Controller_VMC_inPlace(t, q, dqdt, z, p)
%
% This is a virtual model controller for a 5-link biped in 2D that aims to
% make the CoM and main body follow a desired trajectory while standing in
% place.
%
% This controller works with the following models:
%    - 5LinkBiped_floatBase
%
% Note that the syntax of the controller function has to be the same for
% any controller used with the simulation framework. 
%
%   u = Controller(t, q, dqdt, z, p)
%
% INPUT:    t    -- time point;
%           q    -- vector of generalized positions;
%           dqdt -- vector of generalized rates;
%           z    -- vector of discrete states;
%           p    -- vector of system parameters.
% OUTPUT:   u    -- controller outputs. If time-stepping integration is
%                   used, these have to be joint torques; if event-based
%                   integration is used, the output u is passed into the
%                   dynamics functions of the system (FlowMap, JumpMap,
%                   JumpSet).
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
%   See also TIMESTEPPING, HYBRIDDYNAMICS,  FLOWMAP, JUMPMAP, JUMPSET

function tau = Controller_VMC_inPlace(t, q, dqdt, z, p)
    % Extract continuous states:
    [x,y,phi,alphaL,alphaR,betaL,betaR,...
        dx,dy,dphi,dalphaL,dalphaR,dbetaL,dbetaR] = contStates(q, dqdt);
    
    %% Define motion targets:
    x_des   = 0;   % desired horizontal position of the hip
    y_des   = 0.7; % desired hip height
    phi_des = 0;   % desired body pitch
    
    %% Compute desired virtual forces (corresponding to the set motion targets):
    % horizontal force at the hip:
    P=10; 
    D=2;
    Fx = P*(x_des - x) + D*(0 - dx);            
    % vertical force at the hip:
    P=10; 
    D=2;
    Fy = P*(y_des - y) + D*(0 - dy);     
    % torque at the torso
    P=5; 
    D=1;
    Fphi = P*(phi_des - phi) + D*(0 - dphi);    
    
    %% Compute corresponding joint torques:
    % Evaluate system kinematics and dynamics:
    F_cg = F_CoriGrav(q,dqdt,p);
    % left-leg and right-leg contact point Jacobians:
    JL = ContactJacobianL(q,p);
    JR = ContactJacobianR(q,p);
    
    %% Depending on the contact configuration, we can apply different 
    %% numbers of forces: 
    if z(1) && ~z(2)
        % LEFT SINGLE STANCE (control only height and pitch angle)
        % Extract partial Jacobians for the passive and active degrees of
        % freedom (ignoring x direction): 
        JP = JL(:,2:3);
        JA = JL(:,4:7);
        % Compute desired contact forces (ignoring x direction):
        lamda_des = JP' \ ([Fy; Fphi] - F_cg(2:3));
        % Compute corresponding motor torques:
        tau_mot = -JA'*lamda_des - F_cg(4:7);
        % Overwrite controls for swing leg controller:
        tau_mot([2,4]) = 0;
    elseif ~z(1) && z(2)
        % RIGHT SINGLE STANCE (control only height and pitch angle)
        % Extract partial Jacobians for the passive and active degrees of
        % freedom (ignoring x direction): 
        JP = JR(:,2:3);
        JA = JR(:,4:7);
        % Compute desired contact forces (ignoring x direction):
        lamda_des = JP' \ ([Fy; Fphi] - F_cg(2:3));
        % Compute corresponding motor torques:
        tau_mot = -JA'*lamda_des - F_cg(4:7);
        % Overwrite controls for swing leg controller:
        tau_mot([1,3]) = 0;
    elseif z(1) && z(2)
        % DOUBLE STANCE (control all torso DOFs)
        % Extract partial Jacobians for the passive and active degrees of
        % freedom: 
        JP = [JL(:,1:3); JR(:,1:3)];
        JA = [JL(:,4:7); JR(:,4:7)];
        % Compute desired contact forces (using a least squares solution to
        % find minimal lambdas): 
        lamda_des = pinv(JP') * ([Fx; Fy; Fphi] - F_cg(1:3));
        % Compute corresponding motor torques:
        tau_mot = -JA'*lamda_des - F_cg(4:7);
    else
        % FLIGHT (set all output torques to 0)
        tau_mot = zeros(4,1);
    end
    
    % Assign joint torques to the extended torque vector:
    tau = zeros(7,1);
    tau(4:7) = tau_mot;
end