% *************************************************************************
%
%   tau = Controller_Freeze(t, q, dqdt, z, p)
%
% This is a trivial controller, which seeks to set all joint angles to a
% fixed desired position.
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

function tau = Controller_Freeze(t, q, dqdt, z, p)
    % Extract continuous states:
    [x,y,phi,alphaL,alphaR,betaL,betaR,...
        dx,dy,dphi,dalphaL,dalphaR,dbetaL,dbetaR] = contStates(q, dqdt);

    %% Run a simple PD controller for each joint:

    % Initialize all torques to 0:
    tau = 0*q;

    % Desired joint angles:
    alphaL_des = 0.25;
    alphaR_des = 1.5;
    % alphaR_des = 0.5;
    betaL_des  = -0.5;
    betaR_des  = -0.5;

    % Controller gains:
    P = 10;
    D = 5;

    % PD controls for the joints:
    tau(4) = P*(alphaL_des - alphaL) + D*(0 - dalphaL);
    tau(5) = P*(alphaR_des - alphaR) + D*(0 - dalphaR);
    tau(6) = P*(betaL_des - betaL) + D*(0 - dbetaL);
    tau(7) = P*(betaR_des - betaR) + D*(0 - dbetaR);
end