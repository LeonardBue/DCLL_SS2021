% *************************************************************************
%
% varargout = contStates(q, dqdt)
% 
% This code is for a floating based model of a 7-link walker
%
% This function extracts from the vectors of generalized coordinates and 
% velocities the values of individual coordinates and velocities, and 
% returns them as separate variables. Thus, this function defines the order
% of variables within the state vector that should be kept the same 
% throughout the model simulation and analysis.
%
% INPUT:    q    -- vector of the generalized coordinates;
%           dqdt -- vector fo the generalized velocities;
% OUTPUT:   the set of the system states.
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
%   See also FLOWMAP, JUMPMAP, JUMPSET, SYMBOLICCOMPUTATIONOFEOM

function [x, y, phi, alphaL, alphaR, betaL, betaR, gammaL, gammaR, ...
    dx, dy, dphi, dalphaL, dalphaR, dbetaL, dbetaR, dgammaL, dgammaR] = contStates(q, dqdt)

    % Positions:
    x         = q(1);  % [l] vertical main body position
    y         = q(2);  % [l] horizontal main body position
    phi       = q(3);  % [rad] angle of the main body with the vertical (0 = orthog to the ground, >0 is ccw)
    alphaL    = q(4);  % [rad] angle of the left thigh wrt the main body (0 = parallel to mb, >0 is ccw)
    alphaR    = q(5);  % [rad] angle of the right thigh wrt the main body (0 = parallel to mb, >0 is ccw)
    betaL     = q(6);  % [rad] angle of the left shank wrt the left thigh (0 = straight knee, >0 is ccw)
    betaR     = q(7);  % [rad] angle of the right shank wrt the right thigh (0 = straight knee, >0 is ccw)
    gammaL    = q(8);  % [rad] gammaL, angle of the left foot wrt the left shank (0 = straight ankle, >0 is ccw)
    gammaR    = q(9);  % [rad] gammaR, angle of the right foot wrt the right shank (0 = straight ankle, >0 is ccw)
    % Velocities:
    dx        =  dqdt(1);  % [sqrt(l_0*g)] ... velocity thereof
    dy        =  dqdt(2);  % [sqrt(l_0*g)] ... velocity thereof
    dphi      =  dqdt(3);  % [rad/sqrt(l_0/g)] ... angular velocity thereof
    dalphaL   =  dqdt(4);  % [rad/sqrt(l_0/g)] ... angular velocity thereof
    dalphaR   =  dqdt(5);  % [rad/sqrt(l_0/g)] ... angular velocity thereof
    dbetaL    =  dqdt(6);  % [rad/sqrt(l_0/g)] ... angular velocity thereof
    dbetaR    =  dqdt(7);  % [rad/sqrt(l_0/g)] ... angular velocity thereof
    dgammaL   =  dqdt(8);  % [rad/sqrt(l_0/g)] ... angular velocity thereof
    dgammaR   =  dqdt(9);  % [rad/sqrt(l_0/g)] ... angular velocity thereof
end
% *************************************************************************
% *************************************************************************