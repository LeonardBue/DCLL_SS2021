% *************************************************************************
%
% varargout = contStates(q, dqdt)
% 
% This code is for a floating based model of the Simplest Walking Model
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

function [theta, phi, dtheta, dphi] = contStates(q, dqdt)

    % Positions:
    theta  =  q(1);  % [rad] stance leg angle
    phi    =  q(2);  % [rad] angle between the legs
    % Velocities:
    dtheta =  dqdt(1);  % [rad/sqrt(l/g)] ... angular velocity thereof
    dphi   =  dqdt(2);  % [rad/sqrt(l/g)] ... angular velocity thereof
end
% *************************************************************************
% *************************************************************************