% *************************************************************************
%
% varargout = contStates(q, dqdt)
% 
% This code is for a minimal-ccordinate model of a 5-link walker
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

function [q1, q2, q3, q4, q5, ...
    dq1, dq2, dq3, dq4, dq5] = contStates(q, dqdt)

    % Positions:
    q1  = q(1);  % [rad] angle of the main body with the vertical (0 = orthog to the ground, >0 is ccw)
    q2  = q(2);  % [rad] angle of the stance shank wrt the stance thigh (0 = straight knee, >0 is ccw)
    q3  = q(3);  % [rad] angle of the stance thigh wrt the main body (0 = parallel to mb, >0 is ccw)
    q4  = q(4);  % [rad] angle of the swing thigh wrt the main body (0 = parallel to mb, >0 is ccw)
    q5  = q(5);  % [rad] angle of the swing shank wrt the swing thigh (0 = straight knee, >0 is ccw)
    % Velocities:
    dq1 = dqdt(1);  % [rad/sqrt(l_0/g)] ... angular velocity thereof
    dq2 = dqdt(2);    % [rad/sqrt(l_0/g)] ... angular velocity thereof
    dq3 = dqdt(3);  % [rad/sqrt(l_0/g)] ... angular velocity thereof
    dq4 = dqdt(4);    % [rad/sqrt(l_0/g)] ... angular velocity thereof
    dq5 = dqdt(5);    % [rad/sqrt(l_0/g)] ... angular velocity thereof
end
% *************************************************************************
% *************************************************************************