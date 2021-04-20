% *************************************************************************
%
% ddqddt = FlowMap(t, q, dqdt, z, p, ControllerFcn)
% 
% This code is for a floating based model of the Simplest Walking Model
%
% The function defines the continuous dynamics. The model's current
% continuous and discrete states, as well as the model parameters and a
% controller function are given by the calling routine, and the generalized 
% accelerations are returned. 
%
% INPUT:    t    -- current time instant;
%           q    -- a vector of positions (generalized coordinates);
%           dqdt -- a vector of velocities;
%           z    -- a vector of discrete states;
%           p    -- a vector of model parameters;
%           ControllerFcn -- a handle to the system controller function;
% OUTPUT:   ddqddq -- accelerations of the generalized coordinates q.
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
%   See also JUMPMAP, JUMPSET, SYMBOLICCOMPUTATIONOFEOM

function ddqddt = FlowMap(~, q, dqdt, ~, p, ~)
    % Extract continuous states:
    [theta,phi,dtheta,dphi] = contStates(q, dqdt);
    % Extract parameters:
    gamma = systParam(p);
       
    % Dynamics (the mass-matrix and the right-hand side expression):
    M = [1,  0; ...
         1, -1];
    f_cg = [sin(theta-gamma); ...
            -dtheta^2*sin(phi) + cos(theta-gamma)*sin(phi)];
    % EQM:
    ddqddt = M\(f_cg);
end
% *************************************************************************
% *************************************************************************
    