% *************************************************************************
%
% ddqddt = FlowMap(t, q, dqdt, z, p, ControllerFcn)
% 
% This code is for a minimal-ccordinate model of a 5-link walker
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


function ddqddt = FlowMap(t, q, dqdt, z, p, ControllerFcn)
    % Evalute the controller function
    tau = ControllerFcn(t, q, dqdt, z, p);
    % Compute the differentiable force vector (i.e. coriolis, gravity, and
    % actuator forces): 
    f_cg = F_CoriGrav(q, dqdt, p);
    % Mass matrix
    M = MassMatrix(q, p);

    % EQM:
    ddqddt = M\(f_cg + tau);
end
% *************************************************************************
% *************************************************************************
    