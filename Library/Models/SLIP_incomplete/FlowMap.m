% *************************************************************************
%
% function ddqddt = FlowMap(t, q, dqdt, z, p, ControllerFcn)
% 
% This code is for a floating based model of the SLIP model
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
%   See also JUMPMAP, JUMPSET

function ddqddt = FlowMap(t, q, dqdt, z, p, ControllerFcn)
    % Extract continuous states:
    [x,y,dxdt,dydt] = contStates(q, dqdt);
    % Extract discrete states:
    [phase,contPt] = discStates(z);
    % Extract parameters:
    [g,l_0,m_0,k,angAtt] = systParam(p);
    
    % ************************************
    % ************************************
    % TASK 2: IMPLEMENT YOUR CONTINUOUS DYNAMICS HERE
    % ************************************
    % ************************************

    % Compute COM-accelerations according to the phase:
    switch phase
        case {0,2} %(flight)
            ddxddt =  0;
            ddyddt = -g;
        case 1 %(stance)
            % compute leg length and  leg angle
            l_leg = sqrt((x-contPt)^2 + y^2);
            gamma = atan2(x-contPt, y);
            % compute spring forces
            f_spring = k * (l_0-l_leg);
            % CoM accelerations
            ddxddt = f_spring * sin(gamma) / m_0;
            ddyddt = f_spring * cos(gamma) / m_0;
    end
    ddqddt = [ddxddt, ddyddt].';
end
% *************************************************************************
% *************************************************************************
    