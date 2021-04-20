% *************************************************************************
%
% evntVal = JumpSet(t, q, dqdt, z, p, ControllerFcn)
%
% This code is for a floating based model of the Simplest Walking Model
%
% The function defines the occurrence of discrete events. The model's
% current continuous and discrete states together with the model parameters
% and a controller function are provided by the calling routine to which a
% vector of event function values is returned. The directional
% zero-crossings of these functions each trigger a different event. 
%
% INPUT:    t    -- current time instant;
%           q    -- vector of positions (generalized coordinates);
%           dqdt -- vector of velocities;
%           z    -- vector of discrete states;
%           p    -- vector of model parameters;
%           ControllerFcn -- handle to the system controller function;
% OOUTPUT:  evntVal -- vector of values of the event functions; each 
%                      element corresponds to a function, of which a 
%                      zero-crossing (with positive derivative) is detected
%                      as an event.
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
%   See also FLOWMAP, JUMPMAP, SYMBOLICCOMPUTATIONOFEOM

function evntVal = JumpSet(t, q, dqdt, z, p, ControllerFcn)
    % Extract continuous states:
    [theta,phi,dtheta,dphi] = contStates(q, dqdt);
    
    % Event 1: touch-down
    n_events = 1;
    evntVal = zeros(n_events,1);
    
    % *******
    % Event 1: Detect touch-down 
    if phi < -0.05
        % Event is detected if swing foot goes below the ground
        evntVal(1) = -(cos(theta) - cos(theta-phi));
    else
        % But only when it is sufficiently ahead, so as to avoid scuffing
        evntVal(1) = -1;
    end
end
% *************************************************************************
% *************************************************************************