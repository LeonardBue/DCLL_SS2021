% *************************************************************************
%
% [qPLUS, dqdtPLUS, zPLUS, endsStride] = JumpMap(t, qMINUS, dqdtMINUS, zMINUS, p, ControllerFcn, event)
% 
% This code is for a floating based model of the Simplest Walking Model
%
% The function defines the discrete dynamics. The model's current
% continuous and discrete states before an event (together with the model
% parameters and a controller function) are provided by the calling routine
% to which the states after the event are returned. 

% INPUT:    t             -- time of the event;
%           qMINUS        -- positions vector just before the event;
%           dqdtMINUS     -- velocities vector just before the event;
%           zMINUS        -- vector of discrete states just before the event;
%           p             -- vector of model parameters;
%           ControllerFcn -- handle to the system controller function;
%           event         -- index of the event, as defined in 'JumpSet.m';
% OUTPUT:   qPLUS      -- positions vector just after the event;
%           dqdtPLUS   -- velocities vector just after the event;
%           zPLUS      -- discrete states vector just after the event;
%           endsStride -- a Boolean flag that indicates whether the event
%                         terminates the stride or not.
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
%   See also FLOWMAP, JUMPSET, SYMBOLICCOMPUTATIONOFEOM

function [qPLUS, dqdtPLUS, zPLUS, endsStride] = JumpMap(t, qMINUS, dqdtMINUS, zMINUS, p, ControllerFcn, event)
    % Extract continuous states:
    [theta,phi,dtheta,dphi] = contStates(qMINUS, dqdtMINUS);
    
    
    % Event 1: touch-down
    switch event
        case 1 % Event 1: detected touch-down
            h = [-1, 0,                             0, 0; ...
                 -2, 0,                             0, 0; ...
                  0, 0,                  cos(2*theta), 0; ...
                  0, 0, cos(2*theta)*(1-cos(2*theta)), 0];

            % State after collision
            yPLUS = h*[qMINUS; dqdtMINUS];
            qPLUS    = yPLUS(1:2);
            dqdtPLUS = yPLUS(3:4);

            % This event marks the end of the stride
            endsStride = true;
    end
    % Assemble the discrete state vector just after the event:
    zPLUS = zMINUS;
    % Discrete states zPLUS are not present in this model, but need to be
    % output for consistency with the simulation framework 
end
% *************************************************************************
% *************************************************************************