% *************************************************************************
%
% [qPLUS, dqdtPLUS, zPLUS, endsStride] = JumpMap(t, qMINUS, dqdtMINUS, zMINUS, p, ControllerFcn, event)
% 
% This code is for a floating based model of the SLIP model
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
%   See also FLOWMAP, JUMPSET

function [qPLUS, dqdtPLUS, zPLUS, endsStride] = JumpMap(t, qMINUS, dqdtMINUS, zMINUS, p, ControllerFcn, event)
    % Extract continuous states:
    [xMINUS,yMINUS,dxdtMINUS,dydtMINUS] = contStates(qMINUS, dqdtMINUS);
    % Extract discrete states:
    [phaseMINUS,contPtMINUS] = discStates(zMINUS);
    % Extract parameters:
    [g,l_0,m_0,k,angAtt] = systParam(p);

    % Evalute the controller function:
    angAtt = ControllerFcn(t, qMINUS, dqdtMINUS, zMINUS, p);
    
    % As there is no jump in positions or velocities, we copy the states
    % before the event:
    qPLUS    = qMINUS;
    dqdtPLUS = dqdtMINUS;
    
    % ************************************
    % ************************************
    % TASK 3: IMPLEMENT YOUR JUMP MAP HERE
    % ************************************
    % ************************************
    
    % Event 1: touch-down
    % Event 2: lift-off
    % Event 3: apex transit (dy==0 during flight)
    % Event 4: fall (y==0 in any phase)
    switch event
        case 1 % Event 1: Detect touch-down
            contPtPLUS = 
            phasePLUS  = 
            % Intermediate event. Simulation continues
            endsStride = false; 
        case 2 % Event 2: Detect lift-off
            contPtPLUS = 
            phasePLUS  = 
            % Intermediate event. Simulation continues
            endsStride = false; 
        case 3 % Event 3: detect apex transit (dy==0 during flight)
            contPtPLUS =
            phasePLUS  =
            % This event marks the end of the stride
            endsStride = true;
        case 4 % Event 4: Detect fall (y==0)
            % Contact point remains unchanged
            contPtPLUS = contPtMINUS;
            % Phase remains unchanged
            phasePLUS = phaseMINUS;
            % This event stops the simulation
            endsStride = true; 
    end
    % Assemble the discrete state vector just after the event:
    zPLUS = [phasePLUS; contPtPLUS];
end
% *************************************************************************
% *************************************************************************