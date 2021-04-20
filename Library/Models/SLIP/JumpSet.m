% *************************************************************************
%
% evntVal = JumpSet(t, q, dqdt, z, p, ControllerFcn)
%
% This code is for a floating based model of the SLIP model
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
%   See also FLOWMAP, JUMPMAP

function evntVal = JumpSet(t, q, dqdt, z, p, ControllerFcn)
    % Extract continuous states:
    [x,y,dxdt,dydt] = contStates(q, dqdt);
    % Extract discrete states:
    [phase,contPt] = discStates(z);
    % Extract parameters:
    [g,l_0,m_0,k,angAtt] = systParam(p);

    % Evalute the controller function:
    angAtt = ControllerFcn(t, q, dqdt, z, p);

    % Event 1: touch-down
    % Event 2: lift-off
    % Event 3: apex transit (dy==0 during flight)
    % Event 4: fall (y==0 in any phase)
    n_events = 4;
    evntVal = zeros(n_events,1);
    
    % *******
    % Event 1: Detect touch-down
    if phase == 0 %(i.e., in flight)
        % Event is detected if foot goes below the ground during flight
        ft_height = y - l_0*cos(angAtt);
        % A event is defined by a transition from negative to positive. We
        % hence need a minus sign in the event function:
        evntVal(1) = -ft_height;
    else
        % Only detect this event during flight
        evntVal(1) = -1;
    end
    
    % *******
    % Event 2: Detect lift-off
    if phase == 1 %(i.e., in stance)
        % Event is detected if leg is fully extended. Compute the leg
        % length:
        l_leg = sqrt((x-contPt)^2 + (y-0)^2);
        % When this becomes larger than the uncompressed leg lenght, the
        % event is triggered:
        evntVal(2) = l_leg - l_0;
    else
        % Only detect this event during stance
        evntVal(2) = -1;
    end
    
    % *******
    % Event 3: Detect apex transit
    if phase == 2 %(i.e., in flight and after passing the contact point)
        % Event is detected if the vertical velocity goes from positive to
        % negative.  To detect a '-' -> '+' transition, we need a minus
        % sign in the event function: 
        evntVal(3) = -dydt;
	else
        % Only detect this event during flight after stance
        evntVal(3) = -1;
    end
    
    % *******
    % Event 4: Detect fall
        % Event is detected if the vertical position goes from positive to
        % negative.  To detect a '-' -> '+' transition, we need a minus
        % sign in the event function: 
        evntVal(4) = -y;
end
% *************************************************************************
% *************************************************************************