% *************************************************************************
%
%   angAtt = Controller_Raibert(t, q, dqdt, z, p)
%
% This is a Raibert-style controller for the angle-of-attack of the simple 
% SLIP model. During each flight phase, the controller sets the
% angle-of-attack so as to maintain a given desired speed.
% 
% This controller works with the following models:
%    - SLIP
%
% Note that the syntax of the controller function has to be the same for
% any controller used with the simulation framework. 
%
%   u = Controller(t, q, dqdt, z, p)
%
% INPUT:    t    -- time point;
%           q    -- vector of generalized positions;
%           dqdt -- vector of generalized rates;
%           z    -- vector of discrete states;
%           p    -- vector of system parameters.
% OUTPUT:   u    -- controller outputs. If time-stepping integration is
%                   used, these have to be joint torques; if event-based
%                   integration is used, the output u is passed into the
%                   dynamics functions of the system (FlowMap, JumpMap,
%                   JumpSet).
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
%   See also TIMESTEPPING, HYBRIDDYNAMICS,  FLOWMAP, JUMPMAP, JUMPSET

function angAtt = Controller_Raibert(t, q, dqdt, z, p)
    % Controller parameters:
    dx_des   = 1;     % [sqrt(g*l_0)] desired speed
    T_stance = 0.64;  % [sqrt(l_0/g)] duration of stance phase
    k_dx     = 0.1;   % [-] speed feedback gain

    % extract states and parameters:
    [x,y,dx,dy] = contStates(q,dqdt);
    [g, l_0, m_0, k, angAtt] = systParam(p);


    % Raibert controller for the angle-of-attack:
    angAtt = asin( dx*T_stance/(2*l_0) + k_dx*(dx - dx_des)/l_0 );
end

