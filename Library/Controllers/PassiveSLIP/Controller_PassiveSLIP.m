% *************************************************************************
%
%   angAtt = Controller_PassiveSLIP(t, q, dqdt, z, p)
%
% This is a trivial controller for the simple SLIP model which set the
% angle-of-attack to a fixed value. Effectively, this makes the SLIP model
% passive.
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

function angAtt = Controller_PassiveSLIP(t, q, dqdt, z, p)
    % Use the default angle-of-attack:
    [g,l_0,m_0,k,angAtt] = systParam(p);
end

