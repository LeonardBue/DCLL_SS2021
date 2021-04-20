% *************************************************************************
%
% varargout = discStates(z)
% 
% This code is for a floating based model of a 5-link walker
%
% This function extracts from the vector of discrete states the values of 
% individual states and returns them as separate variables. Thus, this 
% function defines the order of variables within the discrete state vector
% that should be kept the same throughout the model simulation and analysis.
%
% INPUT:    z -- vector of the discrete states;
% OUTPUT:   the set of the system discrete states.
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

function [phaseL, phaseR] = discStates(z)

    phaseL = z(1); % The current phase of the left leg  (flight = 0, stance = 1)
    phaseR = z(2); % The current phase of the right leg (flight = 0, stance = 1)
end
% *************************************************************************
% *************************************************************************