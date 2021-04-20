% *************************************************************************
%
% varargout = systParam(p)
% 
% This code is for a floating based model of the Simplest Walking Model
%
% This function defines the default values of the system parameters and
% their order within the parameter vector p. The system parameters stay 
% constant throughout simulation; their order within p has to remain fixed
% as the vector p is passed to various functions in the simulation 
% framework.
% There are two ways to call this function:
% 
%   	p = systParam([]);
%   gamma = systParam(p);
% 
% - In the first case, the function assigns default values to the model
% parameters and returns them as a single parameter vector p.
% - In the second case, this function extracts the values of individual 
% parameters from the input vector p and returns them as separate
% variables. If p is empty, default parameter values are returned.
% 
% INPUT:    p -- vector of the model parameters;
% OUTPUT:   the set of the model parameters.
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

function gamma = systParam(p)

    % The Simplest Walking Model has only one parameter, the slope angle gamma.
    % Hence, this function either provides the default slope angle (if the
    % input p is empty) or returns p unchanged.

    if isempty(p)  % if empty input, define system parameters
        gamma = 0.01;    % [rad] slope angle
    else
        gamma = p;
    end
end
% *************************************************************************
% *************************************************************************