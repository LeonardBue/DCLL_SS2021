% *************************************************************************
%
% varargout = systParam(p)
% 
% This code is for a minimal-ccordinate model of a 5-link walker
%
% This function defines the default values of the system parameters and
% their order within the parameter vector p. The system parameters stay 
% constant throughout simulation; their order within p has to remain fixed
% as the vector p is passed to various functions in the simulation 
% framework.
% There are two ways to call this function:
% 
%                                p = systParam([]);
%   [g,l1,l2,l3,m1,m2,m3,j1,j2,j3] = systParam(p);
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

function varargout = systParam(p)

    if isempty(p)  % if empty input, define system parameters
        % Physics:
        g   = 1;     % [g] gravity
        % Parameters of the model
        l1  = 0.6;   % [l_0] main body COG height
        l2  = 0.5;   % [l_0] tight length
        l3  = 0.5;   % [l_0] shank length
        m1  = 0.5;   % [m_0] mass of the main body
        m2  = 0.15;  % [m_0] mass of the thigh
        m3  = 0.1;   % [m_0] mass of the shank
        j1  = 0.02;  % [m_0*l_0^2] inertia of the main body
        j2  = 0.004; % [m_0*l_0^2] inertia of the thigh
        j3  = 0.004; % [m_0*l_0^2] inertia of the shank
        % Assemble parameter vector:
        p = [g, l1, l2, l3, m1, m2, m3, j1, j2, j3]';
    end

    % Generate output:
    if nargout == 1
        % Call sequence:   p = systParam([]);
        varargout{1} = p;
    elseif nargout > 1
        % Call sequence:   [g,l1,l2,l3,m1,m2,m3,j1,j2,j3] = systParam(p);
        varargout = num2cell(p);
    end
end
% *************************************************************************
% *************************************************************************