% *************************************************************************
%
% varargout = systParam(p)
% 
% This code is for a floating based model of the SLIP model
%
% This function defines the default values of the system parameters and
% their order within the parameter vector p. The system parameters stay 
% constant throughout simulation; their order within p has to remain fixed
% as the vector p is passed to various functions in the simulation 
% framework.
% There are two ways to call this function:
% 
%                      p = systParam([]);
%   [g,l_0,m_0,k,angAtt] = systParam(p);
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
        g       = 1;     % [g] gravity
        % Parameter of the model
        l_0     = 1;   % [l_0] main body COG height
        m_0     = 1;   % [m_0] mass of the main body
        k       = 25;  % [m_0*g/l_0] linear spring stiffness in the leg
        angAtt  = 0;   % [rad] angle of attack of the leg (0 = straight downwards);
                       % this angle value is used if controller is turned off
        % Assemble parameter vector:
        p = [g, l_0, m_0, k, angAtt]';
    end

    % Generate output:
    if nargout == 1
        % Call sequence:   p = systParam([]);
        varargout{1} = p;
    elseif nargout > 1
        % Call sequence:   [g, l_0, m_0, k, angAtt] = systParam(p);
        varargout = num2cell(p);
    end
end
% *************************************************************************
% *************************************************************************