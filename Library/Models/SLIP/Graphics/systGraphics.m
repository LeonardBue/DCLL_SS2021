% *************************************************************************
%
% graphicsHandle = systGraphics(q0, p)
% 
% This code is for a floating based model of the SLIP model
%
% This function provides the simulation framework with a handle to a 
% graphics object that creates animation of the simulated system. The 
% output graphicsHandle has to be a class of type OutputCLASS.
% 
% INPUT:    q0  --  vector of generalized coordinates of the system;
%           p   --  vector of system parameters.
% OUTPUT    graphicsHandle  --  handle to a graphics object of class OutputCLASS.
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
%   See also SYMBOLICCOMPUTATIONOFEOM

function graphicsHandle = systGraphics(q0, p)

    % Use standard SLIP graphics
    graphicsHandle = SLIP_Model_Graphics(q0,p);
end
% *************************************************************************
% *************************************************************************