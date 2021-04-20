% *************************************************************************
%
%   tau = Controller_StaticLocomotion(t, q, dqdt, z, p)
%
% This is a static walking controller that uses an inverse kinematics
% approach and PID controller to let the feet and main body track desired
% trajectories.
%
% This controller works with the following models:
%    - 7LinkBiped_floatBase
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

function tau = Controller_StaticLocomotion(t, q, dqdt, z, p)
    % Extract continuous states:
    [x,y,phi,alphaL,alphaR,betaL,betaR,gammaL,gammaR,...
        dx,dy,dphi,dalphaL,dalphaR,dbetaL,dbetaR,dgammaL,dgammaR] = contStates(q, dqdt);
    % Extract System parameters:
    [g,l1,l2,l3,l4,m1,m2,m3,m4,j1,j2,j3,j4] = systParam(p);

    % Retrieve desired trajectories for feet and mainbody (in world
    % coordinates)
    [xF_l, yF_l, xF_r, yF_r, xM, yM] = ComputeKinematicTrajectories(t);

    % Run inverse kinematics
    % Explicit, fast:
    betaL_des  = -acos(((xF_l - xM-l4/2)^2 + (yF_l - yM)^2-l2^2-l2^2)/(2*l2*l2));
    alphaL_des = -atan((xF_l - xM-l4/2)/(yF_l - yM)) - atan((l3*sin(betaL_des))/(l2+l3*cos(betaL_des)));
    gammaL_des = -alphaL_des - betaL_des + pi/2;
    betaR_des  = -acos(((xF_r - xM-l4/2)^2 + (yF_r - yM)^2-l2^2-l2^2)/(2*l2*l2));
    alphaR_des = -atan((xF_r - xM-l4/2)/(yF_r - yM)) - atan((l3*sin(betaR_des))/(l2+l3*cos(betaR_des)));
    gammaR_des = -alphaR_des - betaR_des + pi/2;

    %% Run a simple PD controller for each joint:
    tau = zeros(9,1);
    P = 5;
    D = 2;
    tau(4) = P*(alphaL_des - alphaL) + D*(0 - dalphaL);
    tau(5) = P*(alphaR_des - alphaR) + D*(0 - dalphaR);
    tau(6) = P*(betaL_des - betaL) + D*(0 - dbetaL);
    tau(7) = P*(betaR_des - betaR) + D*(0 - dbetaR);

    % only apply large ankle torques when the foot is on the ground:
    if z(1) && z(3)
        P = 50; D = 20;
    else
        P = .5; D = .2;
    end
    tau(8) = P*(gammaL_des - gammaL) + D*(0 - dgammaL);
    if z(2) && z(4)
        P = 50; D = 20;
    else
        P = .5; D = .2;
    end
    tau(9) = P*(gammaR_des - gammaR) + D*(0 - dgammaR);
end