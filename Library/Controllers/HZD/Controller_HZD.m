% *************************************************************************
%
%   tau = Controller_HZD(t, q, dqdt, z, p)
%
% This is a Hybrid Zero Dynamics (HZD) controller for a 5-link bipedal
% model in 2D.
%
% This controller works with the following models:
%    - 5LinkBiped_minCoord
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

function tau_joint = Controller_HZD(t, q, dqdt, z, p)

    % Compute the current phase variable and its time derivative:
    th     = -q(1) + q(2)/2;
    dth_dq = [-1, 1/2, 0, 0, 0];
    dth_dt = dth_dq*dqdt;

    % Compute the constraint function hD and the relevant derivatives:
    [hD, dhD_dth, ddhD_ddth] = targetEvolution(th);
    
    % Compute the constraint error, and its derivatives
    y     = q(2:5) - hD;
    dy_dq = [zeros(4,1),eye(4)] - dhD_dth*dth_dq;
    dy_dt = dy_dq*dqdt;
    
    % Set up the components of the EoM:
    M    = MassMatrix(q,p);
    f_cg = F_CoriGrav(q,dqdt,p);
    S    = [zeros(1,4); eye(4)];

    % Compute controls required to remain on the virtual constraints
    tau_constraint = (dy_dq*(M\S)) \ (ddhD_ddth*dth_dt^2 - dy_dq*(M\f_cg));
    
    
    % Compute controls to asymptotically stabilize the constraint error!
    % Implement a PD controller
    Kp = 50;
    Kd = 2*sqrt(Kp);
    % Compute required torques to covnerge back to the virtual constraints:
    tau_control = (dy_dq*(M\S)) \ ( - Kp*y - Kd*dy_dt);
    
    % Extract active joint torques:
    tau = tau_constraint + tau_control;
    tau_joint = S*tau;
end