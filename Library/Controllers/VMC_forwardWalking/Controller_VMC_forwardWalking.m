% *************************************************************************
%
%   tau = Controller_VMC_forwardWalking(t, q, dqdt, z, p)
%
% This is a virtual model controller for forward walking of a 5-link biped
% in 2 D. It implements: 1) vertical force on the hip to keep it at a 
% desired height; 2) torque on the main body to keep its pitch fixed; 
% 3) during double stance, horizontal force on the hip to maintain forward 
% speed.
%
% This controller works with the following models:
%    - 5LinkBiped_floatBase
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

function tau = Controller_VMC_forwardWalking(t, q, dqdt, z, p)
    % Extract continuous states:
    [x,y,phi,alphaL,alphaR,betaL,betaR,...
        dx,dy,dphi,dalphaL,dalphaR,dbetaL,dbetaR] = contStates(q, dqdt);
    % Extract system parameters:
    [g,l1,l2,l3,m1,m2,m3,j1,j2,j3] = systParam(p);

    %% Define motion targets:
    dx_des  = 0.3; % desired forward speed
    y_des   = 0.9; % desired hip height
    phi_des = 0;   % desired body pitch
    
    %% Compute desired virtual forces (corresponding to the set motion targets):
    % horizontal force at the hip:
    D=2;  % only control speed, not position 
    Fx = D*(dx_des - dx);            
    % vertical force at the hip:
    P=10; 
    D=2;
    Fy = P*(y_des - y) + D*(0 - dy);     
    % torque at the torso
    P=5; 
    D=1;
    Fphi = P*(phi_des - phi) + D*(0 - dphi);    
   
    %% Swing-leg control parameters:
    % Step length:
    x_step = 0.3;    
    % Horizontal distance between CoM and the stance foot at which the
    % swing controller switches from clearing the ground to positioning the
    % leg to make a step:
    delta_x = 0.1;
    % Parameters of the ground-clearance swing controller (in the first
    % half of the swing phase):
    delta_alpha = 0.3;  % the swing thigh is controlled to be ahead of the stance thigh by this angle
    beta_clear  = 1.5;  % target swing knee angle
    % Position and derivative gains in the swing-leg position control:
    Psw = 2;
    Dsw = 0.5;
    
    
    %% Compute corresponding joint torques:
    % Evaluate system kinematics and dynamics:
    F_cg = F_CoriGrav(q,dqdt,p);
    CoGs = CoGPositions(q,p);
    footPts = FootPtPositions(q,p);
    % left-leg and right-leg contact point Jacobians:
    JL = ContactJacobianL(q,p);
    JR = ContactJacobianR(q,p);
    
    %% Depending on the contact configuration, we can apply different 
    %% numbers of forces to the main body and will use different control 
    %% for the swing legs: 
    if z(1) && ~z(2)
        % LEFT SINGLE STANCE (control only height and pitch angle)
        % Extract partial Jacobians for the passive and active degrees of
        % freedom (ignoring x direction): 
        JP = JL(:,2:3);
        JA = JL(:,4:7);
        % Compute desired contact forces (ignoring x direction):
        lamda_des = JP' \ ([Fy; Fphi] - F_cg(2:3));
        % Compute corresponding motor torques:
        tau_mot = -JA'*lamda_des - F_cg(4:7);
        %%% Swing leg controller:
        if CoGs(1) < footPts(1,1) + delta_x
            % if the MB is not forward enough (first part of single
            % stance), move the swing leg forward and clear the ground
            alphaSwing_des = alphaL + delta_alpha;
            betaSwing_des = beta_clear;
        else
            % if the MB is sufficiently forward, move the swing foot to
            % desired stepping location; use inverse kinematics to
            % determine the target angles of the swing leg
            x_rel = x_step + footPts(1,1) - x;  % horizontal position of the desired stepping location relative to the hip
            [alphaSwing_des, betaSwing_des] = legIK(x_rel, -y);
        end
        % swing hip position control:
        tau_mot(2) = Psw*(alphaSwing_des - alphaR) + Dsw*(dalphaL - dalphaR);
        % swing knee position control:
        tau_mot(4) = Psw*(betaSwing_des - betaR) + Dsw*(0 - dbetaR);
        
    elseif ~z(1) && z(2)
        % RIGHT SINGLE STANCE (control only height and pitch angle)
        % Extract partial Jacobians for the passive and active degrees of
        % freedom (ignoring x direction): 
        JP = JR(:,2:3);
        JA = JR(:,4:7);
        % Compute desired contact forces (ignoring x direction):
        lamda_des = JP' \ ([Fy; Fphi] - F_cg(2:3));
        % Compute corresponding motor torques:
        tau_mot = -JA'*lamda_des - F_cg(4:7);
        %%% Swing leg controller:
        if CoGs(1) < footPts(1,2) + delta_x
            % if the MB is not forward enough (first part of single
            % stance), move the swing leg forward and clear the ground
            alphaSwing_des = alphaR + delta_alpha;
            betaSwing_des  = beta_clear;
        else
            % if the MB is sufficiently forward, move the swing foot to
            % desired stepping location; use inverse kinematics to
            % determine the target angles of the swing leg
            x_rel = x_step + footPts(1,2) - x;  % horizontal position of the desired stepping location relative to the hip
            [alphaSwing_des, betaSwing_des] = legIK(x_rel, -y);
        end
        % swing hip position control:
        tau_mot(1) = Psw*(alphaSwing_des - alphaL) + Dsw*(dalphaR - dalphaL);
        % swing knee position control:
        tau_mot(3) = Psw*(betaSwing_des - betaL) + Dsw*(0 - dbetaL);
        
    elseif z(1) && z(2)
        % DOUBLE STANCE (control all torso DOFs)
        % Extract partial Jacobians for the passive and active degrees of
        % freedom: 
        JP = [JL(:,1:3); JR(:,1:3)];
        JA = [JL(:,4:7); JR(:,4:7)];
        % Compute desired contact forces (using a least squares solution to
        % find minimal lambdas): 
        lamda_des = pinv(JP') * ([Fx; Fy; Fphi] - F_cg(1:3));
        % Compute corresponding motor torques:
        tau_mot = -JA'*lamda_des - F_cg(4:7);
    else
        % FLIGHT (set all output torques to 0)
        tau_mot = zeros(4,1);
    end
    
    % Assign joint torques to the extended torque vector:
    tau = zeros(7,1);
    tau(4:7) = tau_mot;
    
    
    % Inverse kinematics of the 5-link walker:
    function [alpha, beta] = legIK(x_rel, y_rel)
        % Input: relative positions x_rel and y_rel of a foot with respect to the hip
        % Output: hip (alpha) and knee (beta) angles of the leg
        beta  = acos( (y_rel^2+x_rel^2-l2^2-l3^2)/(2*l2*l3) );
        alpha = phi - asin( sin(beta)*l3/sqrt(y_rel^2+x_rel^2) ) - atan(x_rel/y_rel);
    end
    
end
