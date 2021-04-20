% *************************************************************************
%
% evntVal = JumpSet(t, q, dqdt, z, p, ControllerFcn)
%
% This code is for a floating based model of a 5-link walker
%
% The function defines the occurrence of discrete events. The model's
% current continuous and discrete states together with the model parameters
% and a controller function are provided by the calling routine to which a
% vector of event function values is returned. The directional
% zero-crossings of these functions each trigger a different event. 
%
% INPUT:    t    -- current time instant;
%           q    -- vector of positions (generalized coordinates);
%           dqdt -- vector of velocities;
%           z    -- vector of discrete states;
%           p    -- vector of model parameters;
%           ControllerFcn -- handle to the system controller function;
% OOUTPUT:  evntVal -- vector of values of the event functions; each 
%                      element corresponds to a function, of which a 
%                      zero-crossing (with positive derivative) is detected
%                      as an event.
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
%   See also FLOWMAP, JUMPMAP, SYMBOLICCOMPUTATIONOFEOM

function evntVal = JumpSet(t, q, dqdt, z, p, ControllerFcn)
    % Extract discrete states:
    [phaseL, phaseR] = discStates(z);
    
    % Evalute the controller function
    tau = ControllerFcn(t, q, dqdt, z, p);
        
    % Compute the differentiable force vector (i.e. coriolis, gravity, and
    % actuator forces): 
    f_cg = F_CoriGrav(q,dqdt,p);
    % Mass matrix
    M = MassMatrix(q,p);
    % Contact kinematics:
    posL  = ContactPointL(q,p);
    JL    = ContactJacobianL(q,p);
    dJLdtTIMESdqdt = ContactJacobianLDtTIMESdqdt(q,dqdt,p);
    posR  = ContactPointR(q,p);
    JR    = ContactJacobianR(q,p);
    dJRdtTIMESdqdt = ContactJacobianRDtTIMESdqdt(q,dqdt,p);

    % Compute current contact forces:
    % Requirement for a closed contact is that the contact point
    % acceleration is zero:
    % J*ddq + dJdt*dq = 0 
    % with EoM:
    % ddq = inv(M)*(f_cg + J'*lambda + tau)
    % -> J*inv(M)*(f_cg + J'*lambda + tau) + dJdt*dq = 0
    % -> J*inv(M)*(f_cg + tau) + J*inv(M)*J'*lambda + dJdt*dq = 0
    % -> lambda = inv(J*inv(M)*J')*(-J*inv(M)*(f_cg  +tau) - dJdt*dq)
    if (phaseL == 0) && (phaseR == 0)
        % both legs in the air:
        lambda_L = [0;0];
        lambda_R = [0;0];
    end
    if (phaseL == 1) && (phaseR == 0)
        % only left leg on the ground:
        lambda_L = (JL*(M\JL'))\(-JL*(M\(f_cg+tau)) - dJLdtTIMESdqdt);
        lambda_R = [0;0];
    end
    if (phaseL == 0) && (phaseR == 1)
        % only right leg on the ground:
        lambda_L = [0;0];
        lambda_R = (JR*(M\JR'))\(-JR*(M\(f_cg+tau)) - dJRdtTIMESdqdt);
    end
    if (phaseL == 1) && (phaseR == 1)
        % both feet on the ground:
        J             = [JL;JR];
        dJdtTIMESdqdt = [dJLdtTIMESdqdt; dJRdtTIMESdqdt];
        f_contX = (J*(M\J'))\(-J*(M\(f_cg+tau)) - dJdtTIMESdqdt);
        lambda_L = f_contX(1:2);
        lambda_R = f_contX(3:4);
    end
        
    % Event 1: touch-down left leg
    % Event 2: lift-off left leg
    % Event 3: touch-down right leg
    % Event 4: lift-off right leg
    n_events = 4;
    evntVal = zeros(n_events,1);
    
    % *******
    % Event 1: Detect touch-down left leg
    if phaseL == 0
        % Event is detected if left foot goes below the ground during
        % flight
        evntVal(1) = -posL(2);
    else
        % But only in phase 2
        evntVal(1) = -1;
    end
    
    % *******
    % Event 2: Detect lift-off left leg
    if phaseL == 1
        % Event is detected when vertical contact forces becomes negative:
        evntVal(2) = - lambda_L(2) - 0.05; % Add a little offset to eliminate numerical jitter
    else
        % But only in phase 1
        evntVal(2) = -1;
    end
    
    % *******
    % Event 3: Detect touch-down right leg
    if phaseR == 0
        % Event is detected if right foot goes below the ground during
        % flight 
        evntVal(3) = -posR(2);
    else
        % But only in phase 2
        evntVal(3) = -1;
    end
    
    % *******
    % Event 4: Detect lift-off right leg
    if phaseR == 1
        % Event is detected when vertical contact forces becomes negative:
        evntVal(4) = - lambda_R(2) - 0.05; % Add a little offset to eliminate numerical jitter
    else
        % But only in phase 1
        evntVal(4) = -1;
    end
end
% *************************************************************************
% *************************************************************************