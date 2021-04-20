% *************************************************************************
%
% ddqddt = FlowMap(t, q, dqdt, z, p, ControllerFcn)
% 
% This code is for a floating based model of a 5-link walker
%
% The function defines the continuous dynamics. The model's current
% continuous and discrete states, as well as the model parameters and a
% controller function are given by the calling routine, and the generalized 
% accelerations are returned. 
%
% INPUT:    t    -- current time instant;
%           q    -- a vector of positions (generalized coordinates);
%           dqdt -- a vector of velocities;
%           z    -- a vector of discrete states;
%           p    -- a vector of model parameters;
%           ControllerFcn -- a handle to the system controller function;
% OUTPUT:   ddqddq -- accelerations of the generalized coordinates q.
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
%   See also JUMPMAP, JUMPSET, SYMBOLICCOMPUTATIONOFEOM


function ddqddt = FlowMap(t, q, dqdt, z, p, ControllerFcn)
    % Extract discrete states:
    [phaseL, phaseR] = discStates(z);
       
    % Evalute the controller function
    tau = ControllerFcn(t, q, dqdt, z, p);
    % Compute the differentiable force vector (i.e. coriolis, gravity, and
    % actuator forces): 
    f_cg = F_CoriGrav(q, dqdt, p);
    % Mass matrix
    M = MassMatrix(q, p);
    % Contact kinematics:
    JL = ContactJacobianL(q, p);
    dJLdtTIMESdqdt = ContactJacobianLDtTIMESdqdt(q, dqdt, p);
    JR = ContactJacobianR(q, p);
    dJRdtTIMESdqdt = ContactJacobianRDtTIMESdqdt(q, dqdt, p);

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
    
    % EQM:
    ddqddt = M\(f_cg + JL.'*lambda_L + JR.'*lambda_R + tau);
end
% *************************************************************************
% *************************************************************************
    