% *************************************************************************
%
% [qPLUS, dqdtPLUS, zPLUS, endsStride] = JumpMap(t, qMINUS, dqdtMINUS, zMINUS, p, ControllerFcn, event)
% 
% This code is for a floating based model of a 5-link walker
%
% The function defines the discrete dynamics. The model's current
% continuous and discrete states before an event (together with the model
% parameters and a controller function) are provided by the calling routine
% to which the states after the event are returned. 

% INPUT:    t             -- time of the event;
%           qMINUS        -- positions vector just before the event;
%           dqdtMINUS     -- velocities vector just before the event;
%           zMINUS        -- vector of discrete states just before the event;
%           p             -- vector of model parameters;
%           ControllerFcn -- handle to the system controller function;
%           event         -- index of the event, as defined in 'JumpSet.m';
% OUTPUT:   qPLUS      -- positions vector just after the event;
%           dqdtPLUS   -- velocities vector just after the event;
%           zPLUS      -- discrete states vector just after the event;
%           endsStride -- a Boolean flag that indicates whether the event
%                         terminates the stride or not.
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
%   See also FLOWMAP, JUMPSET, SYMBOLICCOMPUTATIONOFEOM

function [qPLUS, dqdtPLUS, zPLUS, endsStride] = JumpMap(t, qMINUS, dqdtMINUS, zMINUS, p, ControllerFcn, event)
    % Extract discrete states:
    [phaseL_MINUS, phaseR_MINUS] = discStates(zMINUS);
    
    % As most things remain unchanged, we copy the states before the event:
    qPLUS = qMINUS;
    dqdtPLUS = dqdtMINUS;
    phaseL_PLUS = phaseL_MINUS;
    phaseR_PLUS = phaseR_MINUS;
    
    % Event 1: touch-down left leg
    % Event 2: lift-off left leg
    % Event 3: touch-down right leg
    % Event 4: lift-off right leg
    switch event
        case 1 % Event 1: detected touch-down left leg
            % At touch-down, the contact point comes to a complete rest, a
            % fully plastic collision is computed.  If the right foot is on
            % the ground before the collision, there are two possible 
            % outcomes:
            % A) the right foot leaves the ground
            % B) both feet remain in contact.
            %
            % Compute mass matrix
            M = MassMatrix(qMINUS,p);
            % Contact Jacobians:
            JL = ContactJacobianL(qMINUS,p);
            JR = ContactJacobianR(qMINUS,p);
            % The computation for case A) and B) are the same, just the
            % jacobian J is defined differently:
            % Impact law: Rest after contact -> dcPLUS=J*dqPLUS = 0
            % with: M*(dqPLUS-dqMINUS) = J.'*Lambda 
            % dqPLUS = inv(M)*J.'*Lambda + dqMINUS
            % -> J*inv(M)*J.'*Lambda + J*qMINUS = 0
            % -> Lambda = -inv(J*inv(M)*J')*J*dqMINUS
            % dqPLUS = inv(M)*J'*Lambda + dqMINUS
            % Outcome A)
            J = JL;
            Lambda_A = -((J*(M\J.'))\J)*dqdtMINUS;
            dqdtPLUS_A = M\(J.'*Lambda_A) + dqdtMINUS;
            % Outcome B)
            J = [JL;JR];
            Lambda_B = -((J*(M\J.'))\J)*dqdtMINUS;
            dqdtPLUS_B = M\(J.'*Lambda_B) + dqdtMINUS;
            if(phaseR_MINUS == 0)
                % right leg in the air (only one outcome possible):
                dqdtPLUS = dqdtPLUS_A;
                phaseL_PLUS = 1;
                phaseR_PLUS = 0;
            else
                % right leg on the ground (check both outcomes):
                dcPLUSR = JR*dqdtPLUS_A;
                if (dcPLUSR(2)>0)
                    % If the vertical velocity of the right leg is
                    % positive, then A is a realistic case
                    Apossible = true;
                else
                    Apossible = false;
                end
                if (Lambda_B(4)>0)
                    % If the impuls of the right leg is positive, then B
                    % is a realistic case
                    Bpossible = true;
                else
                    Bpossible = false;
                end
                if (Apossible && ~Bpossible)
                    dqdtPLUS = dqdtPLUS_A;
                    phaseL_PLUS = 1;
                    phaseR_PLUS = 0;
                end
                if (~Apossible && Bpossible)
                    dqdtPLUS = dqdtPLUS_B;
                    phaseL_PLUS = 1;
                    phaseR_PLUS = 1;
                end
                if ((Apossible && Bpossible) || (~Apossible && ~Bpossible))
                    % It appears that the collision outcome is either
                    % ambiguous or includes sliding with infinite friction.
                    % This is something that can happen with rigid body  
                    % collisions and means that our model assumption fails.
                    % There is nothing we can do.  Let's assume case A) and
                    % throw a warning:
                    warning('Collision computation failed.  Erroneously assuming single stance')
                    dqdtPLUS = dqdtPLUS_A;
                    phaseL_PLUS = 1;
                    phaseR_PLUS = 0;
                end
            end
            % Intermediate event. Simulation continues
            endsStride = false; 
        case 2 % Event 2: detected lift-off left leg
            % Velocities remain unchanged.
            % Set new phase for the left leg
            phaseL_PLUS  = 0;
            % Intermediate event. Simulation continues
            endsStride = false; 
        case 3 % Event 3: detected touch-down right leg
            % At touch-down, the contact point comes to a complete rest, a
            % fully plastic collision is computed.  If the left foot is on
            % the ground before the collision, there are two possible 
            % outcomes:
            % A) the left foot leaves the ground
            % B) both feet remain in contact.
            %
            % Compute mass matrix
            M = MassMatrix(qMINUS,p);
            % Contact Jacobians:
            JL = ContactJacobianL(qMINUS,p);
            JR = ContactJacobianR(qMINUS,p);
            % The computation for case A) and B) are the same, just the
            % jacobian J is defined differently:
            % Impact law: Rest after contact -> dcPLUS=J*dqPLUS = 0
            % with: M*(dqPLUS-dqMINUS) = J.'*Lambda 
            % dqPLUS = inv(M)*J.'*Lambda + dqMINUS
            % -> J*inv(M)*J.'*Lambda + J*qMINUS = 0
            % -> Lambda = -inv(J*inv(M)*J')*J*dqMINUS
            % dqPLUS = inv(M)*J'*Lambda + dqMINUS
            % Outcome A)
            J = JR;
            Lambda_A = -((J*(M\J.'))\J)*dqdtMINUS;
            dqdtPLUS_A = M\(J.'*Lambda_A) + dqdtMINUS;
            % Outcome B)
            J = [JL;JR];
            Lambda_B = -((J*(M\J.'))\J)*dqdtMINUS;
            dqdtPLUS_B = M\(J.'*Lambda_B) + dqdtMINUS;
            if(phaseL_MINUS == 0)
                % left leg in the air (only one outcome possible):
                dqdtPLUS = dqdtPLUS_A;
                phaseL_PLUS = 0;
                phaseR_PLUS = 1;
            else
                % left leg on the ground (check both outcomes):
                dcPLUSL = JL*dqdtPLUS_A;
                if (dcPLUSL(2)>0)
                    % If the vertical velocity of the left leg is
                    % positive, then A is a realistic case
                    Apossible = true;
                else
                    Apossible = false;
                end
                if (Lambda_B(2)>0)
                    % If the impuls of the left leg is positive, then B
                    % is a realistic case
                    Bpossible = true;
                else
                    Bpossible = false;
                end
                if (Apossible && ~Bpossible)
                    dqdtPLUS = dqdtPLUS_A;
                    phaseL_PLUS = 0;
                    phaseR_PLUS = 1;
                end
                if (~Apossible && Bpossible)
                    dqdtPLUS = dqdtPLUS_B;
                    phaseL_PLUS = 1;
                    phaseR_PLUS = 1;
                end
                if ((Apossible && Bpossible) || (~Apossible && ~Bpossible))
                    % It appears that the collision outcome is either
                    % ambiguous or includes sliding with infinite friction.
                    % This is something that can happen with rigid body  
                    % collisions and means that our model assumption fails.
                    % There is nothing we can do.  Let's assume case A) and
                    % throw a warning:
                    warning('Collision computation failed.  Erroneously assuming single stance')
                    dqdtPLUS = dqdtPLUS_A;
                    phaseL_PLUS = 0;
                    phaseR_PLUS = 1;
                end
            end
            % Intermediate event. Simulation continues
            endsStride = false; 
        case 4 % Event 4: detected liftoff right leg
            % Velocities remain unchanged.
            % Set new phase for the right leg
            phaseR_PLUS  = 0;
            % This event marks the end of the stride
            endsStride = true;
    end
    % Assemble the discrete state vector just after the event:
    zPLUS = [phaseL_PLUS; phaseR_PLUS];
end
% *************************************************************************
% *************************************************************************