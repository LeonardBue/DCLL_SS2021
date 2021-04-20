% *************************************************************************
%
% [qPLUS, dqdtPLUS, zPLUS, endsStride] = JumpMap(t, qMINUS, dqdtMINUS, zMINUS, p, ControllerFcn, event)
% 
% This code is for a minimal-ccordinate model of a 5-link walker
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
    % Extract continuous states:
    [q1MINUS,q2MINUS,q3MINUS,q4MINUS,q5MINUS,dqdt1MINUS,dqdt2MINUS,dqdt3MINUS,dqdt4MINUS,dqdt5MINUS] = contStates(qMINUS,dqdtMINUS);
    % Extract parameters:
    [g,l1,l2,l3,m1,m2,m3,j1,j2,j3] = systParam(p);
    
    
    % Event 1: touch-down
    switch event
        case 1 % Event 1: detected touch-down
            % The roles of the swing and stance leg switch, so we can directly set 
            % the new angles:
            qPLUS = [q1MINUS - q2MINUS - q3MINUS + q4MINUS + q5MINUS;
                     q5MINUS;
                     q4MINUS;
                     q3MINUS;
                     q2MINUS];
    
            % The velocities are processed in a floating-base system with 7
            % coordinates. To this end, we need to compute the velocities
            % of the hip before collision, and switch swing and stance leg:   
            dqdtMINUS_cont = [ dqdt2MINUS*(l1*cos(q2MINUS-q1MINUS+q3MINUS)/2 + l2*cos(q1MINUS-q2MINUS)) - dqdt1MINUS*(l1*cos(q2MINUS-q1MINUS+q3MINUS)/2 + l3*cos(q1MINUS) + l2*cos(q1MINUS-q2MINUS)) + dqdt3MINUS*l1*cos(q2MINUS-q1MINUS+q3MINUS)/2;
                              -dqdt1MINUS*(l3*sin(q1MINUS) - l1*sin(q2MINUS-q1MINUS+q3MINUS) + l2*sin(q1MINUS-q2MINUS)) - dqdt2MINUS*(l1*sin(q2MINUS-q1MINUS+q3MINUS) - l2*sin(q1MINUS-q2MINUS)) - dqdt3MINUS*l1*sin(q2MINUS-q1MINUS+q3MINUS);
                               dqdt1MINUS - dqdt2MINUS - dqdt3MINUS + dqdt4MINUS + dqdt5MINUS;
                               dqdt5MINUS;
                               dqdt4MINUS;
                               dqdt3MINUS;
                               dqdt2MINUS];
    
            % At touch-down, the contact point comes to a complete rest, a
            % fully plastic collision is computed. Compute contact jacobian
            % and mass matrix for the floating-base system:
            CoGs = CoGPositions(qPLUS, p);
            qPLUS_cont = [CoGs(1); CoGs(2); qPLUS];
            J_cont = ContactJacobian(qPLUS_cont, p);
            M_cont = ContactMassMatrix(qPLUS_cont, p);
            % Project EoM into the contact space:
            % Rest after contact: J*qPlus = 0
            % with: M*(dqPlus-dqMinus) = I_cont_q % with the contact impulse I_cont
            % dqPlus = inv(M)*I_cont_q + dqMinus
            % -> J*inv(M)*I_cont_q + dqMinus = 0
            % -> J*inv(M)*J'*I_cont_x + dqMinus = 0
            % -> I_cont_x = -inv(J*inv(M)*J')*dqMinus
            % -> I_cont_q = -J'*inv(J*inv(M)*J')*dqMinus
            % dqPlus = -inv(M)*J'*inv(J*inv(M)*J')*dqMinus + dqMinus
            dqdtPLUS  = (eye(7) - M_cont\(J_cont'*((J_cont*(M_cont\J_cont'))\J_cont)))*dqdtMINUS_cont;
            % Retrive the post-impact velocities:
            dqdtPLUS = dqdtPLUS(3:7);
            % This event marks the end of the stride
            endsStride = true;
    end
    % Assemble the discrete state vector just after the event:
    zPLUS = zMINUS;
    % Discrete states zPLUS are not present in this model, but need to be
    % output for consistency with the simulation framework 
end