% *************************************************************************
%
% function [t, q, dqdt, z] = TimeStepping(q0, dqdt0, z0, p, simOptions)
%
% This function simulates the planar hybrid dynamics associated with the 
% dynamic model available on the MATLAB search path.  In particular, it 
% needs the following functions (ideally created with
% SymbolicComputationOfEoM): 
%
% M    = MassMatrix(q, p);             - The mass matrix
% f_cg = F_CoriGrav(q, dqdt, p);       - The coriolis, centrifugal, and gravitational forces
% dN   = ContactDistanceN(q, p);       - The normal position of all contact points
% JT   = ContactJacobianT(q, p);       - The contact Jacobian in tangential direction
% JN   = ContactJacobianN(q, p);       - The contact Jacobian in normal direction
%
% Input to these functions are the generalized coordinates 'q', generalized
% velocities 'dqdt', and the model parameters p. 
%
% TimeStepping returns trajectories for time 't', joint angles 'q', joint
% velocities 'dqdt', contact flags 'z' (1=contact, 0= no contact),
% tangential contact impulses 'pT', and normal contact impulses 'pN'.
% 
% INPUT:    q0     -- vector of initial positions of the system;
%           dqdt0  -- vector of initial velocities of the system;
%           z0     -- contact flags (1=contact, 0=no contact); these are
%                     automatically computed by the solver and should not 
%                     be provided as input;
%           p       -- vector of the system parameters;
%           simOptions -- structure providing additional simulation options;
%               Possible options are:
%               - options.tSTART        -- simulation initial time;
%               - options.tMAX          -- maximum simulation time;
%               - options.ControllerFcn -- handle to a controller function;
%               - options.graphOUTPUT   -- graphical output object which 
%                 must be derived from the class 'OutputCLASS'; leave it 
%                 out or set to [] for no animation;
%               - options.dtGraphOUT    -- time step of the animation;
%               - options.dt            -- time step of the time-stepping
%                 integration scheme and also of the numerical output;
%               - options.mu            -- coefficient of friction;
%               - options.r             -- iteration parameter;
%               - options.acc           -- accuracy of integration;
%               - options.nMax          -- maximum number of iterations.
% 
% OUTPUT:   t    -- time points of the resulting system trajectory;
%           q    -- trajectories of the position states;
%           dqdt -- trajectories of the velocity states;
%           z    -- trajectories of the contact flags (1=contact, 0= no contact)
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

function [t, q, dqdt, z] = TimeStepping(q0, dqdt0, ~, p, simOptions)

    % Process optional inputs and get values for various simulation
    % parameters:
    [tSTART,tMAX,ControllerFcn,graphOUTPUT,dt,mu,r,acc,nMax] = handleOptions(simOptions);
    
    % Determine the number of contact points:
    nContPts = length(ContactDistanceN(q0,p));
    
    % Timing
    t = tSTART:dt:tMAX;
    
    % Initialize output vectors:
    q     = zeros(size(q0,1),size(t,2));
    dqdt  = zeros(size(q0,1),size(t,2));
    z     = zeros(size(ContactDistanceN(q0, p),1),size(t,2));
    % These are tangential and normal contact impulses; they are currently
    % not provided as the output, but we store them anyway:
    pT    = zeros(size(ContactDistanceN(q0, p),1),size(t,2));
    pN    = zeros(size(ContactDistanceN(q0, p),1),size(t,2));
    
    % Initialize time stepping:
    qA    = q0;
    dqdtA = dqdt0;
    PT    = zeros(nContPts,1);
    PN    = zeros(nContPts,1);
    
    tOutput = graphOUTPUT.rate;
    q(:,1)    = qA;
    dqdt(:,1) = dqdtA;
    for i = 2:size(t,2)
        % Integrate positions with an Euler-Forward Step to the middle of
        % the current interval:
        qM = qA + dt/2*dqdtA;
        % Determine which contact points are in penetration
        dN = ContactDistanceN(qM, p);
        % Compute components of dynamic equations:
        M = MassMatrix(qM, p);
        f_cg = F_CoriGrav(qM,dqdtA,p);  % NOTE:  this is evaluated with the initial velocity (velocities jump in the middle of the interval)
        JT = ContactJacobianT(qM, p);
        JN = ContactJacobianN(qM, p);
        
        % Controller:
        tau = ControllerFcn(t(i), qA, dqdtA, dN<=0, p);
        
        % Prepare the iteration that is used to determine the contact
        % impulses:
        PT_OLD = PT;
        PN_OLD = PN;
        % Pre-compute the non-contact dependent components of the EoMs:
        dqdtA_invM_fcg_tau = dqdtA + M\(f_cg*dt + tau*dt);
        invM_JT = M\JT.';
        invM_JN = M\JN.';
        % Fixed-point iteration over PN and PT:
        for j = 1:nMax
            % Solve the EoMs with the current contact impulses:    
            % dqdtE = dqdtA + invM*(f_cg*p.dt + JT'* PT_OLD + JN'*PN_OLD + tau*p.dt);
            % This is a faster version which uses some pre-computed
            % expressions:
            dqdtE = dqdtA_invM_fcg_tau + invM_JT*PT_OLD + invM_JN*PN_OLD;
            % Get an update for the contact impulses.
            % This is prox interval [-mu*PN, + mu*PN]:
            PT = max(-mu*PN_OLD, min(+mu*PN_OLD, PT_OLD - r*JT*dqdtE));
            % This is % prox R_0^+
            PN = max(0, PN_OLD - r*JN*dqdtE);  
            % Set impulses for feet that are not in contact to 0:
            PT(dN>0)=0;
            PN(dN>0)=0;
            if norm([PT-PT_OLD;PN-PN_OLD]) < acc
                break;
            end
            % Prepare for next step
            PT_OLD = PT;
            PN_OLD = PN;
        end
        if (j == nMax)
            warning('Fixed point iteration did not converge.')
        end
        % Integrate positions with a Euler-Forward Step to the end of
        % the current interval:
        qE = qM + dt/2*dqdtE;
        
        % Produce graphical output:
        if t(i) >= tOutput
            update(graphOUTPUT, qE, [], tOutput, tau);
            tOutput = tOutput + graphOUTPUT.rate;
        end
        % Store results:
        q(:,i)    = qE;
        dqdt(:,i) = dqdtE;
        z(:,i)    = dN<=0;
        pT(:,i)   = PT;
        pN(:,i)   = PN;
        % Prepare for next step:
        qA    = qE;
        dqdtA = dqdtE;
    end
    
end


% *********************************************************************
function [tSTART,tMAX,ControllerFcn,graphOUTPUT,dt,mu,r,acc,nMax] = handleOptions(options)
% Handle all optional inputs to the simulation. For each option, check if 
% it has been provided and, if not, set it to a default value:
    % Initial simulation time:
    if isfield(options,'tSTART')
        tSTART = options.tSTART;
    else
        tSTART = 0;
    end
    % Maximum simulation time:
    if isfield(options,'tMAX')
        tMAX = options.tMAX;
    else
        tMAX = 1000;
    end
    % Controller:
    if isfield(options,'controller')
        ControllerFcn = options.controller;
    else
        % If no controller is provided, use the ZeroTorques controller:
        path(path, genpath('Controllers\NoControl'));
        ControllerFcn = @Controller_NoControl;
    end
    % Graphics output class:
    if isfield(options,'graphOUTPUT') && isa(options.graphOUTPUT,'OutputCLASS')
        graphOUTPUT = options.graphOUTPUT;
    else
        graphOUTPUT = [];
    end
    % Graphical output time step:
    if ~isempty(graphOUTPUT) && isfield(options,'dtGraphOUT') && ~isempty(options.dtGraphOUT)
        graphOUTPUT.rate = options.dtGraphOUT;
    else
        % If graphOUTPUT is provided but dtGraphOUT is not, default rate is 
        % used that is defined in the graphOUTPUT class.
    end
    
    %%% Parameters of the time-stepping algorithm:
    % Time step:
    if isfield(options,'dt')
        dt = options.dt;
    else
        dt = 0.001;
    end
    % Coefficient of friction:
    if isfield(options,'mu')
        mu = options.mu;
    else
        mu = 10;
    end
    % Iteration parameter:
    if isfield(options,'r')
        r = options.r;
    else
        r = 0.01;
    end
    % Accuracy of iteration:
    if isfield(options,'acc')
        acc = options.acc;
    else
        acc = 1e-10;
    end
    % Maximum number of iterations:
    if isfield(options,'nMax')
        nMax = options.nMax;
    else
        nMax = 1000;
    end
end
% *********************************************************************
