% *************************************************************************
%
% function [tOUT, qOUT, dqdtOUT, zOUT] = HybridDynamics(qIN, dqdtIN, zIN, p, options)
%
% This MATLAB function simulates the hybrid dynamics associated with the
% functions 'JumpSet', 'JumpMap', and 'FlowMap'.  These functions must be
% available on the MATLAB search path:
%
% - evntVal = JumpSet(t,q,dqdt,z,p,ControllerFcn), describes a set of event
%            functions 'e', that trigger the corresponding event if a
%            zero-crossing is detected in positive direction.  
% - [qPLUS,dqdtPLUS,zPLUS,endsStride] = JumpMap(t,qMINUS,dqdtMINUS,zMINUS,p,ControllerFcn,event), 
%            describes a set of event handlers 'g' that define the
%            instantenous changes of the states during a specific event.
%            If the event iends the stride (endsStride==true), the
%            simulation is aborted after this event. 
% - ddqddt = FlowMap(t,q,dqdt,z,p,ControllerFcn), describes the continuous
%            dynamics of the hybrid system by providing the system
%            accelerations.
%
% If the system is passive, the result of the simulation only depends on
% the initial continuous states 'qIN' and 'dqdtIN', the initial discrete 
% states 'zIN', and a vector of parameters 'p'.  
%
% An active system depends additionally on a controller function provided
% by the handle 'options.controller' and defined as follows:
% --  u = Controller(t, q, dqdt, z, p), 
% which returns the actuation input vector u for every step of the 
% simulation.
%
% HybridDynamics returns trajectories of the system until either the first
% terminal event occurs or a specified maximum simulation time is reached.
%
% Additionally, the function can be provided with an output object
% 'options.graphOUTPUT' for graphical display which must be derived from 
% the class 'OutputCLASS'. The graphics are updated throughout the
% simulation creating an animation.
%
% INPUT:    qIN     -- vector of initial positions of the system;
%           dqdtIN  -- vector of initial velocities of the system;
%           zIN     -- vector of initial discrete states of the system;
%           p       -- vector of the system parameters;
%           options -- a structure providing additional simulation options;
%               Possible options are:
%               - options.tSTART        -- simulation initial time;
%               - options.tMAX          -- maximum simulation time;
%               - options.ControllerFcn -- handle to a controller function;
%               - options.graphOUTPUT   -- graphical output object which 
%                 must be derived from the class 'OutputCLASS'; leave it 
%                 out or set to [] for no animation;
%               - options.dtGraphOUT    -- time step of the animation;
%               - options.dtOUT         -- time step of the numerical 
%                 output (it does not affect the integration time step);
%               - options.odeOPTIONS    -- any ode45 options that would
%                 override the default integration settings.
% 
% OUTPUT:   tOUT    -- time points of the resulting system trajectory;
%           qOUT    -- trajectories of the position states;
%           dqdtOUT -- trajectories of the velocity states;
%           zOUT    -- trajectories of the discrete states;
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
%   See also FLOWMAP, JUMPMAP, JUMPSET

function [tOUT, qOUT, dqdtOUT, zOUT] = HybridDynamics(qIN, dqdtIN, zIN, p, options)

    % *********************************************************************
    % INPUT HANDLING
    % Process provided options to get all necessary simulation parameter
    % values or assign default values:
    [tIN,tMAX,ControllerFcn,graphOUTPUT,dtOUT,odeOPTIONS,simulateSingleStride] = handleOptions(options);
    
    % Set up the options for output and event detection (do this last, as 
    % these should never be overwritten by the user):
    odeOPTIONS = odeset(odeOPTIONS,'Events',@Events,'OutputFcn',@OutputFcn);
    % END INPUT HANDLING
    % *********************************************************************
        
    % *********************************************************************
    % SIMULATE UNTIL EITHER TERMINAL EVENT OR MAXIMUM TIME
    
    % Initialize output:
    tOUT    = [];
    qOUT    = [];
    dqdtOUT = [];
    zOUT    = [];
                     
    % Start integration:
    endsStride = false;
    % Initial states:
    t0 = tIN;
    y0 = [qIN; dqdtIN];
    z  = zIN;
    
    %   Start clock for the timing of the output function
    if ~isempty(graphOUTPUT)
        tspanGraph = graphOUTPUT.getTimeVector(t0,tMAX);
        frameGraph = 1;
        tic
    end
    while ~(endsStride && simulateSingleStride)
        % Integrate until the next event, maximally until time tMAX:
        if isempty(dtOUT)
            tspan = [t0,tMAX];
        else
            tspan = t0:dtOUT:tMAX;
            tspan = [tspan(diff(tspan)~=0), tspan(end)];
        end
        % NOTE: even though ode45 is provided with an initial state as a
        % column vector, the results are stored in rows.
        [~,~,te,ye,ie] = ode45(@ODE,tspan,y0,odeOPTIONS);
        
        if isempty(ie)        
            % No event occurred. The simulation ran out of time without
            % reaching a terminal event.
            break;
        else
            % Handle the discrete change of states at events by calling the
            % jump map (which must be on the MATLAB search path): 
            qe    = ye(end,1:end/2)';
            dqdte = ye(end,end/2+1:end)';
            [q0, dqdt0, z, endsStride] = JumpMap(te(end), qe, dqdte, z, p, ControllerFcn, ie(end));
            t0 = te(end);
            y0 = [q0; dqdt0];
            % Display the result of the discrete changes:
            OutputFcn(t0,y0,[]);
        end    
    end
    % DONE SIMULATING
    % *********************************************************************
    
    
    
    % *********************************************************************
    % Event Detection   
    function [value_,isterminal_,direction_] = Events(t, y_)
        q    = y_(1:end/2);
        dqdt = y_(end/2+1:end);
        % Get values of the event function by calling the jump set function
        % (which must be on the MATLAB search path):
        value_ = JumpSet(t, q, dqdt, z, p, ControllerFcn);
        n_events_ = length(value_);
        isterminal_ = ones(n_events_,1); % All events are terminal for the ode integration
                                         % (but not necessarily for the overall simulation)
        direction_  = ones(n_events_,1); % All events require a positive derivative
    end
    % End Event Detection
    % *********************************************************************
    
    % *********************************************************************
    % ODE of the continuous dynamics
    function dydt_ = ODE(t, y_)
        % Extract positions and velocities:
        q    = y_(1:end/2);
        dqdt = y_(end/2+1:end);
        % Get continuous derivatives, by calling the flow map function
        % (which must be on the MATLAB search path):
        ddqddt = FlowMap(t, q, dqdt, z, p, ControllerFcn);
        % Assemble into a single vector for the integrator:
        dydt_ = [dqdt; ddqddt];
    end
    % End ODE
    % *********************************************************************
    
    % *********************************************************************
    % Call the update function for the current state
    function status_ = OutputFcn(t_,y_,plot_flag_)
        % Extract positions and velocities:
        q_    = y_(1:end/2,:);
        dqdt_ = y_(end/2+1:end,:);
        if isempty(plot_flag_)
            % Update the graphics:
            if ~isempty(graphOUTPUT)
                for i = 1:length(t_)
                    % If graphics output has not reached the last requested
                    % time frame and if simulation has reached the next
                    % graphics time frame:
                    if frameGraph<length(tspanGraph) && t_(i)>=tspanGraph(frameGraph+1)
                        u = ControllerFcn(t_(i), q_(:,i), dqdt_(:,i), z, p);
                        graphOUTPUT = update(graphOUTPUT, q_(:,i), z, t_(i), u);
                        frameGraph = frameGraph + 1;
                    end
                end
            end
            % Add the new points to the numerical output:
            tOUT    = [tOUT, t_];
            qOUT    = [qOUT, q_];
            dqdtOUT = [dqdtOUT, dqdt_];
            zOUT    = [zOUT, repmat(z,1,length(t_))];
        elseif strcmp(plot_flag_,'init') %First step:
            % Update the graphics:
            if ~isempty(graphOUTPUT)
                u = ControllerFcn(t_(1), q_(:), dqdt_(:), z, p);
                graphOUTPUT = update(graphOUTPUT, q_(:), z, t_(1), u);
            end
            % Add the new points to the output:
            tOUT    = [tOUT, t_(1)];
            qOUT    = [qOUT, q_];
            dqdtOUT = [dqdtOUT, dqdt_];
            zOUT    = [zOUT, z];
        end
        status_ = 0; % keep integrating
    end
    % End output
    % *********************************************************************
end
% *************************************************************************
% *************************************************************************


% *********************************************************************
function [tIN,tMAX,ControllerFcn,graphOUTPUT,dtOUT,odeOPTIONS,simulateSingleStride] = handleOptions(options)
% Handle all optional inputs to the simulation. For each option, check if 
% it has been provided and, if not, set it to a default value:
    % Initial simulation time:
    if isfield(options,'tSTART')
        tIN = options.tSTART;
    else
        tIN = 0;
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
        % If no controller is provided, use the NoControl controller
        % which must be on the MATLAB search path:
        path(path, genpath('Controllers\NoControl'));
        ControllerFcn = @Controller_NoControl;
    end
    % Graphics output class:
    if isfield(options,'graphOUTPUT') && isa(options.graphOUTPUT,'OutputCLASS')
        graphOUTPUT = options.graphOUTPUT;
    else
        % No graphics by default:
        graphOUTPUT = [];
    end
    % Graphical output time step:
    if ~isempty(graphOUTPUT) && isfield(options,'dtGraphOUT') && ~isempty(options.dtGraphOUT)
        graphOUTPUT.rate = options.dtGraphOUT;
    else
        % By default, update rate provided in the graphics class is used
    end
    % Numerical output time step:
    if isfield(options,'dtOUT')
        dtOUT = options.dtOUT;
    else
        % Default ode45 output time points
        dtOUT = [];
    end
    % ode options:
    if isfield(options,'odeOPTIONS')
        odeOPTIONS = options.odeOPTIONS;
    else
        odeOPTIONS = odeset('RelTol',1e-6,...
                            'AbsTol',1e-12,...
                            'MaxStep',0.01);
    end
    % Simulate continuously (until tMAX) or just a single stride:
    if isfield(options,'simulateSingleStride')
        simulateSingleStride = options.simulateSingleStride;
    else
        simulateSingleStride = false;
    end
end
% *********************************************************************