% *************************************************************************
% classdef SLIP_Model_Graphics(p) < OutputCLASS
%
% Two-dimensional graphics of a SLIP model.
%
% The graphics object must be initialized with the vector of system
% parameters p.
%
% Created by C. David Remy on 07/10/2011; updated on Sept 6, 2019
% MATLAB 2018a
%
% Documentation:
%  'A MATLAB Framework For Gait Creation', 2011, C. David Remy, Keith
%  Buffinton, and Roland Siegwart,  International Conference on
%  Intelligent Robots and Systems, September 25-30, San Francisco, USA 
%
classdef SLIP_Model_Graphics < OutputCLASS 
    % Private attributes:
    properties (SetAccess = 'private', GetAccess = 'private')
        fig; % The output window
        ax;  % The output axis
        % The parameter vector:
        p;
        % Patch and line objects used in the graphical representation
        COGPatch;
        SpringLine;
    end
    % Public methods:
    methods
        % Constructor:
        function obj = SLIP_Model_Graphics(q0, p)
            obj.slowDown = 1; % Run this in real time.
            obj.rate     = 0.04;   % with 25 fps
            
            % Copy the parameter vector:
            obj.p = p;
            
            % Initialize the graphics
            obj.fig = figure();
            clf(obj.fig);
            obj.ax = axes;
            % Set some window properties
            set(obj.fig,'Name','2D-Output of a SLIP model');  % Window title
            set(obj.fig,'Color','w');         % Background color

            % CoG positions and system parameters (the order of elements in
            % q0 and p is defined in the functions 'contStates.m' and
            % 'systParam.m'):
            x = q0(1);
            y = q0(2);
            l_0    = p(2);
            angAtt = p(5);
            
            % Create graphic objects:
                % The representation of the COG as patch object:
                phi = linspace(0, pi/2, 10);
                vert_x = [0,sin(phi)*0.1,0];
                vert_x = [vert_x;vert_x;-vert_x;-vert_x]' + x;
                vert_y = [0,cos(phi)*0.1,0];
                vert_y = [vert_y;-vert_y;-vert_y;vert_y]' + y;
                obj.COGPatch = patch(vert_x, vert_y, cat(3,[1 0 1 0], [1 0 1 0],[1 0 1 0]));

                % The representation of the springy leg as a line object:
                comp = l_0 - 1;
                vert_x = zeros(1,37);
                vert_x([3 5 7  9 11 13 15]) = - 0.05;
                vert_x([4 6 8 10 12 14 16]) = + 0.05;
                vert_y = [0,linspace(-0.2,-0.8-comp,16),-1-comp*ones(1,20)];
                phi = linspace(0,2*pi,20);
                vert_x(18:37) =  0        + sin(phi)*0.02;
                vert_y(18:37) = -1 - comp + cos(phi)*0.02;
                % Rotate and shift:
                T = [ cos(angAtt),-sin(angAtt);
                      sin(angAtt), cos(angAtt)];
                vert_rot = T*[vert_x;vert_y];
                vert_x = vert_rot(1,:) + x;
                vert_y = vert_rot(2,:) + y;
                obj.SpringLine = line(vert_x,vert_y, 'color','k', 'LineWidth',2);
            
            % Draw the ground. It reaches from -0.5 to +8.5.
            h   = 0.01; % Height of the bar at the top
            n   = 180;  % Number of diagonal stripes in the shaded area
            s   = 0.05;  % Spacing of the stripes
            w   = 0.01; % Width of the stripes
            ext = 0.1;  % Length of the stripes
            % Create vertices by shifting a predefined pattern 'n' times to the
            % right:
            v = [-0.5,0;
                 repmat([0,-h],n,1) + [-0.5+linspace(0,s*n,n)',zeros(n,1)];
                 repmat([-ext,-ext-h],n,1) + [-0.5+linspace(0,s*n,n)',zeros(n,1)];
                 repmat([-ext+w,-ext-h],n,1) + [-0.5+linspace(0,s*n,n)',zeros(n,1)];
                 repmat([w,0-h],n,1) + [-0.5+linspace(0,s*n,n)',zeros(n,1)];
                 -0.5+s*n+w,0];
            % Connect to faces:
            f = [1,2,4*n+1,4*n+2;
                 repmat([0,n,2*n,3*n],n,1) + repmat((1:n)',1,4)+1];
            % Color is uniformly black
            patch('faces', f, 'vertices', v);
            
            % Set up view:
            box on
            grid on
            axis equal
            axis([-0.5,8.5,-0.5,2.5])
        end
        % Updated function.  Is called by the integrator:
        function obj = update(obj, q, z, ~, u)
            % extract states and parameters:
            x = q(1);
            y = q(2);
            phase  = z(1);
            contPt = z(2);
            l_0 = obj.p(2);
            % The angle-of-attack is the only control in SLIP (for passive
            % SLIP, the control u is equal to the angle-of-attack specified
            % in the model parameters, obj.p(5)):
            angAtt = u;
            
            % Evaluate states:
            % Leg configuration:
            switch phase
                case {0,2} %(flight)
                    % leg is uncompressed and leg angle = angle of attack
                    l_leg = l_0;
                    gamma_leg = angAtt;
                case 1 %(stance)
                    % Compute the leg length and leg angle
                    l_leg = sqrt((x-contPt)^2 + (y-0)^2);
                    gamma_leg = atan2(contPt-x, y-0);
            end
            
             % COG:
            phi = linspace(0, pi/2, 10);
            vert_x = [0,sin(phi)*0.1,0];
            vert_x = [vert_x;vert_x;-vert_x;-vert_x]' + x;
            vert_y = [0,cos(phi)*0.1,0];
            vert_y = [vert_y;-vert_y;-vert_y;vert_y]' + y;
            set(obj.COGPatch,'xData', vert_x, 'yData',vert_y);
            
            % Springy Leg
            comp = l_leg - 1;
            vert_x = zeros(1,37);
            vert_x([3 5 7  9 11 13 15]) = - 0.05;
            vert_x([4 6 8 10 12 14 16]) = + 0.05;
            vert_y = [0,linspace(-0.2,-0.8-comp,16),-1-comp*ones(1,20)];
            phi = linspace(0,2*pi,20);
            vert_x(18:37) =  0        + sin(phi)*0.02;
            vert_y(18:37) = -1 - comp + cos(phi)*0.02;
            % Rotate and shift:
            T = [ cos(gamma_leg),-sin(gamma_leg);
                  sin(gamma_leg), cos(gamma_leg)];
            vert_rot = T*[vert_x;vert_y];
            vert_x = vert_rot(1,:) + x;
            vert_y = vert_rot(2,:) + y;
            set(obj.SpringLine,'xData', vert_x, 'yData',vert_y);
            axis(obj.ax, [-0.5,8.5,-0.5,2.5]);
            drawnow();
        end
    end
end