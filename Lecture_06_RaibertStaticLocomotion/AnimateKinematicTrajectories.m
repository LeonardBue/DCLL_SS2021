% For given desired trajectories of the 7-link biped's feet and main body,
% this function uses inverse kinematics to compute the joint trajectories. 
% It then shows animation of the desired motion. The desired feet and MB
% trajectories are defined in 'ComputeKinematicTrajectories.m'.
% 
function AnimateKinematicTrajectories()
    % Pre output:
    fig = figure('Name','Animation of the kinematic motion','Units','Normalized','OuterPosition',[0.5,0.0,0.5,1.0]);
        
    n = 200;    % number of discretization points in the desired trajectory
    tMax = 25;  % maximum simulation time

    % Initialize trajectory vectors:
    t = linspace(0,tMax,n);
    xF_l = zeros(1,n);
    yF_l = zeros(1,n);
    xF_r = zeros(1,n);
    yF_r = zeros(1,n);
    xM = zeros(1,n);
    yM = zeros(1,n);

    % Compute the feet and main body trajectories:
    for i = 1:n
        [xF_l(i), yF_l(i), xF_r(i), yF_r(i), xM(i), yM(i)] = ComputeKinematicTrajectories(t(i));
    end

    % Show an animation:
    for i = 1:size(t,2)
        % Do IK:
        x = xM(i);
        y = yM(i);
        phi = 0;
        betaL  = -acos(((xF_l(i)-xM(i)-0.25/2)^2 + (yF_l(i)-yM(i))^2-0.5^2-0.5^2)/(2*0.5*0.5));
        alphaL = -atan((xF_l(i)-xM(i)-0.25/2)/(yF_l(i)-yM(i))) - atan((0.5*sin(betaL))/(0.5+0.5*cos(betaL)));
        gammaL = -alphaL - betaL + pi/2;
        betaR  = -acos(((xF_r(i)-xM(i)-0.25/2)^2 + (yF_r(i)-yM(i))^2-0.5^2-0.5^2)/(2*0.5*0.5));
        alphaR = -atan((xF_r(i)-xM(i)-0.25/2)/(yF_r(i)-yM(i))) - atan((0.5*sin(betaR))/(0.5+0.5*cos(betaR)));
        gammaR = -alphaR - betaR + pi/2;

        % Update the figure:
        figure(fig);
        clf;
        hold on;
        xs = [x; x+0.5*sin(phi + alphaL); x+0.5*sin(phi + alphaL)+0.5*sin(phi + alphaL + betaL); x+0.5*sin(phi + alphaL)+0.5*sin(phi + alphaL + betaL)+0.25*sin(phi + alphaL + betaL+gammaL)];
        ys = [y; y-0.5*cos(phi + alphaL); y-0.5*cos(phi + alphaL)-0.5*cos(phi + alphaL + betaL); y-0.5*cos(phi + alphaL)-0.5*cos(phi + alphaL + betaL)-0.25*cos(phi + alphaL + betaL+gammaL)];
        plot(xs,ys,'r');
        xs = [x; x+0.5*sin(phi + alphaR); x+0.5*sin(phi + alphaR)+0.5*sin(phi + alphaR + betaR); x+0.5*sin(phi + alphaR)+0.5*sin(phi + alphaR + betaR)+0.25*sin(phi + alphaR + betaR+gammaR)];
        ys = [y; y-0.5*cos(phi + alphaR); y-0.5*cos(phi + alphaR)-0.5*cos(phi + alphaR + betaR); y-0.5*cos(phi + alphaR)-0.5*cos(phi + alphaR + betaR)-0.25*cos(phi + alphaR + betaR+gammaR)];
        plot(xs,ys,'b');
        axis equal
        axis([min(xM)-1,max(xM)+1,-0.5,1.5]);
        drawnow();
    end
end