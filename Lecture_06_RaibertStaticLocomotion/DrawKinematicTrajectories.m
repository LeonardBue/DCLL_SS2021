% This function plots out desired trajectories of the 7-link biped's feet
% and main body, both against time and in the plane of motion. The desired 
% trajectories are defined in 'ComputeKinematicTrajectories.m'.
% 
function DrawKinematicTrajectories()
    n = 1000;   % number of discretization points in the desired trajectory
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

    % Plot the trajectories as a function of time:
    figure('Name','Foot-center trajectories (in x- and y-direction) as a function of time','Units','Normalized','OuterPosition',[0.0,0.5,0.5,0.5]);
    clf; hold on; box on;
    plot(t,xF_l,'r-');
    plot(t,yF_l,'r:');
    plot(t,xF_r,'b-');
    plot(t,yF_r,'b:');
    plot(t,xM,'g-');
    plot(t,yM,'g:');
    supportMax = max(xF_l+0.25/2-abs(yF_l*1e24), xF_r+0.25/2-abs(yF_r*1e24));
    supportMin = min(xF_l-0.25/2+abs(yF_l*1e24), xF_r-0.25/2+abs(yF_r*1e24));
    plot(t,[supportMax;supportMin],'k--');
    legend('xFoot_l','yFoot_l','xFoot_r','yFoot_r','xMB','yMB','Edge of support','Edge of support','Location','NorthWest');

    % Plot the trajectories in x-y space:
    figure('Name','Foot-center trajectories in x-y space','Units','Normalized','OuterPosition',[0.0,0.0,0.5,0.5]);
    clf; hold on; box on;
    plot(xF_l,yF_l,'r');
    plot(xF_r,yF_r,'b');
    plot(xM,yM);
    axis equal
    legend('Foot_l','Foot_rl','MB');
end