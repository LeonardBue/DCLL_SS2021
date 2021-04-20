% This function plots out desired trajectories of the 7-link biped's feet
% and main body, both against time and in the plane of motion. The desired 
% trajectories are defined in 'ComputeKinematicTrajectories.m'.
% 
function DrawKinematicTrajectories()
    [t, xF_l, yF_l, xF_r, yF_r, xCoG, yCoG, Zref, Z] = PlanZMPTrajectories();
    xM = xCoG;
    yM = yCoG;

    % Plot the trajectories as a function of time:
    figure('Name','Foot-center trajectories (in x- and y-direction) as a function of time','Units','Normalized','OuterPosition',[0.0,0.5,0.5,0.5]);
    clf; hold on; box on;
    plot(t,xF_l,'r-');
    plot(t,yF_l,'r:');
    plot(t,xF_r,'b-');
    plot(t,yF_r,'b:');
    plot(t,xM,'g-');
    plot(t,yM,'g:');
    plot(t,Zref,'c-');
    plot(t,Z,'m-');
    supportMax = max(xF_l+0.25/2-abs(yF_l*1e24), xF_r+0.25/2-abs(yF_r*1e24));
    supportMin = min(xF_l-0.25/2+abs(yF_l*1e24), xF_r-0.25/2+abs(yF_r*1e24));
    plot(t,[supportMax;supportMin],'k--');
    legend('xFoot_l','yFoot_l','xFoot_r','yFoot_r','xMB','yMB','xZMP_{des}','xZMP','Edge of support','Edge of support','Location','NorthWest');

    % Plot the trajectories in x-y space:
    figure('Name','Foot-center trajectories in x-y space','Units','Normalized','OuterPosition',[0.0,0.0,0.5,0.5]);
    clf; hold on; box on;
    plot(xF_l,yF_l,'r');
    plot(xF_r,yF_r,'b');
    plot(xM,yM);
    axis equal
    legend('Foot_l','Foot_rl','MB');
end