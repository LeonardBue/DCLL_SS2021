% This function integrates the zero dynamics (i.e. the dynamics of the
% phase variable) of the system.
% 
% INPUT:    theta0  -- initial phase variable value;
%           dtheta0 -- initial phase variable rate.
% OUTPUT:   t       -- time trajectory;
%           theta   -- phase variable trajectory;
%           dtheta  -- phase variable rate trajectory.
%
function [t, theta, dtheta_dt] = zeroDynamics(theta0, dtheta0)
    ChangeMe = 0;
    
    % Load system parameters:
    p = systParam([]);

    t0 = 0;
    tMax = 15;

    % Integrate zero dynamics:
    options = odeset('RelTol',1e-10, 'AbsTol',1e-10);
    [t,x] = ode45(@(t,x)f_ode(t,x,p), [t0,tMax], [theta0; dtheta0], options);

    % Cut off the ends which do not occur for the robot due to collisions
    th_max = 0.3;
    ind = find(abs(x(:,1))>th_max, 1);
    t = t(1:ind);
    x = x(1:ind,:);
    % Extract theta and its time derivative:
    theta     = x(:,1);
    dtheta_dt = x(:,2);



    %% Dyanamics of theta on the constraint manifold:
    function dx = f_ode(t, x, p)
        theta     = x(1);
        dtheta_dt = x(2);

        % Nominal trajectories (zero manifold):
        [hD, dhD_dth, ddhD_ddth] = targetEvolution(theta);

        %%%% CODE 3.1.1 complete this code %%%%
        % Find the map T, to recover q and dqdt from theta and hD.
        T = [-1, 1/2, 0, 0, 0;...
            zeros(4,1), eye(4)];
        q = T*[theta; hD];
        
        dq_dth = T*[1; dhD_dth];
        dq_dt  = dq_dth*dtheta_dt;
        %%%% End 3.1.1 %%%%

        % Robot dynamics matrices:
        M    = MassMatrix(q, p);
        f_cg = F_CoriGrav(q, dq_dt, p);

        % Zero dynamics matrices:
        TM    = T'*M*T;
        Tf_cg = T'*f_cg;

        %%%% CODE 3.1.2 complete below %%%%
        % Compute the acceleration of the phase variable theta
        ddtheta_ddt = (TM(1,1) + TM(1, 2:5)*dhD_dth) \ (Tf_cg(1) - TM(1, 2:5)*ddhD_ddth*dtheta_dt^2);
        %%%% End 3.1.2 %%%%

        dx = [dtheta_dt;
              ddtheta_ddt];
    end
end