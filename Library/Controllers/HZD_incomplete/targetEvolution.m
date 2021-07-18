% This function computes desired constraints (and their 1st and 2nd
% derivatives) of the actuated joints as a function of the phase variable.
% 
% INPUT:    th -- phase variable.
% OUTPUT:   hD        -- vector of desired constraints for each actuated joint;
%           dhD_dth    -- derivative thereof wrt theta;
%           ddhD_ddth -- 2nd derivative thereof wrt theta.
%
function [hD,dhD_dth,ddhD_ddth] = targetEvolution(th)
    ChangeMe = 0;
    % Initialize output:
    hD = zeros(4,1);
    dhD_dth = zeros(4,1);
    ddhD_ddth = zeros(4,1);

    % Parameters of the desired constraints (trajectories)
    th_min      = -0.3; % Theta at which the knee bending starts
    th_max      = +0.3; % Theta at which the knee bending ends
    beta_max    = -0.9; % Maximal knee bend during swing
    beta_D      = -0.1; % Desired knee angle during stance
    beta_offset = 0.01; % Overextension of the swing knee
    phi_D       = 0;    % Desired pitch angle


    % Compute desired positions:
    % The swing knee follows a semi-circular-like trajectory (represented
    % here with cubic splines) such as to clear the ground in the first
    % half of stance and to extend to almost full length in the 2nd half of
    % stance to prepare for collision:
    %%%% CODE 1.2.1 inspect this (see above for parameter choices) %%%%
    if th<=th_min
        hD(4) = beta_offset + beta_D;
    elseif th<=0
        hD(4) = beta_offset + beta_D + 2*beta_max*th_min^(-3)*th^3 - 3*beta_max*th_min^(-2)*th^2 + 0*th + beta_max;
    elseif th<=th_max
        hD(4) = beta_offset + beta_D + 2*beta_max*th_max^(-3)*th^3 - 3*beta_max*th_max^(-2)*th^2 + 0*th + beta_max;
    else
        hD(4) = beta_offset + beta_D;
    end
    %%%% End 1.2.1

    %%%% CODE 1.2.2 complete this (use constants defined above %%%%
    % The stance knee is kept to a fixed angle of beta_D, almost straight:
    hD(1) = beta_offset + beta_D;
    % The stance hip is set such as to keep the MB pitch fixed:
    hD(2) = phi_D - th;
    % The swing hip is set such as for the swing leg to mirror the stance leg:
    hD(3) = - phi_D - th - hD(4)/2;
    %%%% End 1.2.2

    
    % Compute derivatives with respect to theta:
    %%%% CODE 1.3.1 complete this %%%%
    if th<=th_min
        dhD_dth(4) = 0;
    elseif th<=0
        dhD_dth(4) = 6*beta_max*th_min^(-3)*th^2 - 6*beta_max*th_min^(-2)*th;
    elseif th<=th_max
        dhD_dth(4) = 6*beta_max*th_max^(-3)*th^2 - 6*beta_max*th_max^(-2)*th;
    else
        dhD_dth(4) = 0;
    end
    dhD_dth(1) = 0;
    dhD_dth(2) = -1;
    dhD_dth(3) = -1 - 1/2*dhD_dth(4);

    
    % Compute second derivatives:
    if th<=th_min
        ddhD_ddth(4) = 0;
    elseif th<=0
        ddhD_ddth(4) = 12*beta_max*th_min^(-3)*th - 6*beta_max*th_min^(-2);
    elseif th<=th_max
        ddhD_ddth(4) = 12*beta_max*th_max^(-3)*th - 6*beta_max*th_max^(-2);
    else
        ddhD_ddth(4) = 0;
    end
    ddhD_ddth(1) = 0;
    ddhD_ddth(2) = 0;
    ddhD_ddth(3) = - 1/2*ddhD_ddth(4);
    %%%% End 1.3.1 %%%%
end