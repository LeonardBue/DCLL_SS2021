% This function defines the motion profile (desired trajectories) for the
% feet and main body of the 7-link biped in 2D.
% 
% INPUT:    t    -- time instant at which desired positions are to be 
%                   computed;
% OUTPUT:   xF_l -- desired horizontal location of the left foot;
%           yF_l -- desired vertical location of the left foot;
%           xF_r -- desired horizontal location of the right foot;
%           yF_r -- desired vertical location of the right foot;
%           xM   -- desired horizontal location of the main body CoM;
%           yM   -- desired vertical location of the main body CoM.
% 
function [xF_l, yF_l, xF_r, yF_r, xM, yM] = ComputeKinematicTrajectories(t)

    % Parameters (ZMP):
    T    = 5;    % Stride time
    beta = 0.6;  % Duty factor
    hSt  = 0.2;  % Stride height
    lSt  = 1.0;  % Stride length
    hHip = 0.8;  % Hip height 
	
    % Motions are defined piecewise, either as cubic splines or constant
    % values. 
    
    % These parameters form a cubic spline that will bring the foot
    % position from 0 to lSt as the normalized stride time goes from beta
    % to 1.
    % Compute spline parameters:
    AxF = [  T^3*beta^3, T^2*beta^2, T*beta, 1;
           3*T^2*beta^2,   2*T*beta,      1, 0;
                T^3*1^3,    T^2*1^2,    T*1, 1; 
              3*T^2*1^2,      2*T*1,      1, 0];
    bxF = AxF\[0;0;lSt;0]; 
    % For the y motion, two cubic splines move the leg up and down again as
    % the normalized stride time goes from beta to (1+beta)/2 and from
    % (1+beta)/2 to 1:
    % Compute spline parameters:
    AyF1 = [          T^3*beta^3,         T^2*beta^2,         T*beta, 1;
                    3*T^2*beta^2,           2*T*beta,              1, 0;
              T^3*((beta+1)/2)^3, T^2*((beta+1)/2)^2, T*((beta+1)/2), 1; 
            3*T^2*((beta+1)/2)^2,   2*T*((beta+1)/2),              1, 0];
    byF1 = AyF1\[0;0;hSt;0]; 
    AyF2 = [  T^3*((beta+1)/2)^3, T^2*((beta+1)/2)^2, T*((beta+1)/2), 1; 
            3*T^2*((beta+1)/2)^2,   2*T*((beta+1)/2),              1, 0;
                         T^3*1^3,            T^2*1^2,            T*1, 1; 
                       3*T^2*1^2,              2*T*1,              1, 0];
    byF2 = AyF2\[hSt;0;0;0]; 
    % The main body is moved twice per stride, in the phase when both legs
    % are on the ground, which last from
    % 0 to beta-0.5 and 0.5 to beta
    AxM1 = [                 0,                0,            0, 1; 
                             0,                0,            1, 0;
              T^3*(beta-0.5)^3, T^2*(beta-0.5)^2, T*(beta-0.5), 1; 
            3*T^2*(beta-0.5)^2,   2*T*(beta-0.5),            1, 0];
    bxM1 = AxM1\[-lSt/2;0;0;0]; 
    AxM2 = [  T^3*(1/2)^3, T^2*(1/2)^2, T*(1/2), 1; 
            3*T^2*(1/2)^2,   2*T*(1/2),       1, 0;
               T^3*beta^3,  T^2*beta^2,  T*beta, 1;
             3*T^2*beta^2,    2*T*beta,       1, 0];
    bxM2 = AxM2\[0;0;lSt/2;0]; 
    
    % Compute current stride and normalized stride time for each leg:
    n_l = floor(t/T);       % Stride number
    s_l = mod(t,T)/T;       % Normalize time to stride time,
    n_r = floor((t+T/2)/T); % Stride number
    s_r = mod(t+T/2,T)/T;   % Normalize time to stride time, right leg is half a stride ahead
    
    % left foot:
    if s_l<beta 
        % foot on the ground:
        xF_l = n_l*lSt;
        yF_l = 0;
    else
        % cubic spline for x:
        xF_l = n_l*lSt + bxF(1)*T^3*s_l^3 + bxF(2)*T^2*s_l^2 + bxF(3)*T*s_l + bxF(4);
        % two cubic splines for y:
        if s_l<(beta+1)/2
            yF_l = byF1(1)*T^3*s_l^3 + byF1(2)*T^2*s_l^2 + byF1(3)*T*s_l^1 +byF1(4)*s_l^0;
        else
            yF_l = byF2(1)*T^3*s_l^3 + byF2(2)*T^2*s_l^2 + byF2(3)*T*s_l^1 +byF2(4)*s_l^0;
        end
    end
    
    % right foot:
    if s_r<beta 
        % foot on the ground:
        xF_r = (n_r-0.5)*lSt;
        yF_r = 0;
    else
        % cubic spline for x:
        xF_r = (n_r-0.5)*lSt + bxF(1)*T^3*s_r^3 + bxF(2)*T^2*s_r^2 + bxF(3)*T*s_r + bxF(4);
        % two cubic splines for y:
        if s_r<(beta+1)/2
            yF_r = byF1(1)*T^3*s_r^3 + byF1(2)*T^2*s_r^2 + byF1(3)*T*s_r^1 +byF1(4)*s_r^0;
        else
            yF_r = byF2(1)*T^3*s_r^3 + byF2(2)*T^2*s_r^2 + byF2(3)*T*s_r^1 +byF2(4)*s_r^0;
        end
    end
        
    % Main body:
    if s_l<(beta-0.5) 
        xM = n_l*lSt + bxM1(1)*T^3*s_l^3 + bxM1(2)*T^2*s_l^2 + bxM1(3)*T*s_l + bxM1(4);
        yM = hHip;
    elseif s_l<0.5
        xM = n_l*lSt;
        yM = hHip;
    elseif s_l<beta
        xM = n_l*lSt + bxM2(1)*T^3*s_l^3 + bxM2(2)*T^2*s_l^2 + bxM2(3)*T*s_l + bxM2(4);
        yM = hHip;
    else
        xM = (n_l+0.5)*lSt;
        yM = hHip;
    end
end

