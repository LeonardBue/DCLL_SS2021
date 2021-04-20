% *************************************************************************
%
% Script "Symbolic computation of the equations of motion"
%
% This code is for a floating based model of the Simplest Walking Model
%
% The script auto-generates MATLAB functions to describe the kinematics and
% dynamics. 
%
% INPUT:  - NONE
% OUTPUT: - NONE
%
%   C. David Remy remy@inm.uni-stuttgart.de
%   Matlab R2018
%   (uses the symbolic math toolbox)
%   4/22/2020
%   v21
%
% Based on the paper:
%  'A MATLAB Framework For Gait Creation', 2011, C. David Remy, Keith
%  Buffinton, and Roland Siegwart,  International Conference on
%  Intelligent Robots and Systems, September 25-30, San Francisco, USA 
%
%   See also FLOWMAP, JUMPMAP, JUMPSET


%% Definitions
% Generalized coordinates ...
syms theta phi real
q    = [theta phi].';
% ... generalized speeds ...
syms dtheta dphi real
dqdt = [dtheta dphi].';
% ... and parameters:
syms g gamma real    % gravity and slope angle
syms l real          % leg length
syms m1 m2 real      % masses of the hip (MB) and each foot
syms beta real       % mass ratio m2/m1

% Gravity vector in the slope-based frame (axes are parallel and 
% perpendicular to the slope):
gx =  g*sin(gamma);
gy = -g*cos(gamma);
% Foot mass:
m2 = beta*m1;


%% DYNAMICS (obtained via the Euler-Lagrange equation)

% CoG-positions (from kinematics):
CoG_FootR = [0;
             0];
CoG_MB   = [0 - l*sin(theta);
            0 + l*cos(theta)];
CoG_LegL = [-l*sin(theta) + l*sin(theta - phi);
            +l*cos(theta) - l*cos(theta - phi)];
          
% CoG-velocities (computed via jacobians):
d_CoG_MB   = jacobian(CoG_MB,q)*dqdt;
d_CoG_LegR = jacobian(CoG_FootR,q)*dqdt;
d_CoG_LegL = jacobian(CoG_LegL,q)*dqdt;

% Potential Energy (due to gravity):
V = -CoG_MB(1)*m1*gx - CoG_FootR(1)*m2*gx - CoG_LegL(1)*m2*gx + ...
    -CoG_MB(2)*m1*gy - CoG_FootR(2)*m2*gy - CoG_LegL(2)*m2*gy;
V = simplify(expand(V));

% Kinetic Energy:         
T = 0.5 * (m1 * sum(d_CoG_MB.^2) + ...
           m2 * sum(d_CoG_LegR.^2) + ...
           m2 * sum(d_CoG_LegL.^2));
T = simplify(expand(T));

% Lagrangian:
L = T-V;
% Partial derivatives:
dTdq   = jacobian(T,q).';
dVdq   = jacobian(V,q).';
dLdqdt = jacobian(L,dqdt).';
      
% Compute Mass Matrix:
M = jacobian(dLdqdt,dqdt);
M = simplify(M);

% Compute the coriolis and gravitational forces:
dL_dqdt_dt = jacobian(dLdqdt,q)*dqdt;
f_c = simplify(+dTdq - dL_dqdt_dt);
f_g = simplify(-dVdq);

% The equations of motion are given with these functions as:   
% M * dqddt = f_c(q, dqdt) + f_g(q);
disp('Raw EOM components:')
disp('M:')
disp(simplify(expand(M)));
disp('f_c:')
disp(simplify(expand(f_c)));
disp('f_g:')
disp(simplify(expand(f_g)));

% In the paper, these equations are written as
% M * dqddt + -f_c(q, dqdt) + -f_g(q) = 0;
% Furthermore, they are simplified by dividing everything by m1*l^2.
% And finally, the second line has been multiplied with (-1).  (I
% personally find that last operation annoying, as it leads to a
% non-symmetrical mass matrix).
% Either way, the terms are:
disp('EOMs as shown in the paper:')
disp(+1*[1,0;0,-1]*simplify(M./(l^2*m1)));
disp('ddq');
disp('+');
disp(-1*[1,0;0,-1]*simplify(f_c./(l^2*m1)));
disp('+');
disp(-1*[1,0;0,-1]*simplify(f_g./(l^2*m1)));
disp('=');
disp([0;0]);

