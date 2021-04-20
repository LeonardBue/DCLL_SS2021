% *************************************************************************
%
% Script "Symbolic computation of the equations of motion"
%
% This code is for a floating based model of a 7-link walker
%
% The script auto-generates MATLAB functions to describe the kinematics and
% dynamics. 
%
% INPUT:  - NONE
% OUTPUT: - NONE
% FILES:  
%         (for continuous dynamics)
%         - function M             = MassMatrix(q, p)
%         - function f_cg          = F_CoriGrav(q, dqdt, p)
%         (for time stepping)
%         - dN = ContactDistanceN(q, p)
%         - JT = ContactJacobianT(q, p)
%         - JN = ContactJacobianN(q, p)
%         (for graphics)
%         - function CoGs          = CoGPositions(q, p)
%         - function links         = LinkPositions(q, p)
%         - function footPts       = FootPtPositions(q, p)
%
% where
% q = [x y phi alphaL alphaR betaL betaR gammaL gammaR].'
% is the vector of the generalized coordinates,
% dqdt = [dx dy dphi dalphaL dalphaR dbetaL dbetaR dgammaL dgammaR].'
% is the vector of generalized speeds, and
% p = [g l1 l2 l3 l4 m1 m2 m3 m4 j1 j2 j3 j4].'
% is a vector of system parameters.
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


%% Definitions
% Generalized coordinates ...
syms x y phi alphaL alphaR betaL betaR gammaL gammaR real
q    = [x y phi alphaL alphaR betaL betaR gammaL gammaR].';
% ... generalized speeds ...
syms dx dy dphi dalphaL dalphaR dbetaL dbetaR dgammaL dgammaR real
dqdt = [dx dy dphi dalphaL dalphaR dbetaL dbetaR dgammaL dgammaR].';
% ... and parameters:
%   Gravity
syms g real
%   Segment dimensions:
syms l1 l2 l3 l4 real
%   Masses/Inertia:
syms m1 m2 m3 m4 real
syms j1 j2 j3 j4 real
p = [g l1 l2 l3 l4 m1 m2 m3 m4 j1 j2 j3 j4].';


%% DYNAMICS (obtained via the Euler-Lagrange equation)
% CoG-orientations (from kinematics):
CoG_MB_ang      = phi;
CoG_ThighL_ang  = phi + alphaL;
CoG_ThighR_ang  = phi + alphaR;
CoG_ShankL_ang  = phi + alphaL + betaL;
CoG_ShankR_ang  = phi + alphaR + betaR;
CoG_FootL_ang  = phi + alphaL + betaL + gammaL;
CoG_FootR_ang  = phi + alphaR + betaR + gammaR;
% CoG-positions (from kinematics):
CoG_MB     = [x - 0.5*l1*sin(phi);
              y + 0.5*l1*cos(phi)];
CoG_ThighL = [x + 0.5*l2*sin(phi + alphaL);
              y - 0.5*l2*cos(phi + alphaL)];
CoG_ThighR = [x + 0.5*l2*sin(phi + alphaR);
              y - 0.5*l2*cos(phi + alphaR)];
CoG_ShankL = [x + l2*sin(phi + alphaL) + 0.5*l3*sin(phi + alphaL + betaL);
              y - l2*cos(phi + alphaL) - 0.5*l3*cos(phi + alphaL + betaL)];
CoG_ShankR = [x + l2*sin(phi + alphaR) + 0.5*l3*sin(phi + alphaR + betaR);
              y - l2*cos(phi + alphaR) - 0.5*l3*cos(phi + alphaR + betaR)];
CoG_FootL  = [x + l2*sin(phi + alphaL) + l3*sin(phi + alphaL + betaL) + 0.5*l4*sin(phi + alphaL + betaL + gammaL);
              y - l2*cos(phi + alphaL) - l3*cos(phi + alphaL + betaL) - 0.5*l4*cos(phi + alphaL + betaL + gammaL)];
CoG_FootR  = [x + l2*sin(phi + alphaR) + l3*sin(phi + alphaR + betaR) + 0.5*l4*sin(phi + alphaR + betaR + gammaR);
              y - l2*cos(phi + alphaR) - l3*cos(phi + alphaR + betaR) - 0.5*l4*cos(phi + alphaR + betaR + gammaR)];
          
% CoG-velocities (computed via jacobians):
d_CoG_MB     = jacobian(CoG_MB,q)*dqdt;
d_CoG_ThighL = jacobian(CoG_ThighL,q)*dqdt;
d_CoG_ThighR = jacobian(CoG_ThighR,q)*dqdt;
d_CoG_ShankL = jacobian(CoG_ShankL,q)*dqdt;
d_CoG_ShankR = jacobian(CoG_ShankR,q)*dqdt;
d_CoG_FootL  = jacobian(CoG_FootL,q)*dqdt;
d_CoG_FootR  = jacobian(CoG_FootR,q)*dqdt;
d_CoG_MB_ang     = jacobian(CoG_MB_ang,q)*dqdt;
d_CoG_ThighL_ang = jacobian(CoG_ThighL_ang,q)*dqdt;
d_CoG_ThighR_ang = jacobian(CoG_ThighR_ang,q)*dqdt;
d_CoG_ShankL_ang = jacobian(CoG_ShankL_ang,q)*dqdt;
d_CoG_ShankR_ang = jacobian(CoG_ShankR_ang,q)*dqdt;
d_CoG_FootL_ang  = jacobian(CoG_FootL_ang,q)*dqdt;
d_CoG_FootR_ang  = jacobian(CoG_FootR_ang,q)*dqdt;

% Potential Energy (due to gravity):
V = CoG_MB(2)*m1*g + ...
    CoG_ThighL(2)*m2*g + ...
    CoG_ThighR(2)*m2*g + ...
    CoG_ShankL(2)*m3*g + ...
    CoG_ShankR(2)*m3*g + ...
    CoG_FootL(2)*m4*g + ...
    CoG_FootR(2)*m4*g;
V = simplify(expand(V));

% Kinetic Energy:         
T = 0.5 * (m1 * sum(d_CoG_MB.^2) + ...
           m2 * sum(d_CoG_ThighL.^2) + ...
           m2 * sum(d_CoG_ThighR.^2) + ...
           m3 * sum(d_CoG_ShankL.^2) + ...
           m3 * sum(d_CoG_ShankR.^2) + ...
           m4 * sum(d_CoG_FootL.^2) + ...
           m4 * sum(d_CoG_FootR.^2) + ...
           j1 * d_CoG_MB_ang^2 + ...
           j2 * d_CoG_ThighL_ang^2 + ...
           j2 * d_CoG_ThighR_ang^2 + ...
           j3 * d_CoG_ShankL_ang^2 + ...
           j3 * d_CoG_ShankR_ang^2 + ...
           j4 * d_CoG_FootL_ang^2 + ...
           j4 * d_CoG_FootR_ang^2);
T = simplify(expand(T));

% Lagrangian:
L = T-V;
% Partial derivatives:
dLdq   = jacobian(L,q).';
dLdqdt = jacobian(L,dqdt).';
      
% Compute Mass Matrix:
M = jacobian(dLdqdt,dqdt);
M = simplify(M);

% Compute the coriolis and gravitational forces:
dL_dqdt_dt = jacobian(dLdqdt,q)*dqdt;
f_cg = dLdq - dL_dqdt_dt;
f_cg = simplify(f_cg);

% The equations of motion are given with these functions as:   
% M * dqddt = f_cg(q, dqdt) + u;


%% KINEMATICS (for graphical output)
% Joint-positions:
Head  = [x - l1*sin(phi); 
         y + l1*cos(phi)];
Hip   = [x;
         y];
KneeL = [x + l2*sin(phi + alphaL);
         y - l2*cos(phi + alphaL)];
KneeR = [x + l2*sin(phi + alphaR);
         y - l2*cos(phi + alphaR)];
AnkleL = [x + l2*sin(phi + alphaL) + l3*sin(phi + alphaL + betaL);
          y - l2*cos(phi + alphaL) - l3*cos(phi + alphaL + betaL)];
AnkleR = [x + l2*sin(phi + alphaR) + l3*sin(phi + alphaR + betaR);
          y - l2*cos(phi + alphaR) - l3*cos(phi + alphaR + betaR)];
ToeL = [x + l2*sin(phi + alphaL) + l3*sin(phi + alphaL + betaL) + l4*sin(phi + alphaL + betaL + gammaL);
        y - l2*cos(phi + alphaL) - l3*cos(phi + alphaL + betaL) - l4*cos(phi + alphaL + betaL + gammaL)];
ToeR = [x + l2*sin(phi + alphaR) + l3*sin(phi + alphaR + betaR) + l4*sin(phi + alphaR + betaR + gammaR);
        y - l2*cos(phi + alphaR) - l3*cos(phi + alphaR + betaR) - l4*cos(phi + alphaR + betaR + gammaR)];

% CoGs (incl orientation of the segments):
CoGs = [CoG_MB,     CoG_ThighL,     CoG_ShankL,     CoG_FootL,     CoG_ThighR,     CoG_ShankR,     CoG_FootR;
        CoG_MB_ang, CoG_ThighL_ang, CoG_ShankL_ang, CoG_FootL_ang, CoG_ThighR_ang, CoG_ShankR_ang, CoG_FootR_ang];

% Links (or rather: joint positions)
links = [ToeL, AnkleL, KneeL, Hip, Head, Hip, KneeR, AnkleR, ToeR];

% Position of the foot points:     
footPts = [AnkleL, AnkleR, ToeL, ToeR];


%% CONTACT DYNAMICS 
% Contact points:
cont_pointLH = AnkleL;
cont_pointRH = AnkleR;
cont_pointLF = ToeL;
cont_pointRF = ToeR;
dN = [cont_pointLH(2);cont_pointRH(2);cont_pointLF(2);cont_pointRF(2)];

% Contact Jacobians:
JLH = jacobian(cont_pointLH,q);
JRH = jacobian(cont_pointRH,q);
JLF = jacobian(cont_pointLF,q);
JRF = jacobian(cont_pointRF,q);
JLH = simplify(JLH);
JRH = simplify(JRH);
JLF = simplify(JLF);
JRF = simplify(JRF);
JT = [JLH(1,:);JRH(1,:);JLF(1,:);JRF(1,:)];    % tangential
JN = [JLH(2,:);JRH(2,:);JLF(2,:);JRF(2,:)];    % normal
    

%% Create MATLAB-functions:
if ~exist('AutoGeneratedFcts','dir')
    mkdir('AutoGeneratedFcts')
end
% for continuous dynamics:
matlabFunction(M,'file','AutoGeneratedFcts\MassMatrix','vars',{q, p});
matlabFunction(f_cg,'file','AutoGeneratedFcts\F_CoriGrav','vars',{q, dqdt, p});
% for time stepping:
matlabFunction(dN,'file','AutoGeneratedFcts\ContactDistanceN','vars',{q, p});
matlabFunction(JT,'file','AutoGeneratedFcts\ContactJacobianT','vars',{q, p});
matlabFunction(JN,'file','AutoGeneratedFcts\ContactJacobianN','vars',{q, p});
% for graphics:
matlabFunction(CoGs,'file','AutoGeneratedFcts\CoGPositions','vars',{q, p});
matlabFunction(links,'file','AutoGeneratedFcts\LinkPositions','vars',{q, p});
matlabFunction(footPts,'file','AutoGeneratedFcts\FootPtPositions','vars',{q, p});