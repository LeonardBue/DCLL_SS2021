% *************************************************************************
%
% Script "Symbolic computation of the equations of motion"
%
% This code is for a minimal-ccordinate model of a 5-link walker
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
%         (for implicit constraints and collisions)
%         - function contHeight    = ContactHeight(q, p)
%         - function J_cont        = ContactJacobian(q, p)
%         - function M_cont        = ContactMassMatrix(q, p)
%         (for graphics)
%         - function CoGs          = CoGPositions(q, p)
%         - function links         = LinkPositions(q, p)
%         - function footPts       = FootPtPositions(q, p)
%
% where
% q = [q1 q2 q3 q4 q5].'
% is the vector of the generalized coordinates,
% dqdt = [dq1 dq2 dq3 dq4 dq5].'
% is the vector of generalized speeds, and
% p = [g l1 l2 l3 m1 m2 m3 j1 j2 j3].'
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
%
%   See also FLOWMAP, JUMPMAP, JUMPSET


%% Definitions
% Generalized coordinates ...
syms q1 q2 q3 q4 q5 real
q    = [q1 q2 q3 q4 q5].';
% ... generalized speeds ...
syms dq1 dq2 dq3 dq4 dq5 real
dqdt = [dq1 dq2 dq3 dq4 dq5].';
% ... and parameters:
%   Gravity
syms g real
%   Segment dimensions:
syms l1 l2 l3 real
%   Masses/Inertia:
syms m1 m2 m3 real
syms j1 j2 j3 real
p = [g l1 l2 l3 m1 m2 m3 j1 j2 j3].';


%% DYNAMICS (obtained via the Euler-Lagrange equation)
% CoG-orientations (from kinematics):
CoG_MB_ang          = q1 - q2 - q3;
CoG_StanceThigh_ang = q1 - q2;
CoG_StanceShank_ang = q1;
CoG_SwingThigh_ang  = q1 - q2 - q3 + q4;
CoG_SwingShank_ang  = q1 - q2 - q3 + q4 + q5;
% CoG-positions (from kinematics):
% (the reference position is ground level at the position of contact; y is orthogonal to the ground) 
CoG_MB = [-l3*sin(CoG_StanceShank_ang) - l2*sin(CoG_StanceThigh_ang) - 0.5*l1*sin(CoG_MB_ang);
          +l3*cos(CoG_StanceShank_ang) + l2*cos(CoG_StanceThigh_ang) + 0.5*l1*cos(CoG_MB_ang)];
CoG_StanceThigh = [-l3*sin(CoG_StanceShank_ang) - 0.5*l2*sin(CoG_StanceThigh_ang);
                   +l3*cos(CoG_StanceShank_ang) + 0.5*l2*cos(CoG_StanceThigh_ang)];
CoG_StanceShank = [-0.5*l3*sin(CoG_StanceShank_ang);
                   +0.5*l3*cos(CoG_StanceShank_ang)];
CoG_SwingThigh  = [-l3*sin(CoG_StanceShank_ang) - l2*sin(CoG_StanceThigh_ang) + 0.5*l2*sin(CoG_SwingThigh_ang);
                   +l3*cos(CoG_StanceShank_ang) + l2*cos(CoG_StanceThigh_ang) - 0.5*l2*cos(CoG_SwingThigh_ang)];
CoG_SwingShank =  [-l3*sin(CoG_StanceShank_ang) - l2*sin(CoG_StanceThigh_ang) + l2*sin(CoG_SwingThigh_ang) + 0.5*l3*sin(CoG_SwingShank_ang);
                   +l3*cos(CoG_StanceShank_ang) + l2*cos(CoG_StanceThigh_ang) - l2*cos(CoG_SwingThigh_ang) - 0.5*l3*cos(CoG_SwingShank_ang)];
         
% CoG-velocities (computed via jacobians):
d_CoG_MB              = jacobian(CoG_MB,q)*dqdt;
d_CoG_StanceThigh     = jacobian(CoG_StanceThigh,q)*dqdt;
d_CoG_StanceShank     = jacobian(CoG_StanceShank,q)*dqdt;
d_CoG_SwingThigh      = jacobian(CoG_SwingThigh,q)*dqdt;
d_CoG_SwingShank      = jacobian(CoG_SwingShank,q)*dqdt;
d_CoG_MB_ang          = jacobian(CoG_MB_ang,q)*dqdt;
d_CoG_StanceThigh_ang = jacobian(CoG_StanceThigh_ang,q)*dqdt;
d_CoG_StanceShank_ang = jacobian(CoG_StanceShank_ang,q)*dqdt;
d_CoG_SwingThigh_ang  = jacobian(CoG_SwingThigh_ang,q)*dqdt;
d_CoG_SwingShank_ang  = jacobian(CoG_SwingShank_ang,q)*dqdt;

% Potential Energy (due to gravity):
V = CoG_MB(2)*m1*g + ...
    CoG_StanceThigh(2)*m2*g + ...
    CoG_SwingThigh(2)*m2*g + ...
    CoG_StanceShank(2)*m3*g + ...
    CoG_SwingShank(2)*m3*g;
V = simplify(expand(V));

% Kinetic Energy:         
T = 0.5 * (m1 * sum(d_CoG_MB.^2) + ...
           m2 * sum(d_CoG_StanceThigh.^2) + ...
           m2 * sum(d_CoG_SwingThigh.^2) + ...
           m3 * sum(d_CoG_StanceShank.^2) + ...
           m3 * sum(d_CoG_SwingShank.^2) + ...
           j1 * d_CoG_MB_ang^2 + ...
           j2 * d_CoG_StanceThigh_ang^2 + ...
           j2 * d_CoG_SwingThigh_ang^2 + ...
           j3 * d_CoG_StanceShank_ang^2 + ...
           j3 * d_CoG_SwingShank_ang^2);
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
Head       = [-l3*sin(CoG_StanceShank_ang) - l2*sin(CoG_StanceThigh_ang) - l1*sin(CoG_MB_ang);
              +l3*cos(CoG_StanceShank_ang) + l2*cos(CoG_StanceThigh_ang) + l1*cos(CoG_MB_ang)];
Hip        = [-l3*sin(CoG_StanceShank_ang) - l2*sin(CoG_StanceThigh_ang);
              +l3*cos(CoG_StanceShank_ang) + l2*cos(CoG_StanceThigh_ang)];
StanceKnee = [-l3*sin(CoG_StanceShank_ang);
              +l3*cos(CoG_StanceShank_ang)];
SwingKnee  = [-l3*sin(CoG_StanceShank_ang) - l2*sin(CoG_StanceThigh_ang) + l2*sin(CoG_SwingThigh_ang);
              +l3*cos(CoG_StanceShank_ang) + l2*cos(CoG_StanceThigh_ang) - l2*cos(CoG_SwingThigh_ang)];
StanceFoot = ([sym(0); 
               sym(0)]);
SwingFoot  = [-l3*sin(CoG_StanceShank_ang) - l2*sin(CoG_StanceThigh_ang) + l2*sin(CoG_SwingThigh_ang) + l3*sin(CoG_SwingShank_ang);
              +l3*cos(CoG_StanceShank_ang) + l2*cos(CoG_StanceThigh_ang) - l2*cos(CoG_SwingThigh_ang) - l3*cos(CoG_SwingShank_ang)];
                     
% CoGs (incl orientation of the segments):
CoGs = [CoG_MB,     CoG_StanceThigh,     CoG_StanceShank,     CoG_SwingThigh,     CoG_SwingShank;
        CoG_MB_ang, CoG_StanceThigh_ang, CoG_StanceShank_ang, CoG_SwingThigh_ang, CoG_SwingShank_ang];

% Links (or rather: joint positions)
links = [StanceFoot, StanceKnee, Hip, Head, Hip, SwingKnee, SwingFoot];

% Position of the foot points:     
footPts = [StanceFoot, SwingFoot];


%% CONTACT DYNAMICS 
% Contact point:
contPoint = simplify(SwingFoot);
contHeight = contPoint(2);

% For this model, the contact dynamics are processed in a 4DoF
% floating-base system. This %% makes it easier to include the post-impact
% constraint (the velocity of the impacting foot point comes to a rest) in
% the computation.  
% Define additional generalized coordinates:
syms x y real
syms dx dy real
% Define extended coordinate vector for contact collision:
q_cont = [x y q.'].';
dqdt_cont = [dx dy dqdt.'].';

% Re-compute the CoG positions (in a floating-base system). The angular
% definitions remain identical:
CoG_MB_float    = [x;
                   y];
CoG_StanceThigh  = CoG_MB_float + simplify(CoG_StanceThigh - CoG_MB);
CoG_StanceShank  = CoG_MB_float + simplify(CoG_StanceShank - CoG_MB);
CoG_SwingThigh   = CoG_MB_float + simplify(CoG_SwingThigh - CoG_MB);
CoG_SwingShank   = CoG_MB_float + simplify(CoG_SwingShank - CoG_MB);
StanceFoot = CoG_MB_float + simplify(StanceFoot - CoG_MB);
CoG_MB = CoG_MB_float;
% CoG-velocities (computed via jacobians):
d_CoG_MB              = jacobian(CoG_MB,q_cont)*dqdt_cont;
d_CoG_StanceThigh     = jacobian(CoG_StanceThigh,q_cont)*dqdt_cont;
d_CoG_StanceShank     = jacobian(CoG_StanceShank,q_cont)*dqdt_cont;
d_CoG_SwingThigh      = jacobian(CoG_SwingThigh,q_cont)*dqdt_cont;
d_CoG_SwingShank      = jacobian(CoG_SwingShank,q_cont)*dqdt_cont;
d_CoG_MB_ang          = jacobian(CoG_MB_ang,q_cont)*dqdt_cont;
d_CoG_StanceThigh_ang = jacobian(CoG_StanceThigh_ang,q_cont)*dqdt_cont;
d_CoG_StanceShank_ang = jacobian(CoG_StanceShank_ang,q_cont)*dqdt_cont;
d_CoG_SwingThigh_ang  = jacobian(CoG_SwingThigh_ang,q_cont)*dqdt_cont;
d_CoG_SwingShank_ang  = jacobian(CoG_SwingShank_ang,q_cont)*dqdt_cont;
% Potential Energy is not needed (We only need the mass matrix)
% Kinetic Energy:         
T = 0.5 * (m1 * sum(d_CoG_MB.^2) + ...
           m2 * sum(d_CoG_StanceThigh.^2) + ...
           m2 * sum(d_CoG_StanceShank.^2) + ...
           m3 * sum(d_CoG_SwingThigh.^2) + ...
           m3 * sum(d_CoG_SwingShank.^2) + ...
           j1 * d_CoG_MB_ang^2 + ...
           j2 * d_CoG_StanceThigh_ang^2 + ...
           j2 * d_CoG_StanceShank_ang^2 + ...
           j3 * d_CoG_SwingThigh_ang^2 + ...
           j3 * d_CoG_SwingShank_ang^2);
T = simplify(expand(T));
% Lagrangian (ignoring potential energy):
L = T;
% Compute Mass Matrix:
dLdqdt = jacobian(L,dqdt_cont)';
M_cont = jacobian(dLdqdt,dqdt_cont);
M_cont = simplify(M_cont);

% Contact point and contact Jacobian:
contPoint = StanceFoot;
% Contact Jacobian:
% NOTE: The round feet introduce a non-holonomic contact constraint.  I.e.
% the Jacobian needs to be expanded afterwards with the foot rotation:
J_cont = jacobian(contPoint,q_cont);
J_cont = simplify(J_cont);                            



%% Create MATLAB-functions:
if ~exist('AutoGeneratedFcts','dir')
    mkdir('AutoGeneratedFcts')
end
% for continuous dynamics:
matlabFunction(M,'file','AutoGeneratedFcts\MassMatrix','vars',{q, p});
matlabFunction(f_cg,'file','AutoGeneratedFcts\F_CoriGrav','vars',{q, dqdt, p});
% for implicit constraints and collisions:
matlabFunction(contHeight,'file','AutoGeneratedFcts\ContactHeight','vars',{q, p});
matlabFunction(J_cont,'file','AutoGeneratedFcts\ContactJacobian','vars',{q_cont, p});
matlabFunction(M_cont,'file','AutoGeneratedFcts\ContactMassMatrix','vars',{q_cont,p});
% for graphics:
matlabFunction(CoGs,'file','AutoGeneratedFcts\CoGPositions','vars',{q, p});
matlabFunction(links,'file','AutoGeneratedFcts\LinkPositions','vars',{q, p});
matlabFunction(footPts,'file','AutoGeneratedFcts\FootPtPositions','vars',{q, p});