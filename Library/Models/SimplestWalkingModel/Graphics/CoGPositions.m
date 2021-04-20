% This function returns positions (locations and orientations) of the 
% Simplest Walking Model's feet and hip. These positions are then used by 
% the model's graphical output class to create animation.
% 
% INPUT:    q  --  vector of the generalized coordinates;
%           p  --  vector of the system parameters.
% OUTPUT:   CoGs  --  matrix storing joint locations and orientations:
%               1st column: stance foot position;
%               2nd column: hip position;
%               3rd column: swing foot position.
%
function CoGs = CoGPositions(q, p)

theta = q(1);
phi   = q(2);

% Positions of the hip and the swing foot with respect to the slope:
% (Note: all lengths are normalized with respect to the leg length l)
xHip = -1*sin(theta);
yHip = 1*cos(theta);
xSwFoot = -1*sin(theta)+1*sin(theta-phi);
ySwFoot = +1*cos(theta)-1*cos(theta-phi);

% Define the positions of the CoGs, model links (legs), and foot points
% for the graphical output:
CoGs    = [[0;0;0], [xHip;yHip;0], [xSwFoot;ySwFoot;0]];
    
    
