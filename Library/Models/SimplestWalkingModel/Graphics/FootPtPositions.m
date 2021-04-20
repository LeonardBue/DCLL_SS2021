% This function returns locations of the Simplest Walking Model's feet. 
% These positions are then used by the model's graphical output class to 
% create animation.
% 
% INPUT:    q  --  vector of the generalized coordinates;
%           p  --  vector of the system parameters.
% OUTPUT:   footPts  --  matrix storing foot locations:
%               1st column: stance foot position;
%               2nd column: swing foot position.
%
function footPts = FootPtPositions(q, p)

theta = q(1);
phi   = q(2);

% Positions of the swing foot with respect to the slope:
% (Note: all lengths are normalized with respect to the leg length l)
xSwFoot = -1*sin(theta)+1*sin(theta-phi);
ySwFoot = +1*cos(theta)-1*cos(theta-phi);

footPts = [[0;0], [xSwFoot;ySwFoot]];
