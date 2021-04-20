% This function defines locations of the model's links. It returns the 
% locations of the joints in the order in which they have to be graphically
% connected to draw all of the model's links. For the Simplest Walking
% Model, this is [stance foot, hip, swing foot] to draw the stance and
% swing leg.
% 
% INPUT:    q  --  vector of the generalized coordinates;
%           p  --  vector of the system parameters.
% OUTPUT:   links  --  matrix storing model's links (legs) locations:
%               1st column: stance foot position;
%               2nd column: hip position;
%               3rd column: swing foot position.
%
function links = LinkPositions(q, p)

theta = q(1);
phi   = q(2);

% Positions of the hip and the swing foot with respect to the slope:
% (Note: all lengths are normalized with respect to the leg length l)
xHip = -1*sin(theta);
yHip = 1*cos(theta);
xSwFoot = -1*sin(theta)+1*sin(theta-phi);
ySwFoot = +1*cos(theta)-1*cos(theta-phi);

links   = [[0;0], [xHip;yHip], [xSwFoot;ySwFoot]];
