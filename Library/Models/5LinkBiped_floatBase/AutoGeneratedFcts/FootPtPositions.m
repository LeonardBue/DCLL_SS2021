function footPts = FootPtPositions(in1,in2)
%FOOTPTPOSITIONS
%    FOOTPTS = FOOTPTPOSITIONS(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.1.
%    23-Apr-2020 21:33:34

alphaL = in1(4,:);
alphaR = in1(5,:);
betaL = in1(6,:);
betaR = in1(7,:);
l2 = in2(3,:);
l3 = in2(4,:);
phi = in1(3,:);
x = in1(1,:);
y = in1(2,:);
t2 = alphaL+phi;
t3 = alphaL+betaL+phi;
t4 = alphaR+phi;
t5 = alphaR+betaR+phi;
footPts = reshape([x+l2.*sin(t2)+l3.*sin(t3),y-l2.*cos(t2)-l3.*cos(t3),x+l2.*sin(t4)+l3.*sin(t5),y-l2.*cos(t4)-l3.*cos(t5)],[2,2]);
