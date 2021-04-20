function CoGs = CoGPositions(in1,in2)
%COGPOSITIONS
%    COGS = COGPOSITIONS(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.1.
%    25-Apr-2020 09:28:16

alphaL = in1(4,:);
alphaR = in1(5,:);
betaL = in1(6,:);
betaR = in1(7,:);
gammaL = in1(8,:);
gammaR = in1(9,:);
l1 = in2(2,:);
l2 = in2(3,:);
l3 = in2(4,:);
l4 = in2(5,:);
phi = in1(3,:);
x = in1(1,:);
y = in1(2,:);
t2 = alphaL+phi;
t3 = sin(t2);
t4 = l2.*t3;
t5 = alphaL+betaL+phi;
t6 = sin(t5);
t7 = alphaR+phi;
t8 = sin(t7);
t9 = l2.*t8;
t10 = alphaR+betaR+phi;
t11 = sin(t10);
t12 = cos(t2);
t13 = alphaL+betaL+gammaL+phi;
t14 = cos(t5);
t15 = cos(t7);
t16 = alphaR+betaR+gammaR+phi;
t17 = cos(t10);
CoGs = reshape([x-l1.*sin(phi).*(1.0./2.0),y+l1.*cos(phi).*(1.0./2.0),phi,x+l2.*t3.*(1.0./2.0),y-l2.*t12.*(1.0./2.0),t2,t4+x+l3.*t6.*(1.0./2.0),y-l2.*t12-l3.*t14.*(1.0./2.0),t5,t4+x+l3.*t6+l4.*sin(t13).*(1.0./2.0),y-l2.*t12-l3.*t14-l4.*cos(t13).*(1.0./2.0),t13,x+l2.*t8.*(1.0./2.0),y-l2.*t15.*(1.0./2.0),t7,t9+x+l3.*t11.*(1.0./2.0),y-l2.*t15-l3.*t17.*(1.0./2.0),t10,t9+x+l3.*t11+l4.*sin(t16).*(1.0./2.0),y-l2.*t15-l3.*t17-l4.*cos(t16).*(1.0./2.0),t16],[3,7]);
