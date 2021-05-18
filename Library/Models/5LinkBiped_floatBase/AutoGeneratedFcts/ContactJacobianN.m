function JN = ContactJacobianN(in1,in2)
%CONTACTJACOBIANN
%    JN = CONTACTJACOBIANN(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.1.
%    23-Apr-2020 21:33:36

alphaL = in1(4,:);
alphaR = in1(5,:);
betaL = in1(6,:);
betaR = in1(7,:);
l2 = in2(3,:);
l3 = in2(4,:);
phi = in1(3,:);
t2 = alphaL+phi;
t3 = sin(t2);
t4 = l2.*t3;
t5 = alphaL+betaL+phi;
t6 = sin(t5);
t7 = l3.*t6;
t8 = t4+t7;
t9 = alphaR+phi;
t10 = sin(t9);
t11 = l2.*t10;
t12 = alphaR+betaR+phi;
t13 = sin(t12);
t14 = l3.*t13;
t15 = t11+t14;
JN = reshape([0.0,0.0,1.0,1.0,t8,t15,t8,0.0,0.0,t15,t7,0.0,0.0,t14],[2,7]);