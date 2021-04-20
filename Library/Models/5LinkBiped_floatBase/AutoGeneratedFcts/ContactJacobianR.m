function JR = ContactJacobianR(in1,in2)
%CONTACTJACOBIANR
%    JR = CONTACTJACOBIANR(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.1.
%    23-Apr-2020 21:33:35

alphaR = in1(5,:);
betaR = in1(7,:);
l2 = in2(3,:);
l3 = in2(4,:);
phi = in1(3,:);
t2 = alphaR+phi;
t3 = cos(t2);
t4 = l2.*t3;
t5 = alphaR+betaR+phi;
t6 = cos(t5);
t7 = l3.*t6;
t8 = t4+t7;
t9 = sin(t2);
t10 = l2.*t9;
t11 = sin(t5);
t12 = l3.*t11;
t13 = t10+t12;
JR = reshape([1.0,0.0,0.0,1.0,t8,t13,0.0,0.0,t8,t13,0.0,0.0,t7,t12],[2,7]);
