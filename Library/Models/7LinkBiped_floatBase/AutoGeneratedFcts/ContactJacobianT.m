function JT = ContactJacobianT(in1,in2)
%CONTACTJACOBIANT
%    JT = CONTACTJACOBIANT(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.1.
%    25-Apr-2020 09:28:16

alphaL = in1(4,:);
alphaR = in1(5,:);
betaL = in1(6,:);
betaR = in1(7,:);
gammaL = in1(8,:);
gammaR = in1(9,:);
l2 = in2(3,:);
l3 = in2(4,:);
l4 = in2(5,:);
phi = in1(3,:);
t2 = alphaL+phi;
t3 = cos(t2);
t4 = l2.*t3;
t5 = alphaL+betaL+phi;
t6 = cos(t5);
t7 = l3.*t6;
t8 = t4+t7;
t9 = alphaR+phi;
t10 = cos(t9);
t11 = l2.*t10;
t12 = alphaR+betaR+phi;
t13 = cos(t12);
t14 = l3.*t13;
t15 = t11+t14;
t16 = alphaL+betaL+gammaL+phi;
t17 = cos(t16);
t18 = l4.*t17;
t19 = t4+t7+t18;
t20 = alphaR+betaR+gammaR+phi;
t21 = cos(t20);
t22 = l4.*t21;
t23 = t11+t14+t22;
JT = reshape([1.0,1.0,1.0,1.0,0.0,0.0,0.0,0.0,t8,t15,t19,t23,t8,0.0,t19,0.0,0.0,t15,0.0,t23,t7,0.0,t7+t18,0.0,0.0,t14,0.0,t14+t22,0.0,0.0,t18,0.0,0.0,0.0,0.0,t22],[4,9]);
