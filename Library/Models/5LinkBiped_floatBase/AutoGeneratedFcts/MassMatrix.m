function M = MassMatrix(in1,in2)
%MASSMATRIX
%    M = MASSMATRIX(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.1.
%    23-Apr-2020 21:33:32

alphaL = in1(4,:);
alphaR = in1(5,:);
betaL = in1(6,:);
betaR = in1(7,:);
j1 = in2(8,:);
j2 = in2(9,:);
j3 = in2(10,:);
l1 = in2(2,:);
l2 = in2(3,:);
l3 = in2(4,:);
m1 = in2(5,:);
m2 = in2(6,:);
m3 = in2(7,:);
phi = in1(3,:);
t2 = alphaL+phi;
t3 = cos(t2);
t4 = alphaR+phi;
t5 = cos(t4);
t6 = alphaL+betaL+phi;
t7 = cos(t6);
t8 = l3.*m3.*t7.*(1.0./2.0);
t9 = l2.*m2.*t3.*(1.0./2.0);
t10 = l2.*m3.*t3;
t11 = alphaR+betaR+phi;
t12 = cos(t11);
t13 = l3.*m3.*t12.*(1.0./2.0);
t14 = l2.*m2.*t5.*(1.0./2.0);
t15 = l2.*m3.*t5;
t16 = m2.*2.0;
t17 = m3.*2.0;
t18 = m1+t16+t17;
t19 = sin(t2);
t20 = sin(t4);
t21 = sin(t6);
t22 = l3.*m3.*t21.*(1.0./2.0);
t23 = l2.*m2.*t19.*(1.0./2.0);
t24 = l2.*m3.*t19;
t25 = sin(t11);
t26 = l3.*m3.*t25.*(1.0./2.0);
t27 = l2.*m2.*t20.*(1.0./2.0);
t28 = l2.*m3.*t20;
t29 = cos(phi);
t30 = t8+t9+t10+t13+t14+t15-l1.*m1.*t29.*(1.0./2.0);
t31 = sin(phi);
t32 = t22+t23+t24+t26+t27+t28-l1.*m1.*t31.*(1.0./2.0);
t33 = l2.^2;
t34 = l3.^2;
t35 = cos(betaL);
t36 = l2.*l3.*m3.*t35;
t37 = m2.*t33.*(1.0./4.0);
t38 = m3.*t33;
t39 = m3.*t34.*(1.0./4.0);
t40 = cos(betaR);
t41 = l2.*l3.*m3.*t40;
t42 = t8+t9+t10;
t43 = t22+t23+t24;
t44 = j2+j3+t36+t37+t38+t39;
t45 = l2.*l3.*m3.*t35.*(1.0./2.0);
t46 = j3+t39+t45;
t47 = t13+t14+t15;
t48 = t26+t27+t28;
t49 = j2+j3+t37+t38+t39+t41;
t50 = l2.*l3.*m3.*t40.*(1.0./2.0);
t51 = j3+t39+t50;
t52 = j3+t39;
M = reshape([t18,0.0,t30,t42,t47,t8,t13,0.0,t18,t32,t43,t48,t22,t26,t30,t32,j1+j2.*2.0+j3.*2.0+t36+t41+m2.*t33.*(1.0./2.0)+m3.*t33.*2.0+m3.*t34.*(1.0./2.0)+l1.^2.*m1.*(1.0./4.0),t44,t49,t46,t51,t42,t43,t44,t44,0.0,t46,0.0,t47,t48,t49,0.0,t49,0.0,t51,t8,t22,t46,t46,0.0,t52,0.0,t13,t26,t51,0.0,t51,0.0,t52],[7,7]);
