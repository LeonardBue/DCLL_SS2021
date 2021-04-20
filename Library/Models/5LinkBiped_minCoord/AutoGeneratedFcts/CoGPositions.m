function CoGs = CoGPositions(in1,in2)
%COGPOSITIONS
%    COGS = COGPOSITIONS(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.1.
%    25-Apr-2020 09:27:15

l1 = in2(2,:);
l2 = in2(3,:);
l3 = in2(4,:);
q1 = in1(1,:);
q2 = in1(2,:);
q3 = in1(3,:);
q4 = in1(4,:);
q5 = in1(5,:);
t2 = sin(q1);
t3 = q1-q2;
t4 = sin(t3);
t5 = q1-q2-q3+q4;
t6 = sin(t5);
t7 = -q1+q2+q3;
t8 = cos(q1);
t9 = l3.*t8;
t10 = cos(t3);
t11 = l2.*t10;
t12 = cos(t5);
t13 = q1-q2-q3+q4+q5;
CoGs = reshape([-l3.*t2-l2.*t4+l1.*sin(t7).*(1.0./2.0),t9+t11+l1.*cos(t7).*(1.0./2.0),q1-q2-q3,-l3.*t2-l2.*t4.*(1.0./2.0),t9+l2.*t10.*(1.0./2.0),t3,l3.*t2.*(-1.0./2.0),l3.*t8.*(1.0./2.0),q1,-l3.*t2-l2.*t4+l2.*t6.*(1.0./2.0),t9+t11-l2.*t12.*(1.0./2.0),t5,-l3.*t2-l2.*t4+l2.*t6+l3.*sin(t13).*(1.0./2.0),t9+t11-l2.*t12-l3.*cos(t13).*(1.0./2.0),t13],[3,5]);
