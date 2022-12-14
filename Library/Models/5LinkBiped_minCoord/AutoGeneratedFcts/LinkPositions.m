function links = LinkPositions(in1,in2)
%LINKPOSITIONS
%    LINKS = LINKPOSITIONS(IN1,IN2)

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
t6 = l3.*t2;
t7 = l2.*t4;
t5 = -t6-t7;
t8 = q1-q2-q3+q4;
t9 = sin(t8);
t10 = l2.*t9;
t11 = cos(q1);
t12 = l3.*t11;
t13 = -q1+q2+q3;
t14 = cos(t3);
t15 = l2.*t14;
t16 = t12+t15;
t17 = cos(t8);
t18 = q1-q2-q3+q4+q5;
links = reshape([0.0,0.0,-l3.*t2,t12,t5,t16,-l3.*t2-l2.*t4+l1.*sin(t13),t12+t15+l1.*cos(t13),t5,t16,-t6-t7+t10,t12+t15-l2.*t17,-t6-t7+t10+l3.*sin(t18),t12+t15-l2.*t17-l3.*cos(t18)],[2,7]);
