clear all;
clc;

syms q1 q2 q3 q4 q5 q6 q7;

T01 = [cos(q1) -sin(q1) 0 0;
       sin(q1)  cos(q1) 0 0;
       0 0 0 0.333;
       0 0 0 1];
T12 = [cos(q2) 0 -sin(q2) 0;
       sin(q2) 0  cos(q2) 0;
       0 -1 0 0;
       0 0 0 1];
T23 = [cos(q3) 0  sin(q3) 0;
       sin(q3) 0 -cos(q3) 0;
       0 1 0 0.316;
       0 0 0 1];
T34 = [cos(q4) 0  sin(q4) 0.0825*cos(q4);
       sin(q4) 0 -cos(q4) 0.0825*sin(q4);
       0 1 0 0;
       0 0 0 1];
T45 = [cos(q5) 0 -sin(q5) -0.0825*cos(q5);
       sin(q5) 0  cos(q5) -0.0825*sin(q5);
       0 -1 0 0;
       0 0 0 1];
T56 = [cos(q6) 0  sin(q6) 0;
       sin(q6) 0 -cos(q6) 0;
       0 1 0 0;
       0 0 0 1];
T67 = [cos(q7) 0  sin(q7) 0.088*cos(q7);
       sin(q7) 0 -cos(q4) 0.088*sin(q7);
       0 1 0 0;
       0 0 0 1];
T07 = T01*T12*T23*T34*T45*T56*T67;
T07_mod = [T07(1,:) T07(2,:) T07(3,:)];

J = jacobian(T07_mod,[q1,q2,q3,q4,q5,q6,q7]);
J_sim = simplify(J);

fileID = fopen('j_mat.txt','wt');
fprintf(fileID,J_sim);
