//
// Created by sherif on 20.06.22.
//
#include "iostream"
#include <Eigen/QR>
#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include <math.h>
#include "subpub.h"
#include "sensor_msgs/JointState.h"



using namespace std;
using namespace Eigen;

int check = 0;
Vector<double, 7> q_joint;



Matrix4d fk (Vector<double, 7> q) {
    Matrix4d A1;
    Matrix4d A2;
    Matrix4d A3;
    Matrix4d A4;
    Matrix4d A5;
    Matrix4d A6;
    Matrix4d A7;
    Matrix4d A8;

    double q0 = q[0];
    double q1 = q[1];
    double q2 = q[2];
    double q3 = q[3];
    double q4 = q[4];
    double q5 = q[5];
    double q6 = q[6];

    A1 <<   cos(q0),-sin (q0),0, 0,
            sin(q0), cos (q0), 0, 0,
            0, 0, 1, 0.333,
            0, 0, 0, 1;

    A2 <<   cos(q1),0, -sin (q1), 0,
            sin(q1), 0, cos (q1),0,
            0, -1, 0, 0,
            0, 0, 0, 1;

    A3 <<   cos(q2),0, sin (q2), 0,
            sin(q2), 0, -cos (q2), 0,
            0, 1, 0, 0.316,
            0, 0, 0, 1;

    A4  <<  cos(q3),0, sin (q3), 0.0825*cos(q3),
            sin(q3), 0, -cos (q3), 0.0825*cos(q3),
            0, 1, 0, 0,
            0, 0, 0, 1;

    A5 <<   cos(q4),0, -sin (q4), -0.0825*cos(q4),
            sin(q4), 0, cos (q4), -0.0825*cos(q4),
            0, -1, 0, 0,
            0, 0, 0, 1;

    A6 << cos(q5),0, sin (q5), 0,
            sin(q5), 0, -cos (q5), 0,
            0, 1, 0, 0,
            0, 0, 0, 1;

    A7 << cos(q6),0, sin (q6), 0.088*cos(q6),
            sin(q6), 0, -cos (q6), 0.088*cos(q6),
            0, 1, 0, 0,
            0, 0, 0, 1;

    Matrix4d T70 ;
    T70 = A1*A2*A3*A4*A5*A6*A7 ;
    return T70;
}

MatrixXd jacobian_inv (Vector<double, 7> q) {
    MatrixXd J(12, 7);
    MatrixXd J_inv;

    double q1 = q[0];
    double q2 = q[1];
    double q3 = q[2];
    double q4 = q[3];
    double q5 = q[4];
    double q6 = q[5];
    double q7 = q[6];

    J(0,0) = sin(q7)*(sin(q5)*(sin(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) + cos(q3)*cos(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1))) - cos(q5)*sin(q3)*(cos(q1)*sin(q2) + cos(q2)*sin(q1))) - cos(q7)*(sin(q6)*(cos(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) - cos(q3)*sin(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1))) + cos(q6)*(cos(q5)*(sin(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) + cos(q3)*cos(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1))) + sin(q3)*sin(q5)*(cos(q1)*sin(q2) + cos(q2)*sin(q1))));
    J(0,1) = sin(q7)*(sin(q5)*(sin(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) + cos(q3)*cos(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1))) - cos(q5)*sin(q3)*(cos(q1)*sin(q2) + cos(q2)*sin(q1))) - cos(q7)*(sin(q6)*(cos(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) - cos(q3)*sin(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1))) + cos(q6)*(cos(q5)*(sin(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) + cos(q3)*cos(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1))) + sin(q3)*sin(q5)*(cos(q1)*sin(q2) + cos(q2)*sin(q1))));
    J(0,2) = sin(q7)*(cos(q3)*cos(q5)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) + cos(q4)*sin(q3)*sin(q5)*(cos(q1)*cos(q2) - sin(q1)*sin(q2))) + cos(q7)*(cos(q6)*(cos(q3)*sin(q5)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) - cos(q4)*cos(q5)*sin(q3)*(cos(q1)*cos(q2) - sin(q1)*sin(q2))) + sin(q3)*sin(q4)*sin(q6)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)));
    J(0,3) = cos(q7)*(sin(q6)*(sin(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) - cos(q3)*cos(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2))) - cos(q5)*cos(q6)*(cos(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) + cos(q3)*sin(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)))) + sin(q5)*sin(q7)*(cos(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) + cos(q3)*sin(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)));
    J(0,4) = sin(q7)*(cos(q5)*(sin(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) - cos(q3)*cos(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2))) - sin(q3)*sin(q5)*(cos(q1)*cos(q2) - sin(q1)*sin(q2))) + cos(q6)*cos(q7)*(sin(q5)*(sin(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) - cos(q3)*cos(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2))) + cos(q5)*sin(q3)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)));
    J(0,5) = cos(q7)*(sin(q6)*(cos(q5)*(sin(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) - cos(q3)*cos(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2))) - sin(q3)*sin(q5)*(cos(q1)*cos(q2) - sin(q1)*sin(q2))) - cos(q6)*(cos(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) + cos(q3)*sin(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2))));
    J(0,6) = cos(q7)*(sin(q5)*(sin(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) - cos(q3)*cos(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2))) + cos(q5)*sin(q3)*(cos(q1)*cos(q2) - sin(q1)*sin(q2))) + sin(q7)*(sin(q6)*(cos(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) + cos(q3)*sin(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2))) + cos(q6)*(cos(q5)*(sin(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) - cos(q3)*cos(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2))) - sin(q3)*sin(q5)*(cos(q1)*cos(q2) - sin(q1)*sin(q2))));

    J(1,0) = cos(q6)*(cos(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) - cos(q3)*sin(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1))) - sin(q6)*(cos(q5)*(sin(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) + cos(q3)*cos(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1))) + sin(q3)*sin(q5)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)));
    J(1,1) = cos(q6)*(cos(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) - cos(q3)*sin(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1))) - sin(q6)*(cos(q5)*(sin(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) + cos(q3)*cos(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1))) + sin(q3)*sin(q5)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)));
    J(1,2) = sin(q6)*(cos(q3)*sin(q5)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) - cos(q4)*cos(q5)*sin(q3)*(cos(q1)*cos(q2) - sin(q1)*sin(q2))) - cos(q6)*sin(q3)*sin(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2));
    J(1,3) = - cos(q6)*(sin(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) - cos(q3)*cos(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2))) - cos(q5)*sin(q6)*(cos(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) + cos(q3)*sin(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)));
    J(1,4) = sin(q6)*(sin(q5)*(sin(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) - cos(q3)*cos(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2))) + cos(q5)*sin(q3)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)));
    J(1,5) = - sin(q6)*(cos(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) + cos(q3)*sin(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2))) - cos(q6)*(cos(q5)*(sin(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) - cos(q3)*cos(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2))) - sin(q3)*sin(q5)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)));
    J(1,6) = 0;

    J(2,0) = - cos(q7)*(sin(q5)*(sin(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) + cos(q3)*cos(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1))) - cos(q5)*sin(q3)*(cos(q1)*sin(q2) + cos(q2)*sin(q1))) - sin(q7)*(sin(q6)*(cos(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) - cos(q3)*sin(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1))) + cos(q6)*(cos(q5)*(sin(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) + cos(q3)*cos(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1))) + sin(q3)*sin(q5)*(cos(q1)*sin(q2) + cos(q2)*sin(q1))));
    J(2,1) = - cos(q7)*(sin(q5)*(sin(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) + cos(q3)*cos(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1))) - cos(q5)*sin(q3)*(cos(q1)*sin(q2) + cos(q2)*sin(q1))) - sin(q7)*(sin(q6)*(cos(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) - cos(q3)*sin(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1))) + cos(q6)*(cos(q5)*(sin(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) + cos(q3)*cos(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1))) + sin(q3)*sin(q5)*(cos(q1)*sin(q2) + cos(q2)*sin(q1))));
    J(2,2) = sin(q7)*(cos(q6)*(cos(q3)*sin(q5)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) - cos(q4)*cos(q5)*sin(q3)*(cos(q1)*cos(q2) - sin(q1)*sin(q2))) + sin(q3)*sin(q4)*sin(q6)*(cos(q1)*cos(q2) - sin(q1)*sin(q2))) - cos(q7)*(cos(q3)*cos(q5)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) + cos(q4)*sin(q3)*sin(q5)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)));
    J(2,3) = sin(q7)*(sin(q6)*(sin(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) - cos(q3)*cos(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2))) - cos(q5)*cos(q6)*(cos(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) + cos(q3)*sin(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)))) - cos(q7)*sin(q5)*(cos(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) + cos(q3)*sin(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)));
    J(2,4) = cos(q6)*sin(q7)*(sin(q5)*(sin(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) - cos(q3)*cos(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2))) + cos(q5)*sin(q3)*(cos(q1)*cos(q2) - sin(q1)*sin(q2))) - cos(q7)*(cos(q5)*(sin(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) - cos(q3)*cos(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2))) - sin(q3)*sin(q5)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)));
    J(2,5) = sin(q7)*(sin(q6)*(cos(q5)*(sin(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) - cos(q3)*cos(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2))) - sin(q3)*sin(q5)*(cos(q1)*cos(q2) - sin(q1)*sin(q2))) - cos(q6)*(cos(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) + cos(q3)*sin(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2))));
    J(2,6) = sin(q7)*(sin(q5)*(sin(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) - cos(q3)*cos(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2))) + cos(q5)*sin(q3)*(cos(q1)*cos(q2) - sin(q1)*sin(q2))) - cos(q7)*(sin(q6)*(cos(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) + cos(q3)*sin(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2))) + cos(q6)*(cos(q5)*(sin(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) - cos(q3)*cos(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2))) - sin(q3)*sin(q5)*(cos(q1)*cos(q2) - sin(q1)*sin(q2))));

    J(3,0) = (79*sin(q1)*sin(q2))/250 - (79*cos(q1)*cos(q2))/250 + (11*sin(q7)*(sin(q5)*(sin(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) + cos(q3)*cos(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1))) - cos(q5)*sin(q3)*(cos(q1)*sin(q2) + cos(q2)*sin(q1))))/125 - (33*sin(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)))/400 - (11*cos(q7)*(sin(q6)*(cos(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) - cos(q3)*sin(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1))) + cos(q6)*(cos(q5)*(sin(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) + cos(q3)*cos(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1))) + sin(q3)*sin(q5)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)))))/125 + (33*cos(q5)*(sin(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) + cos(q3)*cos(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1))))/400 - (33*cos(q3)*cos(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)))/400 + (33*sin(q3)*sin(q5)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)))/400;
    J(3,1) = (79*sin(q1)*sin(q2))/250 - (79*cos(q1)*cos(q2))/250 + (11*sin(q7)*(sin(q5)*(sin(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) + cos(q3)*cos(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1))) - cos(q5)*sin(q3)*(cos(q1)*sin(q2) + cos(q2)*sin(q1))))/125 - (33*sin(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)))/400 - (11*cos(q7)*(sin(q6)*(cos(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) - cos(q3)*sin(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1))) + cos(q6)*(cos(q5)*(sin(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) + cos(q3)*cos(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1))) + sin(q3)*sin(q5)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)))))/125 + (33*cos(q5)*(sin(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) + cos(q3)*cos(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1))))/400 - (33*cos(q3)*cos(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)))/400 + (33*sin(q3)*sin(q5)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)))/400;
    J(3,2) = (11*sin(q7)*(cos(q3)*cos(q5)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) + cos(q4)*sin(q3)*sin(q5)*(cos(q1)*cos(q2) - sin(q1)*sin(q2))))/125 + (11*cos(q7)*(cos(q6)*(cos(q3)*sin(q5)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) - cos(q4)*cos(q5)*sin(q3)*(cos(q1)*cos(q2) - sin(q1)*sin(q2))) + sin(q3)*sin(q4)*sin(q6)*(cos(q1)*cos(q2) - sin(q1)*sin(q2))))/125 - (33*cos(q4)*sin(q3)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)))/400 - (33*cos(q3)*sin(q5)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)))/400 + (33*cos(q4)*cos(q5)*sin(q3)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)))/400;
    J(3,3) = (11*cos(q7)*(sin(q6)*(sin(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) - cos(q3)*cos(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2))) - cos(q5)*cos(q6)*(cos(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) + cos(q3)*sin(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)))))/125 - (33*cos(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)))/400 + (33*cos(q5)*(cos(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) + cos(q3)*sin(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2))))/400 - (33*cos(q3)*sin(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)))/400 + (11*sin(q5)*sin(q7)*(cos(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) + cos(q3)*sin(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2))))/125;
    J(3,4) = (11*sin(q7)*(cos(q5)*(sin(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) - cos(q3)*cos(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2))) - sin(q3)*sin(q5)*(cos(q1)*cos(q2) - sin(q1)*sin(q2))))/125 - (33*sin(q5)*(sin(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) - cos(q3)*cos(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2))))/400 + (11*cos(q6)*cos(q7)*(sin(q5)*(sin(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) - cos(q3)*cos(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2))) + cos(q5)*sin(q3)*(cos(q1)*cos(q2) - sin(q1)*sin(q2))))/125 - (33*cos(q5)*sin(q3)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)))/400;
    J(3,5) = (11*cos(q7)*(sin(q6)*(cos(q5)*(sin(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) - cos(q3)*cos(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2))) - sin(q3)*sin(q5)*(cos(q1)*cos(q2) - sin(q1)*sin(q2))) - cos(q6)*(cos(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) + cos(q3)*sin(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)))))/125;
    J(3,6) = (11*cos(q7)*(sin(q5)*(sin(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) - cos(q3)*cos(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2))) + cos(q5)*sin(q3)*(cos(q1)*cos(q2) - sin(q1)*sin(q2))))/125 + (11*sin(q7)*(sin(q6)*(cos(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) + cos(q3)*sin(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2))) + cos(q6)*(cos(q5)*(sin(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) - cos(q3)*cos(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2))) - sin(q3)*sin(q5)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)))))/125;

    J(4,0) = sin(q7)*(sin(q5)*(sin(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) - cos(q3)*cos(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2))) + cos(q5)*sin(q3)*(cos(q1)*cos(q2) - sin(q1)*sin(q2))) - cos(q7)*(sin(q6)*(cos(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) + cos(q3)*sin(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2))) + cos(q6)*(cos(q5)*(sin(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) - cos(q3)*cos(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2))) - sin(q3)*sin(q5)*(cos(q1)*cos(q2) - sin(q1)*sin(q2))));
    J(4,1) = sin(q7)*(sin(q5)*(sin(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) - cos(q3)*cos(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2))) + cos(q5)*sin(q3)*(cos(q1)*cos(q2) - sin(q1)*sin(q2))) - cos(q7)*(sin(q6)*(cos(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) + cos(q3)*sin(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2))) + cos(q6)*(cos(q5)*(sin(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) - cos(q3)*cos(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2))) - sin(q3)*sin(q5)*(cos(q1)*cos(q2) - sin(q1)*sin(q2))));
    J(4,2) = sin(q7)*(cos(q3)*cos(q5)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) + cos(q4)*sin(q3)*sin(q5)*(cos(q1)*sin(q2) + cos(q2)*sin(q1))) + cos(q7)*(cos(q6)*(cos(q3)*sin(q5)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) - cos(q4)*cos(q5)*sin(q3)*(cos(q1)*sin(q2) + cos(q2)*sin(q1))) + sin(q3)*sin(q4)*sin(q6)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)));
    J(4,3) = - cos(q7)*(sin(q6)*(sin(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) + cos(q3)*cos(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1))) - cos(q5)*cos(q6)*(cos(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) - cos(q3)*sin(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)))) - sin(q5)*sin(q7)*(cos(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) - cos(q3)*sin(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)));
    J(4,4) = - sin(q7)*(cos(q5)*(sin(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) + cos(q3)*cos(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1))) + sin(q3)*sin(q5)*(cos(q1)*sin(q2) + cos(q2)*sin(q1))) - cos(q6)*cos(q7)*(sin(q5)*(sin(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) + cos(q3)*cos(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1))) - cos(q5)*sin(q3)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)));
    J(4,5) = -cos(q7)*(sin(q6)*(cos(q5)*(sin(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) + cos(q3)*cos(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1))) + sin(q3)*sin(q5)*(cos(q1)*sin(q2) + cos(q2)*sin(q1))) - cos(q6)*(cos(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) - cos(q3)*sin(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1))));
    J(4,6) = - cos(q7)*(sin(q5)*(sin(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) + cos(q3)*cos(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1))) - cos(q5)*sin(q3)*(cos(q1)*sin(q2) + cos(q2)*sin(q1))) - sin(q7)*(sin(q6)*(cos(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) - cos(q3)*sin(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1))) + cos(q6)*(cos(q5)*(sin(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) + cos(q3)*cos(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1))) + sin(q3)*sin(q5)*(cos(q1)*sin(q2) + cos(q2)*sin(q1))));

    J(5,0) = cos(q6)*(cos(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) + cos(q3)*sin(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2))) - sin(q6)*(cos(q5)*(sin(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) - cos(q3)*cos(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2))) - sin(q3)*sin(q5)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)));
    J(5,1) = cos(q6)*(cos(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) + cos(q3)*sin(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2))) - sin(q6)*(cos(q5)*(sin(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) - cos(q3)*cos(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2))) - sin(q3)*sin(q5)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)));
    J(5,2) = sin(q6)*(cos(q3)*sin(q5)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) - cos(q4)*cos(q5)*sin(q3)*(cos(q1)*sin(q2) + cos(q2)*sin(q1))) - cos(q6)*sin(q3)*sin(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1));
    J(5,3) = cos(q6)*(sin(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) + cos(q3)*cos(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1))) + cos(q5)*sin(q6)*(cos(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) - cos(q3)*sin(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)));
    J(5,4) = -sin(q6)*(sin(q5)*(sin(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) + cos(q3)*cos(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1))) - cos(q5)*sin(q3)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)));
    J(5,5) = sin(q6)*(cos(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) - cos(q3)*sin(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1))) + cos(q6)*(cos(q5)*(sin(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) + cos(q3)*cos(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1))) + sin(q3)*sin(q5)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)));
    J(5,6) = 0;

    J(6,0) = - cos(q7)*(sin(q5)*(sin(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) - cos(q3)*cos(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2))) + cos(q5)*sin(q3)*(cos(q1)*cos(q2) - sin(q1)*sin(q2))) - sin(q7)*(sin(q6)*(cos(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) + cos(q3)*sin(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2))) + cos(q6)*(cos(q5)*(sin(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) - cos(q3)*cos(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2))) - sin(q3)*sin(q5)*(cos(q1)*cos(q2) - sin(q1)*sin(q2))));
    J(6,1) = - cos(q7)*(sin(q5)*(sin(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) - cos(q3)*cos(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2))) + cos(q5)*sin(q3)*(cos(q1)*cos(q2) - sin(q1)*sin(q2))) - sin(q7)*(sin(q6)*(cos(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) + cos(q3)*sin(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2))) + cos(q6)*(cos(q5)*(sin(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) - cos(q3)*cos(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2))) - sin(q3)*sin(q5)*(cos(q1)*cos(q2) - sin(q1)*sin(q2))));
    J(6,2) = sin(q7)*(cos(q6)*(cos(q3)*sin(q5)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) - cos(q4)*cos(q5)*sin(q3)*(cos(q1)*sin(q2) + cos(q2)*sin(q1))) + sin(q3)*sin(q4)*sin(q6)*(cos(q1)*sin(q2) + cos(q2)*sin(q1))) - cos(q7)*(cos(q3)*cos(q5)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) + cos(q4)*sin(q3)*sin(q5)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)));
    J(6,3) = cos(q7)*sin(q5)*(cos(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) - cos(q3)*sin(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1))) - sin(q7)*(sin(q6)*(sin(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) + cos(q3)*cos(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1))) - cos(q5)*cos(q6)*(cos(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) - cos(q3)*sin(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1))));
    J(6,4) = cos(q7)*(cos(q5)*(sin(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) + cos(q3)*cos(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1))) + sin(q3)*sin(q5)*(cos(q1)*sin(q2) + cos(q2)*sin(q1))) - cos(q6)*sin(q7)*(sin(q5)*(sin(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) + cos(q3)*cos(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1))) - cos(q5)*sin(q3)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)));
    J(6,5) = -sin(q7)*(sin(q6)*(cos(q5)*(sin(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) + cos(q3)*cos(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1))) + sin(q3)*sin(q5)*(cos(q1)*sin(q2) + cos(q2)*sin(q1))) - cos(q6)*(cos(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) - cos(q3)*sin(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1))));
    J(6,6) = cos(q7)*(sin(q6)*(cos(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) - cos(q3)*sin(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1))) + cos(q6)*(cos(q5)*(sin(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) + cos(q3)*cos(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1))) + sin(q3)*sin(q5)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)))) - sin(q7)*(sin(q5)*(sin(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) + cos(q3)*cos(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1))) - cos(q5)*sin(q3)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)));

    J(7,0) = (11*sin(q7)*(sin(q5)*(sin(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) - cos(q3)*cos(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2))) + cos(q5)*sin(q3)*(cos(q1)*cos(q2) - sin(q1)*sin(q2))))/125 - (79*cos(q2)*sin(q1))/250 - (79*cos(q1)*sin(q2))/250 - (33*sin(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)))/400 - (11*cos(q7)*(sin(q6)*(cos(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) + cos(q3)*sin(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2))) + cos(q6)*(cos(q5)*(sin(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) - cos(q3)*cos(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2))) - sin(q3)*sin(q5)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)))))/125 + (33*cos(q5)*(sin(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) - cos(q3)*cos(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2))))/400 + (33*cos(q3)*cos(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)))/400 - (33*sin(q3)*sin(q5)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)))/400;
    J(7,1) = (11*sin(q7)*(sin(q5)*(sin(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) - cos(q3)*cos(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2))) + cos(q5)*sin(q3)*(cos(q1)*cos(q2) - sin(q1)*sin(q2))))/125 - (79*cos(q2)*sin(q1))/250 - (79*cos(q1)*sin(q2))/250 - (33*sin(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)))/400 - (11*cos(q7)*(sin(q6)*(cos(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) + cos(q3)*sin(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2))) + cos(q6)*(cos(q5)*(sin(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) - cos(q3)*cos(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2))) - sin(q3)*sin(q5)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)))))/125 + (33*cos(q5)*(sin(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) - cos(q3)*cos(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2))))/400 + (33*cos(q3)*cos(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)))/400 - (33*sin(q3)*sin(q5)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)))/400;
    J(7,2) = (11*sin(q7)*(cos(q3)*cos(q5)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) + cos(q4)*sin(q3)*sin(q5)*(cos(q1)*sin(q2) + cos(q2)*sin(q1))))/125 + (11*cos(q7)*(cos(q6)*(cos(q3)*sin(q5)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) - cos(q4)*cos(q5)*sin(q3)*(cos(q1)*sin(q2) + cos(q2)*sin(q1))) + sin(q3)*sin(q4)*sin(q6)*(cos(q1)*sin(q2) + cos(q2)*sin(q1))))/125 - (33*cos(q4)*sin(q3)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)))/400 - (33*cos(q3)*sin(q5)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)))/400 + (33*cos(q4)*cos(q5)*sin(q3)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)))/400;
    J(7,3) = (33*cos(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)))/400 - (11*cos(q7)*(sin(q6)*(sin(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) + cos(q3)*cos(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1))) - cos(q5)*cos(q6)*(cos(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) - cos(q3)*sin(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)))))/125 - (33*cos(q5)*(cos(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) - cos(q3)*sin(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1))))/400 - (33*cos(q3)*sin(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)))/400 - (11*sin(q5)*sin(q7)*(cos(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) - cos(q3)*sin(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1))))/125;
    J(7,4) = (33*sin(q5)*(sin(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) + cos(q3)*cos(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1))))/400 - (11*sin(q7)*(cos(q5)*(sin(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) + cos(q3)*cos(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1))) + sin(q3)*sin(q5)*(cos(q1)*sin(q2) + cos(q2)*sin(q1))))/125 - (11*cos(q6)*cos(q7)*(sin(q5)*(sin(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) + cos(q3)*cos(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1))) - cos(q5)*sin(q3)*(cos(q1)*sin(q2) + cos(q2)*sin(q1))))/125 - (33*cos(q5)*sin(q3)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)))/400;
    J(7,5) = -(11*cos(q7)*(sin(q6)*(cos(q5)*(sin(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) + cos(q3)*cos(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1))) + sin(q3)*sin(q5)*(cos(q1)*sin(q2) + cos(q2)*sin(q1))) - cos(q6)*(cos(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) - cos(q3)*sin(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)))))/125;
    J(7,6) = - (11*cos(q7)*(sin(q5)*(sin(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) + cos(q3)*cos(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1))) - cos(q5)*sin(q3)*(cos(q1)*sin(q2) + cos(q2)*sin(q1))))/125 - (11*sin(q7)*(sin(q6)*(cos(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) - cos(q3)*sin(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1))) + cos(q6)*(cos(q5)*(sin(q4)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) + cos(q3)*cos(q4)*(cos(q1)*sin(q2) + cos(q2)*sin(q1))) + sin(q3)*sin(q5)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)))))/125;

    J(8,0) = 0;
    J(8,1) = 0;
    J(8,2) = 0;
    J(8,3) = 0;
    J(8,4) = 0;
    J(8,5) = 0;
    J(8,6) = 0;

    J(9,0) = 0;
    J(9,1) = 0;
    J(9,2) = 0;
    J(9,3) = 0;
    J(9,4) = 0;
    J(9,5) = 0;
    J(9,6) = 0;

    J(10,0) = 0;
    J(10,1) = 0;
    J(10,2) = 0;
    J(10,3) = 0;
    J(10,4) = 0;
    J(10,5) = 0;
    J(10,6) = 0;

    J(11,0) = 0;
    J(11,1) = 0;
    J(11,2) = 0;
    J(11,3) = 0;
    J(11,4) = 0;
    J(11,5) = 0;
    J(11,6) = 0;

    J_inv = J.completeOrthogonalDecomposition().pseudoInverse();

    return J_inv;

}

VectorXd makeVec(MatrixXd temp){
    MatrixXd pose_diff_1 = temp.topRows(3);
    MatrixXd pose_diff = pose_diff_1.transpose();
    VectorXd pose_diff_vec(Map<VectorXd>(pose_diff.data(), pose_diff.cols()*pose_diff.rows()));

    return pose_diff_vec;
}

vector<double> q_converter(Vector<double, 7> arr){
    vector<double> arr_ret;
    for(int i = 0; i < 7; i++){
        arr_ret.push_back(arr[i]);
    }
    return arr_ret;
}


Vector<double, 7> mod(Vector<double, 7> arr){
    Vector<double, 7> arr_ret;
    double twoPi = 2.0 * 3.141592865358979;
    for(int i = 0; i < 7; i++){
        arr_ret[i] = arr[i] - twoPi * floor( arr[i] / twoPi);
    }
    return arr_ret;
}

void SubPub::callback(const sensor_msgs::JointState& msg)
{
    ros::Rate loop_rate(1000);
    Vector<double, 7> q_desired;
    q_desired << 1,1,1,1,1,1,1;

    if(check == 0) {
        for (int i = 0; i < 7; i++) {
            q_joint[i] = msg.position[i];
            cout << q_joint[i];
        }
        check++;
    }
    else {
        /*Vector<double, 7> q_actual = q_joint;

        Matrix4d pose_desired = fk(q_desired);
        Matrix4d pose_actual = fk(q_actual);
        Matrix4d pose_diff = pose_desired - pose_actual;
        VectorXd pose_diff_vec = makeVec(pose_diff);

        MatrixXd j_inv;
        VectorXd q_diff;

        while (abs(pose_diff_vec.maxCoeff()) > 0.001) {
            j_inv = jacobian_inv(q_actual);
            q_diff = j_inv * pose_diff_vec;
            q_actual = q_actual + (q_diff * 0.1);
            pose_actual = fk(mod(q_actual));
            pose_diff = pose_desired - pose_actual;
            pose_diff_vec = makeVec(pose_diff);
        }*/

        vector<double> q_msg;
        //q_msg = q_converter(mod(q_actual));
        q_msg = {1,1,1,1,1,1,1};

        std_msgs::Float64MultiArray qMsg;
        qMsg.data.clear();
        qMsg.data = q_msg;
        pubObject.publish(qMsg);
        loop_rate.sleep();

    }


}


int main(int argc, char **argv)
{

    ros::init(argc, argv, "subpub_ik");
    SubPub subPubObject("/joint_states", "/q_desired", 1);
    ros::spin();
    return 0;
}