//
// Created by sherif on 15.05.22.
//

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "math.h"
#include <Eigen/Dense>
#include <iostream>
using namespace Eigen;
using namespace std;

void transformationMatrix (double* joint) {
    MatrixXf A1(4,4);
    MatrixXf A2(4,4);
    MatrixXf A3(4,4);
    MatrixXf A4(4,4);
    MatrixXf A5(4,4);
    MatrixXf A6(4,4);
    MatrixXf A7(4,4);
    MatrixXf A8(4,4);


    for (int i=0; i<7 ; i++ ) {
        switch (i) {
            case '0':
                A1 << cos(joint[i]),-sin (joint[i]),0, 0,
                     sin(joint[i]), cos (joint[i]), 0, 0,
                     0, 0, 1, 0.333,
                     0, 0, 0, 1;
                break;

            case '1':
                A2 << cos(joint[i]),0, -sin (joint[i]), 0,
                      sin(joint[i]), 0, cos (joint[i]),0,
                      0, -1, 0, 0,
                      0, 0, 0, 1;
                break;

            case '2':
                A3 << cos(joint[i]),0, sin (joint[i]), 0,
                      sin(joint[i]), 0, -cos (joint[i]), 0,
                      0, 1, 0, 0.316,
                      0, 0, 0, 1;
                break;

            case '3':
                A4  << cos(joint[i]),0, sin (joint[i]), 0.0825*cos(joint[i]),
                        sin(joint[i]), 0, -cos (joint[i]), 0.0825*cos(joint[i]),
                        0, 1, 0, 0,
                        0, 0, 0, 1;
                break;

            case '4':
                A5 << cos(joint[i]),0, -sin (joint[i]), -0.0825*cos(joint[i]),
                       sin(joint[i]), 0, cos (joint[i]), -0.0825*cos(joint[i]),
                       0, -1, 0, 0,
                       0, 0, 0, 1;
                break;

            case '5':
                A6 << cos(joint[i]),0, sin (joint[i]), 0,
                       sin(joint[i]), 0, -cos (joint[i]), 0,
                       0, 1, 0, 0,
                       0, 0, 0, 1;
                break;
            case '6':
                A7 << cos(joint[i]),0, sin (joint[i]), 0.088*cos(joint[i]),
                       sin(joint[i]), 0, -cos (joint[i]), 0.088*cos(joint[i]),
                       0, 1, 0, 0,
                       0, 0, 0, 1;
                break;

        }

    }
    MatrixXf T70 (4,4);
    T70 << A1*A2*A3*A4*A5*A6*A7 ;
    ROS_INFO_STREAM(T70);

}

void callBack (const sensor_msgs::JointState& msg)
{
    double position[6];

    std::cout << "Joints: ";
    for (int i=0; i<7 ; ++i ) {
        position[i] = msg.position[i];
        std:: cout << position[i] << " " ;
    }
    transformationMatrix (position);
    std::cout << "/n/n";
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "panda_joint_reader_node");
    ros::NodeHandle nh;

    std::string topic_name;
    int queue_size;
    nh.getParam ("topic_name", topic_name);
    nh.getParam ("queue_size", queue_size);
    ros::Subscriber sub = nh.subscribe("/joint_states", 10, callBack);
    ros::spin();

    return 0;
}