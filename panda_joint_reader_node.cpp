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

void transformationMatrix (MatrixXf joint) {
    Matrix4f A1;
    Matrix4f A2;
    Matrix4f A3;
    Matrix4f A4;
    Matrix4f A5;
    Matrix4f A6;
    Matrix4f A7;
    Matrix4f A8;


    for (int i=0; i<7 ; i++ ) {
        switch (i) {
            case '0':
                A1 << cos(joint(i)),-sin (joint(i)),0, 0,
                        sin(joint(i)), cos (joint(i)), 0, 0,
                        0, 0, 1, 0.333;
                        0, 0, 0, 1;
                break;

            case '1':
                A2 << cos(joint(i)),0, -sin (joint(i)), 0,
                        sin(joint(i)), 0, cos (joint(i)),0,
                        0, -1, 0, 0;
                        0, 0, 0, 1;
                break;

            case '2':
                A3 << cos(joint(i)),0, sin (joint(i)), 0,
                        sin(joint(i)), 0, -cos (joint(i)), 0,
                        0, 1, 0, 0.316,
                        0, 0, 0, 1;
                break;

            case '3':
                A4  << cos(joint(i)),0, sin (joint(i)), 0.0825*cos(joint(i)),
                        sin(joint(i)), 0, -cos (joint(i)), 0.0825*cos(joint(i)),
                        0, 1, 0, 0,
                        0, 0, 0, 1;
                break;

            case '4':
                A5 << cos(joint(i)),0, -sin (joint(i)), -0.0825*cos(joint(i)),
                        sin(joint(i)), 0, cos (joint(i)), -0.0825*cos(joint(i)),
                        0, -1, 0, 0,
                        0, 0, 0, 1;
                break;

            case '5':
                A6 << cos(joint(i)),0, sin (joint(i)), 0,
                        sin(joint(i)), 0, -cos (joint(i)), 0,
                        0, 1, 0, 0,
                        0, 0, 0, 1;
                break;
            case '6':
                A7 << cos(joint(i)),0, sin (joint(i)), 0.088*cos(joint(i)),
                        sin(joint(i)), 0, -cos (joint(i)), 0.088*cos(joint(i)),
                        0, 1, 0, 0,
                        0, 0, 0, 1;
                break;

        }

    }
    Matrix4f T70 ;
    T70 << A1*A2*A3*A4*A5*A6*A7 ;
    ROS_INFO_STREAM(T70);

}

void callBack (const sensor_msgs::JointState& msg)
{
    MatrixXf position(6,1);

    std::cout << "Joints: ";
    for (int i=0; i<7 ; ++i ) {
        position(i,1) = msg.position[i];
        std:: cout << position(i,1) << " " ;
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
