//
// Created by rnm on 05.05.22.
// Subscribe to the /jointstates_topic
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "math.h"


double** multiply (double *A , double *B) {
    double**  multi;
    //int row = sizeof(A)/sizeof (A[0]);
    int row = 4;
    //int col = sizeof(A[0])/sizeof(int);
    int col = 4;
    for (int i=0; i<row; ++i) {
        for (int j=0; j<col; ++j){
            for (int k=0; k<col; ++k ){

                //multi += A[i][k] * B[k][j] ;
            }

        }
    }
    return multi;
}

void transformationMatrix (double* joint) {
    double A1[4][4];
    double A2[4][4];
    double A3[4][4];
    double A4[4][4];
    double A5[4][4];
    double A6[4][4];
    double A7[4][4];

    for (int i=0; i<7 ; i++ ) {
        switch (i) {
            case '0':
                A1= {{cos(joint[i]),-sin (joint[i]),0, 0},
                {sin(joint[i]), cos (joint[i]), 0, 0},
                {0, 0, 1, 0.333},
                {0, 0, 0, 1}};
                break;

            case '1':
                A2 = {{cos(joint[i]),0, -sin (joint[i]), 0},
                {sin(joint[i]), 0, cos (joint[i]),0},
                {0, -1, 0, 0},
                {0, 0, 0, 1}};
                break;

            case '2':
                 A3=  {{cos(joint[i]),0, sin (joint[i]), 0},
                       {sin(joint[i]), 0, -cos (joint[i]), 0},
                       {0, 1, 0, 0.316},
                       {0, 0, 0, 1}};
                break;

            case '3':
                A4  =  {{cos(joint[i]),0, sin (joint[i]), 0.0825*cos(joint[i])},
                        {sin(joint[i]), 0, -cos (joint[i]), 0.0825*cos(joint[i])},
                        {0, 1, 0, 0},
                        {0, 0, 0, 1}};
                break;

            case '4':
                A5 =  {{cos(joint[i]),0, -sin (joint[i]), -0.0825*cos(joint[i])},
                {sin(joint[i]), 0, cos (joint[i]), -0.0825*cos(joint[i])},
                {0, -1, 0, 0},
                {0, 0, 0, 1}};
                break;

            case '5':
                A6 =  {{cos(joint[i]),0, sin (joint[i]), 0},
                {sin(joint[i]), 0, -cos (joint[i]), 0},
                {0, 1, 0, 0},
                {0, 0, 0, 1}};
                break;
            case '6':
                A7 =  {{cos(joint[i]),0, sin (joint[i]), 0.088*cos(joint[i])},
                {sin(joint[i]), 0, -cos (joint[i]), 0.088*cos(joint[i])},
                {0, 1, 0, 0},
                {0, 0, 0, 1}};
                break;

        }

    }
    double T20[][] = multiply (A1,A2);
    double T42[][] = multiply (A3,A4);
    double T64[][]= multiply (A5,A6);
    double T74[][] = mutliply (T64,A7);
    double T40[][] = multiply (T20,T42);
    double T70[][] = multpily (T40,T74);
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
    ros::NodeHandle nh(~);

    std::string topic_name;
    int queue_size;
    nh.getParam ("topic_name", topic_name);
    nh.getParam ("queue_size", queue_size);
    ros::Subscriber sub = nh.subscribe("/joint_states", 10, callBack);
    ros::spin();

    return 0;
}

