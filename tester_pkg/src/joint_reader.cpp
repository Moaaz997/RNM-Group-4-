//
// Created by sherif on 19.06.22.
//

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "iostream"

using namespace std;
double joint_vars[7];

void callback(const sensor_msgs::JointState& msg)
{
    for(int i = 0; i < 7; i++)
    {
        joint_vars[i] = msg.position[i] ;
        cout << joint_vars[i];
    }
}


int main(int argc, char **argv)
{

    ros::init(argc, argv, "subscriber");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/joint_states", 1000, callback);
    ros::spin();
    return 0;
}