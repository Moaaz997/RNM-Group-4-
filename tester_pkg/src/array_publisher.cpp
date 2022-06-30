//
// Created by sherif on 19.06.22.
//
#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "publisher");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<std_msgs::Float64MultiArray>("joint_position_example_controller_sim/joint_trajectory_command", 1000);
    ros::Rate loop_rate(10);

    while (ros::ok()){

        std_msgs::Float64MultiArray msg;
        std::vector<double> test_array = {1,1,1,1,1,1,1};
        msg.data = test_array ;

        //ROS_INFO("%s", msg.data.c_str());
        chatter_pub.publish(msg);
        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}



