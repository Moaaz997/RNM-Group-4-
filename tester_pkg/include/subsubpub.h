//
// Created by sherif on 30.06.22.
//

#ifndef TESTER_PKG_SUBSUBPUB_H
#define TESTER_PKG_SUBSUBPUB_H

#include "ros/ros.h"
#include "string"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64MultiArray.h"


using namespace std;


class SubSubPub
{
public:
    SubSubPub(){}
    SubSubPub(string sub1TopicName, string sub2TopicName, string pubTopicName, int queueSize ){
        ros::Rate loop_rate(1);
        subObject = nH.subscribe(sub1TopicName,queueSize,&SubSubPub::callback1,this);
        loop_rate.sleep();
        subObject = nH.subscribe(sub2TopicName,queueSize,&SubSubPub::callback2,this);
        pubObject = nH.advertise<std_msgs::Float64MultiArray>(pubTopicName,queueSize);
    }

    void callback1(const std_msgs::Float64MultiArray& msg);
    void callback2(const sensor_msgs::JointState& msg);


protected:
    ros::Subscriber subObject;
    ros::Publisher pubObject;
    ros::NodeHandle nH;
    vector<double> qtraj_total;
};


#endif //TESTER_PKG_SUBSUBPUB_H
