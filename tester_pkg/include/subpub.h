//
// Created by sherif on 28.06.22.
//

#ifndef TESTER_PKG_SUBPUB_H
#define TESTER_PKG_SUBPUB_H

#include "ros/ros.h"
#include "string"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64MultiArray.h"


using namespace std;


class SubPub
{
public:
    SubPub(){}
    SubPub(string subTopicName, string pubTopicName, int queueSize ){
        subObject = nH.subscribe(subTopicName,queueSize,&SubPub::callback,this);
        pubObject = nH.advertise<std_msgs::Float64MultiArray>(pubTopicName,queueSize);
    }

    void callback(const sensor_msgs::JointState& msg);


protected:
    ros::Subscriber subObject;
    ros::Publisher pubObject;
    ros::NodeHandle nH;
    vector<double> qtraj_total;
};



#endif //TESTER_PKG_SUBPUB_H
