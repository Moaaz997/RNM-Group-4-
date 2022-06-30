//
// Created by sherif on 20.06.22.
//

#include "subsubpub.h"
#include "subpub.h"

#include <Eigen/Dense>
#include <Eigen/QR>
#include <cmath>
#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "sensor_msgs/JointState.h"


using namespace std;
using namespace Eigen;

double joint_desired[7];
double joint_vars[7];
int check1 = 0;
int check = 0;
int trajCount = 0;
double timef = 10.0;
double q_test[7] = {1,1,1,1,1,1,1};

MatrixXd coef_mat_gen(double q_i[], double q_f[]){
    double t0 = 0.0;
    double tf = timef;

    Matrix<double, 6, 6> t_mat {
        {1, t0, pow(t0,2), pow(t0,3), pow(t0,4), pow(t0,5)},
        {0, 1, 2*t0, 3*pow(t0,2), 4*pow(t0,3), 5*pow(t0,4)},
        {0, 0, 2, 6*t0, 12*pow(t0,2), 20*pow(t0,3)},
        {1, tf, pow(tf,2), pow(tf,3), pow(tf,4), pow(tf,5)},
        {0, 1, 2*tf, 3*pow(tf,2), 4*pow(tf,3), 5*pow(tf,4)},
        {0, 0, 2, 6*tf, 12*pow(tf,2), 20*pow(tf,3)},
    };

    Vector<double, 6> joint_info;
    Matrix<double, 7, 6> coef_mat;
    MatrixXd temp;

    for(int i = 0; i < 7; i++){
        joint_info << joint_vars[i],0,0,q_f[i],0,0;
        temp = t_mat.inverse()*joint_info;
        for(int j = 0; j < 6; j++){
            coef_mat(i,j)=temp(j);
        }
    }
    return coef_mat;
}

vector<double> traj_gen(double q_i[], double q_f[]){
    MatrixXd coef_mat;
    coef_mat = coef_mat_gen(q_i,q_f);

    vector<double> traj;

    double k = 0.0;
    int count = 0;

    while(k <= timef) {
        for (int i = 0; i < 7; i++) {
            traj.push_back(coef_mat(i,0) + coef_mat(i,1)*k + coef_mat(i,2)* pow(k,2) + coef_mat(i,3)* pow(k,3) + coef_mat(i,4)* pow(k,4) + coef_mat(i,5)* pow(k,5));
            count++;
        }
        k = k+0.001;
    }

    return traj;
}

/*void SubSubPub::callback1(const std_msgs::Float64MultiArray &msg) {
    if(check1 == 0){
        for (int i = 0; i < 7; i++) {
            joint_desired[i] = msg.data[i];
        }
        check++;
    }

}

void SubSubPub::callback2(const sensor_msgs::JointState& msg)
{
    ros::Rate loop_rate(1000);

    if(check == 0) {
        for (int i = 0; i < 7; i++) {
            joint_vars[i] = msg.position[i];
        }
        qtraj_total = traj_gen(joint_vars, joint_desired);
        check++;
    }
    else if(check == 1){
        vector<double> qtraj_section;
        while (trajCount < qtraj_total.size() ){
            if(ros::ok()) {
                for (int i = 0; i < 7; i++) {
                    qtraj_section.push_back(qtraj_total[trajCount]);
                    trajCount++;
                }
                std_msgs::Float64MultiArray trajMsg;
                trajMsg.data.clear();
                trajMsg.data = qtraj_section;
                pubObject.publish(trajMsg);
                qtraj_section.clear();
            }
        }

    }
}*/
void SubPub::callback(const sensor_msgs::JointState& msg) {
    ros::Rate loop_rate(1000);

    if (check == 0) {
        for (int i = 0; i < 7; i++) {
            joint_vars[i] = msg.position[i];
        }
        qtraj_total = traj_gen(joint_vars, q_test);
        check++;
    } else if (check == 1) {
        vector<double> qtraj_section;
        while (trajCount < qtraj_total.size()) {
            if (ros::ok()) {
                for (int i = 0; i < 7; i++) {
                    qtraj_section.push_back(qtraj_total[trajCount]);
                    trajCount++;
                }
                std_msgs::Float64MultiArray trajMsg;
                trajMsg.data.clear();
                trajMsg.data = qtraj_section;
                loop_rate.sleep();
                pubObject.publish(trajMsg);
                qtraj_section.clear();
            }
        }

    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "subpub_traj");
    //SubSubPub subPubObject("/q_desired","/joint_states", "joint_position_example_controller_sim/joint_command", 1);
    SubPub subPubObject("/joint_states", "joint_position_example_controller_sim/joint_command", 1);
    ros::spin();
    return 0;
}



