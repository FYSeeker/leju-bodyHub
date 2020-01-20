#ifndef SIMCONTROLL_H
#define SIMCONTROLL_H

#include <ros/ros.h>
#include <iostream>
#include <queue>
#include "std_msgs/Bool.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Int32.h"

namespace SimControll {

extern bool simEnable;
extern int8_t simState;

void simInit(ros::NodeHandle nh);
void simStart();
void simStop();
void simPause();
void simulateThread();
void updateJointCmdQueue(const std::vector<double> &jointPositions);
void SimStepDoneCallback(const std_msgs::Bool::ConstPtr &msg);
void SimStateCallback(const std_msgs::Int32::ConstPtr &msg);
}  // namespace SimControll

#endif