#include "SimControll.h"
// #include "Util.h"

namespace SimControll {

ros::Publisher JointCmdPub;
ros::Publisher SimStartPub;
ros::Publisher SimStopPub;
ros::Publisher SimPausePub;
ros::Publisher SimEnSyncPub;
ros::Publisher SimTrigNextPub;
ros::Subscriber SimStepDoneSub;
ros::Subscriber SimStateSub;

pthread_mutex_t mtxJV;

bool stepDone = false;   //一步仿真完成标志
bool simEnable = false;  //仿真启动标志
int8_t simState = 0;
std_msgs::Bool simCtrMsg;
std::queue<std::vector<double>> jointCmdQueue;

void simInit(ros::NodeHandle nh) {
  SimStepDoneSub = nh.subscribe<std_msgs::Bool>("simulationStepDone", 1,
                                                &SimStepDoneCallback);
  SimStateSub =
      nh.subscribe<std_msgs::Int32>("simulationState", 1, &SimStateCallback);

  JointCmdPub = nh.advertise<std_msgs::Float64MultiArray>("joint_command", 1);
  SimStartPub = nh.advertise<std_msgs::Bool>("startSimulation", 1);
  SimStopPub = nh.advertise<std_msgs::Bool>("stopSimulation", 1);
  SimPausePub = nh.advertise<std_msgs::Bool>("pauseSimulation", 1);
  SimEnSyncPub = nh.advertise<std_msgs::Bool>("enableSyncMode", 1);
  SimTrigNextPub = nh.advertise<std_msgs::Bool>("triggerNextStep", 1);

  //等待发布者与接收者建立连接
  ROS_INFO("Waiting for connection with Vrep......");
  while (ros::ok() && (SimStartPub.getNumSubscribers() <= 0 ||
                       SimEnSyncPub.getNumSubscribers() <= 0 ||
                       SimTrigNextPub.getNumSubscribers() <= 0))
    ;
  ROS_INFO("Connection with Vrep completed!!!");
  simCtrMsg.data = 1;               //仿真控制变量
  SimEnSyncPub.publish(simCtrMsg);  //开启vrep同步模式
  SimStartPub.publish(simCtrMsg);   //开始vrep仿真

  pthread_mutex_init(&mtxJV, NULL);
}

void SimStepDoneCallback(const std_msgs::Bool::ConstPtr &msg) {
  if (msg->data) stepDone = true;
}

void SimStateCallback(const std_msgs::Int32::ConstPtr &msg) {
  // pause--2 start--1
  simState = msg->data;
  // ROS_INFO("Simulation state update: %d", msg->data);
}

void updateJointCmdQueue(const std::vector<double> &jointPositions) {
  pthread_mutex_lock(&mtxJV);
  jointCmdQueue.push(jointPositions);  // radian vector
  pthread_mutex_unlock(&mtxJV);
}

void simStart() {
  simCtrMsg.data = 1;              //仿真控制变量
  SimStartPub.publish(simCtrMsg);  //开始vrep仿真
  ROS_INFO("Simulation in Vrep started!");
}

void simStop() {
  simCtrMsg.data = 1;             //仿真控制变量
  SimStopPub.publish(simCtrMsg);  //停止vrep仿真
  ROS_INFO("Simulation in Vrep stopped!");
}

void simPause() {
  simCtrMsg.data = 1;              //仿真控制变量
  SimPausePub.publish(simCtrMsg);  //暂停vrep仿真
  ROS_INFO("Simulation in Vrep paused!");
}

void simulateThread() {
  ROS_INFO("simulateThread initialized!");
  bool jointCmdReady = false;
  std_msgs::Float64MultiArray jointCmd;

  while (ros::ok() && SimControll::simEnable) {
    if (!jointCmdQueue.empty()) {
      pthread_mutex_lock(&mtxJV);
      jointCmd.data.assign(jointCmdQueue.front().begin(),
                           jointCmdQueue.front().end());
      jointCmdQueue.pop();
      pthread_mutex_unlock(&mtxJV);

      jointCmdReady = true;
    }

    if (jointCmdReady) {
      // Vrep仿真一步,大约要150ms才能收到simulationStepDone消息
      stepDone = false;
      SimTrigNextPub.publish(simCtrMsg);
      while (JointCmdPub.getNumSubscribers() <= 0)
        ;
      JointCmdPub.publish(jointCmd);  // 转发数据
      while (stepDone == false && ros::ok()) ros::spinOnce();
    }
  }
  ROS_INFO("simulateThread end!");
}

}  // namespace SimControll