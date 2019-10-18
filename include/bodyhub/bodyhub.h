#ifndef BODYHUB_H
#define BODYHUB_H

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <signal.h>
#include <sys/time.h>  //int gettimeofday(struct timeval *, struct timezone *);
#include <termios.h>   //struct termios
#include <yaml-cpp/yaml.h>
#include <boost/thread.hpp>
#include <queue>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_workbench_toolbox/dynamixel_workbench.h"

#include "bodyhub/JointControlPoint.h"
#include "bodyhub/SensorRawData.h"
#include "bodyhub/ServoPositionAngle.h"
#include "bodyhub/SrvInstRead.h"
#include "bodyhub/SrvInstWrite.h"
#include "bodyhub/SrvServoAllRead.h"
#include "bodyhub/SrvServoAllWrite.h"
#include "bodyhub/SrvServoRead.h"
#include "bodyhub/SrvServoWrite.h"
#include "bodyhub/SrvState.h"
#include "bodyhub/SrvString.h"
#include "bodyhub/SrvSyncWrite.h"
#include "bodyhub/SrvTLSstring.h"
#include "bodyhub/SrvSensorControl.h"
#include "bodyhubStatus.h"
#include "ros/callback_queue.h"
#include "ros/ros.h"
#include "std_msgs/UInt16.h"

#define DEBUG

#define PI acos(-1)
#define SYNC_WRITE_HANDLER_FOR_GOAL_POSITION 0
#define SYNC_WRITE_HANDLER_FOR_GOAL_VELOCITY 1

uint8_t timerClosed = 1;  // timer 20ms 关闭
uint8_t scanRange = 199;
uint8_t scannedId[30];
uint8_t dxlIdCnt = 0;
uint8_t bodyhubState = 0;
uint8_t masterIDControl = 0;
std::string stateNewStr;

std::string offsetFile;
std::string dxlInitPoseFile;
std::string sensorNameIDFile;

double standPositionBuff[20] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

double offsetStorage[20] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

std::vector<double> motoDataAngleStore = {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0};  // 20个舵机下发数据记录存储
                                    // 共20个数据
std::vector<double> motoDataValueStore = {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0};  // 20个舵机下发数据记录存储
                                    // 共20个数据

std::map<std::string, uint8_t> sensorNameIDMap;  // <sensor, ID>
std::map<uint8_t, uint8_t *> sensorDataMap;      // <sensorID, data>

struct MotoPoint {
  double position;  // radian
  double velocity;
  double acceleration;
};

struct SensorModule {
  std::string sensorName;
  uint16_t sensorID;
  uint16_t startAddr;
  uint8_t length;
};


//传感器控制部分
#define CONTROL_DATA_SIZE_MAX 128

typedef struct
{
  uint16_t id;
  uint16_t addr;
  uint16_t lenght;
  uint8_t data[CONTROL_DATA_SIZE_MAX];
}SensorControl_t;

std::queue<SensorControl_t> SensorControlQueue;


DynamixelWorkbench dxlTls;

std::queue<bodyhub::JointControlPoint> motoQueue;
std::queue<bodyhub::JointControlPoint> headCtrlQueue;

//创建互斥锁
pthread_mutex_t mtxMo;
pthread_mutex_t mtxHe;
pthread_mutex_t mtxSL;
pthread_mutex_t MtuexSensorControl;

std_msgs::UInt16 budyhubStateMsg;

ros::Publisher StatusPub;
ros::Publisher ServoPositionPub;
ros::Publisher SensorRawDataPub;
ros::ServiceServer StateService;
ros::ServiceServer MasterIDService;
ros::ServiceServer GetStatusService;
ros::ServiceServer GetJointAngleService;

ros::ServiceServer InstReadService;
ros::ServiceServer InstWriteService;
ros::ServiceServer SyncWriteService;
ros::ServiceServer SetLockStateService;
ros::ServiceServer SetLockStateAllService;
ros::ServiceServer GetLockStateAllService;
ros::ServiceServer SetTarPositionService;
ros::ServiceServer SetTarPositionAllService;
ros::ServiceServer GetPositionAllService;
ros::ServiceServer RegistSensorService;
ros::ServiceServer SetSensorService;
ros::ServiceServer DeleteSensorService;
ros::ServiceServer SensorControlService;

std::list<SensorModule *> sensorsList;

void ClearTimerQueue(void);
void addSensorModule(SensorModule *tempSensor);
void removeSensorModule(SensorModule *tempSensor);
bool ServoBulkRead(uint8_t *bulkReadID, uint8_t readCount, std::string itemName,
                   int32_t *bulkReadData);
bool SensorBulkRead(uint8_t *bulkReadID, uint8_t readCount,
                    uint16_t *bulkReadAddress, uint16_t *bulkReadLength,
                    int32_t *bulkReadData);

bool SensorBulkWrite(uint8_t WriteCount, uint8_t *bulkWriteID,uint16_t *bulkWriteAddress, 
                    uint16_t *bulkWriteLenght, int32_t *bulkWriteData);

bool SensorWrite(uint8_t WriteCount, uint8_t *WriteID,uint16_t *WriteAddress, 
                    uint16_t *WriteLenght, uint8_t WriteData[][CONTROL_DATA_SIZE_MAX]);

#endif