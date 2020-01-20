#include <pthread.h>
#include <iostream>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_workbench_toolbox/dynamixel_workbench.h"

#include "SimControll.h"

#include "LIPMWalk.h"
// #include "Walking.h"
// #include "CPWalk.h"
// #include "CPWalking.h"
// #include "CPWalking1.h"
// #include "CPWalking2.h"
#include "CPWalking5.h"
#include "Kinematics.h"

#include "JY901.h"
#include "SimpleSerial.h"
#include "serial/serial.h"

#include <std_msgs/Float64MultiArray.h>
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int64.h"

#include "Kalman.h"

using namespace GaitManager;
using namespace std;

int stepcountTarget;
// ///////////////////////////////LIPM//////////////////////////////////////////
LIPMWalk mWalk;
// CPWalk* cpWalk;
// CPWalking* cpWalk;
// CPWalking1* cpWalk;
// CPWalking2* cpWalk;
CPWalking5* cpWalk;

#define SYNC_WRITE_HANDLER_FOR_GOAL_POSITION 0
#define SYNC_READ_HANDLER_FOR_PRESENT_POSITION 0
#define SYNC_READ_HANDLER_FOR_PRESENT_VELOCITY 1
#define BULK_READ_PRESENT_POSITION_ADDR 36
#define BULK_READ_PRESENT_POSITION_LEN 2
#define RIGHT_FSR_ID 112
#define LEFT_FSR_ID 111
#define FSR_ADDR 90
#define FSR_ADDR_LEN 4

DynamixelWorkbench dxl_wb;
std::vector<double> motoDataValuePre = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                        0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

uint8_t idArray[30];
uint8_t idCnt = 0;

uint8_t scanRange = 25;
uint8_t scannedId[30];
uint8_t dxlIdCnt = 0;

double measuredJointPos[30];
double measuredJointPWM[30];
double measuredJointCurrent[30];
double measuredJointVel[30];

Kalman* measuredJointFilter[30];

void jointFilter();
void jointFilterInit();

// int assembleOffset[20] = {0,20,100,20,50,30,
// 							  0,0,-50,0,0,-40,
// 0,0,0,
// 0,0,0,	0,0};
// int assembleOffset[20] = {0,20,60,20,40,35,
// 							  0,0,-20,-40,-30,-35,
// 0,0,0,
// 0,0,0,	0,0};
// int assembleOffset[20] = {0,20,60,20,40,45,
// 							  -20,0,-20,-40,-30,-45,
// 0,0,0,
// 0,0,0,	0,0};

// int assembleOffset[20] = {0,20,120,20,40,45,
// 							  -20,0,-80,-40,-30,-45,
// 0,0,0,
// 0,0,0,	0,0};//3.35
// int assembleOffset[20] = {0,20,180,20,40,45,
// 							  -20,0,-140,-40,-40,-45,
// 0,0,0,
// 0,0,0,	0,0};//add mass to 5.38
// int assembleOffset[20] = {0,-15,190,20,40,10,
// 							  -20,33,-140,-40,-40,-15,
// 0,0,0,
// 0,0,0,	0,0};//add mass to 5.38
// int assembleOffset[20] = {0,-15,190,20,40,5,
// 							  -20,33,-145,-40,-40,-10,
// 0,0,0,
// 0,0,0,	0,0};//add mass to 5.38 openloop1 com
// int assembleOffset[20] = {0,-30,190,20,40,5,
// 							  -20,48,-145,-40,-40,-10,
// 0,0,0,
// 0,0,0,	0,0};//add mass to 5.38 openloop1 com
// int assembleOffset[20] = {0,-20,190,20,40,20,
// 							  -20,38,-140,-40,-40,-20,
// 0,0,0,
// 0,0,0,	0,0};//add mass to 3.38 openloop1 com
// int assembleOffset[20] = {0,-25,190,20,40,20,
// 							  20,38,-135,-40,-40,5,
// 0,0,0,
// 0,0,0,	0,0};//add mass to 3.38 openloop1 com
// int assembleOffset[20] = {0,-20,190,20,40,25,
// 							  20,38,-135,-40,-40,-5,
// 0,0,0,
// 0,0,0,	0,0};//add mass to 3.38 openloop1 com
// int assembleOffset[20] = {0,-20,190,20,40,30,
// 							  20,38,-135,-40,-40,-7,
// 0,0,0,
// 0,0,0,	0,0};//add mass to 3.38 openloop1 com  fsr
// int assembleOffset[20] = {0,-15,190,20,40,10,
// 							  -20,33,-140,-40,-40,-20,
// 0,0,0,
// 0,0,0,	0,0};//add mass to 5.38
// int assembleOffset[20] = {0,-15,160,20,40,30,
// 							  -20,33,-110,-40,-40,-20,
// 0,0,0,
// 0,0,0,	0,0};//add mass to 3.38
// int assembleOffset[20] = {0,-15,185,25,35,25,
// 							  -20,33,-140,-40,-40,-5,
// 0,0,0,
// 0,0,0,	0,0};//add mass to 3.38	  footSeprate=0.051*2;  //TimeCon_y=0.95
// int assembleOffset[20] = {0,-15,180,25,35,28,
// 							  -20,33,-148,-45,-45,-15,
// 0,0,0,
// 0,0,0,	0,0};//add mass to 3.38;
// int assembleOffset[20] = {0,-15,185,25,30,25,
// 							  -20,33,-150,-45,-45,-15,
// 0,0,0,
// 0,0,0,	0,0};//add mass to 3.38;
// int assembleOffset[20] = {0,-15,185,25,30,25,
// 							  -20,33,-150,-45,-45,-10,
// 0,0,0,
// 0,0,0,	0,0};//add mass to 3.38;	cap
// int assembleOffset[20] = {0,-15,185,25,30,25,
// 							  -20,33,-150,-45,-45,-10,
// 0,0,0,
// 0,0,0,	0,0};//add mass to 5.38;	cap
// int assembleOffset[20] = {0,-15,185,25,30,35,
// 							  -20,33,-150,-45,-45,-25,
// 0,0,0,
// 0,0,0,	0,0};//add mass to 3.38;	cap 1.0s
// int assembleOffset[20] = {0,-15,185,25,30,35,
// 							  -20,33,-150,-45,-45,-15,
// 0,0,0,
// 0,0,0,	0,0};//add mass to 3.38;	60cmLIPM
// int assembleOffset[20] = {0,-15,185,25,30,30,
// 							  -20,33,-150,-45,-45,-15,
// 0,0,0,
// 0,0,0,	0,0};//add mass to 3.38;	60cmLIPM
// int assembleOffset[20] = {0,-15,185,25,30,25,
// 							  -20,33,-150,-45,-45,-10,
// 0,0,0,
// 0,0,0,	0,0};//add mass to 5.38;	60cmLIPM
// int assembleOffset[20] = {0,-15,185,25,30,40,
// 							  -20,33,-150,-45,-45,-10,
// 0,0,0,
// 0,0,0,	0,0};//add mass to 5.38;  1.0s	60cmLIPM
// int assembleOffset[20] = {0,-15,185,25,30,20,//40
// 						-20,33,-150,-45,-45,-10, 0,0,0,
// 0,0,0,	0,0};//add mass to 5.38;  0.6s	60cmLIPM

// int assembleOffset[22] = {71,118,128,-72,78,20,   -12,23,38,20,-102,8,
// 0,210,0, -136,0,97,0,0,  0,35};//###### Wed Aug 21 16:44:55 CST 2019  last
int assembleOffset[22] = {
    71, 128, 128, -72,  78, 28, -52, 23, 38, 20, -102, 23,
    0,  210, 0,   -136, 0,  97, 0,   0,  0,  35};  //###### Wed Aug 21 16:44:55
                                                   // CST 2019
// int assembleOffset[22] = {71,118,128,-92,38,20,   -12,23,38,20,-62,8,
// 0,210,0, -136,0,97,0,0,  0,35};//###### Wed Aug 21 16:44:55 CST 2019

// int assembleOffset[20] = {0,-20,190,20,40,20,
// 						  -20,38,-140,-40,-40,-8, 0,0,0,
// 0,0,0,	0,0};//add mass to 3.38	 fsr  NoDS

// int assembleOffset[20] = {0,20,260,20,40,45,
// 							  -20,0,-220,-40,-40,-45,
// 0,0,0,
// 0,0,0,	0,0};//add mass to 5.38	 squat
int angleDirection[20] = {1,  1,  -1, -1, 1, -1, 1, 1, 1, 1,
                          -1, -1, 1,  1,  1, 1,  1, 1, 1, 1};

bool initSDKHandlers(void);
bool initDxlWorkbench(void);

bool sendDataToDxl(void);
// void initSquat();
void squat();
// void squatTest();
void* control_thread(void* ptr);

void* control_thread_robot(void* ptr);

///////JY901/////////////
void* JY901_thread(void* ptr);
// SimpleSerial serial_JY901("/dev/ttyUSB0",115200);//JY901串口处理
// 启用JY901时取消注释

// serial::Serial serial_JY901("/dev/ttyUSB0", 115200,
// serial::Timeout::simpleTimeout(1000),
// 	serial::eightbits,serial::parity_none,serial::stopbits_one,serial::flowcontrol_none);
double JY901Roll, JY901Pitch;

// ///////////////////////////////ROS//////////////////////////////////////////
ros::Publisher jointPosTargetPub;
ros::Publisher jointPosMeasurePub;

std_msgs::Float64MultiArray jointPos;

ros::Publisher jointVelTargetPub;
ros::Publisher jointVelMeasurePub;

std_msgs::Float64MultiArray jointVel;

std_msgs::Float64MultiArray footTraj;

ros::Publisher cpref_pub;
ros::Publisher cpC_pub;

ros::Publisher copm_pub;
ros::Publisher copD_pub;
ros::Publisher copref_pub;

ros::Publisher comm_pub;
ros::Publisher comD_pub;
ros::Publisher comref_pub;
ros::Publisher comEsti_pub;

ros::Publisher comvm_pub;
ros::Publisher comvD_pub;
ros::Publisher comvref_pub;
ros::Publisher comvEsti_pub;

ros::Publisher LFootZ;
ros::Publisher RFootZ;
ros::Publisher footDisRef;
ros::Publisher footDis;

ros::Publisher leftFootTraj_pub;
ros::Publisher rightFootTraj_pub;

ros::Publisher contactState_pub;
ros::Publisher stepPhase_pub;

ros::Publisher JY901X;
ros::Publisher JY901Y;

void robotStatePublish();
void advertise();

void RobotStand(uint8_t* idGet, uint8_t idCnt, int32_t velocity);
bool BulkReadFromDxl();
bool SensorBulkRead(uint8_t* bulkReadID, uint8_t readCount,
                    uint16_t* bulkReadAddress, uint16_t* bulkReadLength,
                    int32_t* bulkReadData);
// ///////////////////////////////ROS//////////////////////////////////////////
