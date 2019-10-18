#include "bodyhub/bodyhub.h"

void UpdateState(uint8_t stateNew);

void vectorOut(std::vector<double> &vector_in) {
  /* printf vector double */
  for (unsigned int i = 0; i < vector_in.size(); i++) {
    if (i == vector_in.size() - 1) {
      std::cout << vector_in[i] << std::endl;
    } else
      std::cout << vector_in[i] << ",";
  }
  std::cout << std::endl;
}

void ClearQueue(std::queue<bodyhub::JointControlPoint> &Qtempt) {
  std::queue<bodyhub::JointControlPoint> empty;
  swap(empty, Qtempt);
}

int getch() {
// 获取按键
#if defined(__linux__) || defined(__APPLE__)
  struct termios oldt, newt;
  int ch;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  ch = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return ch;
#elif defined(_WIN32) || defined(_WIN64)
  return _getch();
#endif
}

double Angle2Radian(double angle) { return (angle * PI) / 180; }
double Radian2Angle(double Radian) { return (Radian * 180) / PI; }


void bulkReadProcess()
{
  pthread_mutex_lock(&mtxSL);
  if (sensorsList.size()) 
  {
    bodyhub::SensorRawData sensorRawDataMsg;

    uint8_t readCount = sensorsList.size();
    uint8_t bulkReadID[readCount];
    uint16_t bulkReadAddress[readCount];
    uint16_t bulkReadLength[readCount];
    uint8_t readDataLength = 0;
    uint8_t idNum = 0;
    for (auto s_it = sensorsList.begin(); s_it != sensorsList.end();
          s_it++) {
      bulkReadID[idNum] = (*s_it)->sensorID;
      bulkReadAddress[idNum] = (*s_it)->startAddr;
      bulkReadLength[idNum] = (*s_it)->length;
      readDataLength += (*s_it)->length;
      idNum++;
    }

    int32_t bulkReadData[readDataLength];
    SensorBulkRead(bulkReadID, readCount, bulkReadAddress, bulkReadLength,
                    bulkReadData);

    // publish sensorData
    for (idNum = 0; idNum < readCount; idNum++) {
      sensorRawDataMsg.sensorReadID.push_back(bulkReadID[idNum]);
      sensorRawDataMsg.sensorStartAddress.push_back(bulkReadAddress[idNum]);
      sensorRawDataMsg.sensorReadLength.push_back(bulkReadLength[idNum]);
    }
    for (idNum = 0; idNum < readDataLength; idNum++)
      sensorRawDataMsg.sensorData.push_back(bulkReadData[idNum]);
    sensorRawDataMsg.sensorCount = readCount;
    sensorRawDataMsg.dataLength = readDataLength;
    SensorRawDataPub.publish(sensorRawDataMsg);
  }
  pthread_mutex_unlock(&mtxSL);
}


void SensorControlProcess()
{
  pthread_mutex_lock(&MtuexSensorControl);
  if(!SensorControlQueue.empty())
  {
    uint8_t size = SensorControlQueue.size();
    uint8_t WriteID[size];
    uint16_t WriteAddress[size];
    uint16_t WriteLenght[size];
    uint8_t WriteData[size][CONTROL_DATA_SIZE_MAX];
    uint8_t index = 0;
    SensorControl_t SensorControl;
    while(!SensorControlQueue.empty())
    {
      SensorControl = SensorControlQueue.front();
      WriteID[index] = SensorControl.id;
      WriteAddress[index] = SensorControl.addr;
      WriteLenght[index] = SensorControl.lenght+2;
      memcpy(WriteData[index], SensorControl.data, SensorControl.lenght);
      SensorControlQueue.pop();
      index++;
    }
    SensorWrite(size, WriteID, WriteAddress, WriteLenght, WriteData);
  }
  pthread_mutex_unlock(&MtuexSensorControl);
}


void threadTimer() {
  static struct timespec nextTime;
  struct timespec realTime;
  clock_gettime(CLOCK_MONOTONIC, &nextTime);

  const char *log = NULL;
  bool result = false;

  while (1) {
    // timeset
    nextTime.tv_sec += (nextTime.tv_nsec + 20 * 1000000) / 1000000000;
    nextTime.tv_nsec = (nextTime.tv_nsec + 20 * 1000000) % 1000000000;
    clock_gettime(CLOCK_MONOTONIC, &realTime);
    if (((double)nextTime.tv_sec * 1000.0 +
         (double)nextTime.tv_nsec * 0.001 * 0.001) <
        ((double)realTime.tv_sec * 1000.0 +
         (double)realTime.tv_nsec * 0.001 * 0.001))
      ROS_ERROR("timer20 timeout");
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &nextTime,
                    NULL);  // start run

    //下发舵机数据
    if ((motoQueue.size() > 0) || (headCtrlQueue.size() > 0)) {
      bodyhub::JointControlPoint jntCtrMsg;
      bodyhub::JointControlPoint headCtrMsg;
      bodyhub::ServoPositionAngle servoPositionsMsg;
      int32_t goalPosition[30];
      uint8_t idArray[30] = {0};
      uint8_t idCnt = 0;

      if (motoQueue.size() > 0) {
        pthread_mutex_lock(&mtxMo);
        jntCtrMsg = motoQueue.front();
        for (uint8_t idNum = 0; idNum < jntCtrMsg.positions.size(); idNum++) {
          motoDataAngleStore[idNum] =
              Radian2Angle(jntCtrMsg.positions.at(idNum));
          idArray[idCnt] = idNum + 1;

          goalPosition[idCnt] = dxlTls.convertRadian2Value(
              idArray[idCnt], jntCtrMsg.positions.at(idNum));
          motoDataValueStore[idNum] = goalPosition[idCnt];
          idCnt++;
        }
        motoQueue.pop();
        pthread_mutex_unlock(&mtxMo);
      }
      if (headCtrlQueue.size() > 0) {
        pthread_mutex_lock(&mtxHe);
        headCtrMsg = headCtrlQueue.front();
        if (idCnt > 18) idCnt = 18;
        for (uint8_t idNum = 0; idNum < headCtrMsg.positions.size(); idNum++) {
          motoDataAngleStore[idNum + 18] =
              Radian2Angle(headCtrMsg.positions.at(idNum));
          idArray[idCnt] = idNum + 18 + 1;  // ID19 ID20

          goalPosition[idCnt] = dxlTls.convertRadian2Value(
              idArray[idCnt], headCtrMsg.positions.at(idNum));
          motoDataValueStore[idNum + 18] = goalPosition[idCnt];
          idCnt++;
        }  // ID19 ID20
        headCtrlQueue.pop();
        pthread_mutex_unlock(&mtxHe);
      }

      // #ifdef DEBUG
      //   ROS_INFO("舵机下发：");
      //   vectorOut(motoDataAngleStore);
      //   vectorOut(motoDataValueStore);
      // #endif

      result = dxlTls.syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_POSITION, idArray,
                                idCnt, goalPosition, 1, &log);  //同步写指令
      if (result == false) {
        ROS_ERROR("SigHandler()---%s", log);
      }

      servoPositionsMsg.angle = motoDataAngleStore;
      ServoPositionPub.publish(servoPositionsMsg);
    }

    // sensorsList bulkRead
    else 
    {
      bulkReadProcess();
      SensorControlProcess();
    }
  }
}

void addSensorModule(SensorModule *tempSensor) {
  // check whether the nodule length is zero
  if (tempSensor->length == 0) {
    ROS_ERROR("sensorBulkReadLength is illegal.");
    delete tempSensor;
    return;
  }
  // check whether the module name already exists
  for (auto s_it = sensorsList.begin(); s_it != sensorsList.end(); s_it++) {
    if ((*s_it)->sensorName == tempSensor->sensorName) {
      (*s_it)->length = tempSensor->length;
      (*s_it)->startAddr = tempSensor->startAddr;

      sensorDataMap.erase(tempSensor->sensorID);
      sensorDataMap[tempSensor->sensorID] = new uint8_t[tempSensor->length];
      delete tempSensor;
      return;
    }
  }
  try {
    tempSensor->sensorID = sensorNameIDMap.at(tempSensor->sensorName);
  } catch (const std::out_of_range &e) {
    ROS_ERROR("%s -- [%s] sensorName is illegal.", e.what(),
              tempSensor->sensorName.c_str());
    return;
  }

  sensorsList.push_back(tempSensor);
  sensorsList.unique();

  sensorDataMap[tempSensor->sensorID] = new uint8_t[tempSensor->length];
#ifdef DEBUG
  for (const auto &sensorN : sensorsList)
    std::cout << sensorN->sensorName << std::endl;
#endif
}

void removeSensorModlue(SensorModule *tempSensor) {
  pthread_mutex_lock(&mtxSL);
  delete[] sensorDataMap[tempSensor->sensorID];
  sensorDataMap.erase(tempSensor->sensorID);
  sensorsList.remove(tempSensor);
  pthread_mutex_unlock(&mtxSL);
}

bool ServoBulkRead(uint8_t *bulkReadID, uint8_t readCount, std::string itemName,
                   int32_t *bulkReadData) {
  /* Dynamixels bulkread */
  const char *log = NULL;
  bool result = false;

  dxlTls.clearBulkReadParam();
  for (uint8_t idNum = 0; idNum < readCount; idNum++) {
    result = dxlTls.addBulkReadParam(bulkReadID[idNum], itemName.c_str(), &log);
    if (result == false) ROS_ERROR("ServoBulkRead# %s", log);
  }
  result = dxlTls.bulkRead(&log);
  if (result == false) {
    ROS_ERROR("ServoBulkRead$ %s\n", log);
    return false;
  }
  result = dxlTls.getBulkReadData(&bulkReadData[0], &log);
  if (result == false) {
    ROS_ERROR("ServoBulkRead# %s\n", log);
    return false;
  }
  return true;
}
bool SensorBulkRead(uint8_t *bulkReadID, uint8_t readCount,
                    uint16_t *bulkReadAddress, uint16_t *bulkReadLength,
                    int32_t *bulkReadData) {
  const char *log = NULL;
  bool result = false;

  dxlTls.clearBulkReadParam();
  for (uint8_t idNum = 0; idNum < readCount; idNum++) {
    result = dxlTls.addBulkReadParam(bulkReadID[idNum], bulkReadAddress[idNum],
                                     bulkReadLength[idNum], &log);
    if (result == false) ROS_ERROR("ServoBulkRead# %s", log);
  }
  result = dxlTls.bulkRead(&log);
  if (result == false) {
    ROS_ERROR("ServoBulkRead# %s\n", log);
    return false;
  }
  result = dxlTls.getSensorBulkReadData(&bulkReadID[0], readCount,
                                        &bulkReadAddress[0], &bulkReadLength[0],
                                        &bulkReadData[0], &log);
  if (result == false) {
    ROS_ERROR("ServoBulkRead# %s\n", log);
    return false;
  }

  // bulkReadData[] -> sensorDataMap
  uint8_t dataIndex = 0;
  for (uint8_t idNum = 0; idNum < readCount; idNum++) {
    for (uint8_t i = 0; i < bulkReadLength[idNum]; i++) {
      sensorDataMap[bulkReadID[idNum]][i] = bulkReadData[dataIndex];
      dataIndex++;
    }
  }
#ifdef DEBUG
  printf("\ndataIndex_totoal: %d  sensorsList BULKREAD:", dataIndex);
  for (uint8_t dataNum = 0; dataNum < dataIndex; dataNum++)
    printf(" %d", bulkReadData[dataNum]);
  printf("\n");
#endif

  return true;
}


bool SensorBulkWrite(uint8_t WriteCount, uint8_t *bulkWriteID,uint16_t *bulkWriteAddress, 
                    uint16_t *bulkWriteLenght, int32_t *bulkWriteData)
{
  const char *log = NULL;
  bool result = false;

  for (uint8_t index=0; index<WriteCount; index++) 
  {
    result = dxlTls.addBulkWriteParam(bulkWriteID[index], bulkWriteAddress[index],
                                     bulkWriteLenght[index], bulkWriteData[index], &log);
    if (result == false)
    {
      ROS_ERROR("addBulkWriteParam() %s", log);
      return false;
    }
  }
  result = dxlTls.bulkWrite(&log);
  if (result == false) 
  {
    ROS_ERROR("bulkWrite() %s\n", log);
    return false;
  }
  return true;
}


bool SensorWrite(uint8_t WriteCount, uint8_t *WriteID,uint16_t *WriteAddress, 
                    uint16_t *WriteLenght, uint8_t WriteData[][CONTROL_DATA_SIZE_MAX])
{
  const char *log = NULL;
  bool result = false;

  for (uint8_t index=0; index<WriteCount; index++) 
  {
    result = dxlTls.writeRegister(WriteID[index], WriteAddress[index],
                                     WriteLenght[index], WriteData[index], &log);
    if (result == false)
    {
      ROS_ERROR("writeRegister() %s", log);
      return false;
    }
  }
  return true;
}

// bool ServoMovingGet() {
//   /* at least 1 servo is moving return true ,else return false */
//   int32_t movingGet[20] = {0};

//   ServoBulkRead(scannedId, dxlIdCnt, "Moving", &movingGet[0]);
//   for (uint8_t idNum = 0; idNum < dxlIdCnt; idNum++) {
//     if (movingGet[idNum]) {
// #ifdef DEBUG
//       ROS_INFO("Moving !!");
// #endif
//       return true;
//     }
//   }
//   return false;
// }

void RobotStand(uint8_t *idGet, uint8_t idCnt, int32_t velocity) {
  const char *log = NULL;
  uint8_t idArray[30] = {0};
  int32_t standPositionGoal[20];
  bodyhub::ServoPositionAngle servoPositionsMsg;

  for (uint8_t idNum = 0; idNum < idCnt; idNum++) {
    idArray[idNum] = idGet[idNum];
    standPositionGoal[idNum] = dxlTls.convertRadian2Value(
        idArray[idNum], Angle2Radian(standPositionBuff[idArray[idNum] - 1]));
    motoDataAngleStore[idNum] = standPositionBuff[idArray[idNum] - 1];
    motoDataValueStore[idNum] = standPositionGoal[idNum];
  }

  dxlTls.syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_POSITION, idArray, idCnt,
                   standPositionGoal, 1, &log);  //同步写指令

  servoPositionsMsg.angle = motoDataAngleStore;
  ServoPositionPub.publish(servoPositionsMsg);
}

void LoadSensorNameID(const std::string path) {
  uint8_t mmmm = 138;
  YAML::Node sensorNameIDDoc;
  try {
    sensorNameIDDoc = YAML::LoadFile(path.c_str());
  } catch (const std::exception &e) {
    ROS_WARN("Fail to load sensorNameID yaml.");
    return;
  }
  YAML::Node itemData = sensorNameIDDoc["sensorNameID"];
  if (itemData.size() == 0) return;

  for (YAML::const_iterator itItemNum = itemData.begin(); itItemNum != itemData.end(); itItemNum++) 
  {
    std::string sensorName = itItemNum->first.as<std::string>();
    int sensorID = itItemNum->second.as<int>();

    sensorNameIDMap[sensorName] = sensorID;
  }
}

void LoadOffset(const std::string path) {
  std::map<std::string, double> offsetMap;
  std::string stringID = "ID";
  std::string IDNameStr;

  YAML::Node offsetDoc;
  try {
    offsetDoc = YAML::LoadFile(path.c_str());
  } catch (const std::exception &e) {
    ROS_WARN("Fail to load offset yaml.");
    return;
  }

  YAML::Node itemData = offsetDoc["offset"];
  if (itemData.size() == 0) return;

  for (YAML::const_iterator itItemNum = itemData.begin();
       itItemNum != itemData.end(); itItemNum++) {
    std::string IDName = itItemNum->first.as<std::string>();
    double offsetValue = itItemNum->second.as<double>();

    offsetMap[IDName] = offsetValue;
  }
  for (uint8_t idNum = 1; idNum < 21; idNum++) {
    IDNameStr = stringID + std::to_string(idNum);
    if (offsetMap.find(IDNameStr) == offsetMap.end())
      ROS_WARN("Without find offset of %s ", IDNameStr.c_str());
    else
      offsetStorage[idNum - 1] = offsetMap[IDNameStr];
    printf("offsetStorage: %f \n", offsetStorage[idNum - 1]);
  }
}

void LoadDxlInitPose(const std::string path) {
  std::map<std::string, double> initPoseMap;
  std::string stringID = "ID";
  std::string IDNameStr;

  YAML::Node initPoseDoc;
  try {
    initPoseDoc = YAML::LoadFile(path.c_str());
  } catch (const std::exception &e) {
    ROS_WARN("Fail to load dxlinitPose yaml.");
    return;
  }

  YAML::Node itemData = initPoseDoc["InitPose"];
  if (itemData.size() == 0) return;

  for (YAML::const_iterator itItemNum = itemData.begin();
       itItemNum != itemData.end(); itItemNum++) {
    std::string IDName = itItemNum->first.as<std::string>();
    double initPoseValue = itItemNum->second.as<double>();

    initPoseMap[IDName] = initPoseValue;
  }
  for (uint8_t idNum = 1; idNum < 21; idNum++) {
    IDNameStr = stringID + std::to_string(idNum);
    if (initPoseMap.find(IDNameStr) == initPoseMap.end())
      ROS_WARN("Without find dxlinitPose of %s ", IDNameStr.c_str());
    else
      standPositionBuff[idNum - 1] = initPoseMap[IDNameStr];
    printf("standPositionBuff:%d %f \n", idNum, standPositionBuff[idNum - 1]);
  }
}

void MotoPositionCallback(const bodyhub::JointControlPoint::ConstPtr &msg) {
  uint8_t cnt = 0;
  uint8_t idNum = 0;
  uint8_t idCnt = 0;
  MotoPoint wp;
  std::vector<MotoPoint> goalVector;
  bodyhub::JointControlPoint jntCtrMsg;

  if (masterIDControl == msg->mainControlID) {
    idCnt = msg->positions.size();

    // msg --> wp --> goalVector
    for (idNum = 0; idNum < idCnt; idNum++) {
      wp.position = Angle2Radian(msg->positions.at(idNum));

      if (msg->velocities.size() != 0)
        wp.velocity = msg->velocities.at(idNum);
      else
        wp.velocity = 0.0f;

      if (msg->accelerations.size() != 0)
        wp.acceleration = msg->accelerations.at(idNum);
      else
        wp.acceleration = 0.0f;

      goalVector.push_back(wp);  // id number ++
    }

    // goalVector --> jntCtrMsg
    for (idNum = 0; idNum < idCnt; idNum++) {
      jntCtrMsg.positions.push_back(goalVector[idNum].position);
      jntCtrMsg.velocities.push_back(goalVector[idNum].velocity);
      jntCtrMsg.accelerations.push_back(goalVector[idNum].acceleration);
    }
    jntCtrMsg.mainControlID = msg->mainControlID;

    pthread_mutex_lock(&mtxMo);
    motoQueue.push(jntCtrMsg);
    pthread_mutex_unlock(&mtxMo);

    //数据到达
    if ((bodyhubState == StateEnum::ready) ||
        (bodyhubState == StateEnum::pause))
      UpdateState(StateEnum::running);

    goalVector.clear();
  }
}

void HeadPositionCallback(const bodyhub::JointControlPoint::ConstPtr &msg) {
  /* 仅头部舵机数据接收 0_ID19 1_ID20 */

  uint8_t idNum = 0;
  uint8_t idCnt = 0;
  MotoPoint wp;
  std::vector<MotoPoint> goalVector;
  bodyhub::JointControlPoint headCtrMsg;
  if (masterIDControl == msg->mainControlID) {
    idCnt = msg->positions.size();

    // msg --> wp
    for (idNum = 0; idNum < idCnt; idNum++) {
      wp.position = Angle2Radian(msg->positions.at(idNum));

      if (msg->velocities.size() != 0)
        wp.velocity = msg->velocities.at(idNum);
      else
        wp.velocity = 0.0f;

      if (msg->accelerations.size() != 0)
        wp.acceleration = msg->accelerations.at(idNum);
      else
        wp.acceleration = 0.0f;

      goalVector.push_back(wp);  // id number ++
    }

    // wp --> headCtrMsg
    for (idNum = 0; idNum < idCnt; idNum++) {
      headCtrMsg.positions.push_back(goalVector[idNum].position);
      headCtrMsg.velocities.push_back(goalVector[idNum].velocity);
      headCtrMsg.accelerations.push_back(goalVector[idNum].acceleration);
    }
    headCtrMsg.mainControlID = msg->mainControlID;

    pthread_mutex_lock(&mtxHe);
    headCtrlQueue.push(headCtrMsg);
    pthread_mutex_unlock(&mtxHe);

    //数据到达
    if ((bodyhubState == StateEnum::ready) ||
        (bodyhubState == StateEnum::pause))
      UpdateState(StateEnum::running);

    goalVector.clear();
  }
}

bool RegistSensorCallback(bodyhub::SrvInstWrite::Request &req,
                          bodyhub::SrvInstWrite::Response &res) {
  SensorModule *newSensor = new SensorModule;
  newSensor->sensorName = req.itemName;
  newSensor->startAddr = req.dxlID;
  newSensor->length = req.setData;
  addSensorModule(newSensor);

  res.complete = true;
  return true;
}
bool DeleteSensorCallback(bodyhub::SrvTLSstring::Request &req,
                          bodyhub::SrvTLSstring::Response &res) {
  for (auto s_it = sensorsList.begin(); s_it != sensorsList.end(); s_it++) {
    if ((*s_it)->sensorName == req.str) {
      removeSensorModlue(*s_it);
      delete *s_it;
#ifdef DEBUG
      for (const auto &sensorN : sensorsList)
        std::cout << sensorN->sensorName << std::endl;
#endif

      res.data = 1;
      return true;
    }
  }
  ROS_ERROR("Sensor Module Name [%s] isn't exist !", req.str.c_str());

  res.data = 0;
  return true;
}

bool InstReadSrvCallback(bodyhub::SrvInstRead::Request &req,
                         bodyhub::SrvInstRead::Response &res) {
  bool result = false;
  const char *log = NULL;
  std::string itemName;
  uint8_t dxlID = 0;
  int32_t readGetData = 0;

  //服务被请求
  if (bodyhubState == StateEnum::ready) UpdateState(StateEnum::directOperate);

  if (bodyhubState == StateEnum::directOperate) {
    dxlID = req.dxlID;
    itemName = req.itemName;
    result = dxlTls.readRegister(dxlID, itemName.c_str(), &readGetData, &log);
    if (result == false) {
      ROS_WARN("%s\n", log);
      res.getData = 998;
    } else if (itemName == "Present_Position")
      res.getData =
          Radian2Angle(dxlTls.convertValue2Radian(dxlID, readGetData));
    else
      res.getData = readGetData;

  } else {
    res.getData = 999;
    ROS_WARN("YOU ARE NOT IN directOperate");
  }

  return true;
}

bool InstWriteSrvCallback(bodyhub::SrvInstWrite::Request &req,
                          bodyhub::SrvInstWrite::Response &res) {
  const char *log = NULL;
  std::string itemName;
  uint8_t dxlID = 0;
  int32_t writeSetData = 0;

  //服务被请求
  if (bodyhubState == StateEnum::ready) UpdateState(StateEnum::directOperate);

  if (bodyhubState == StateEnum::directOperate) {
    dxlID = req.dxlID;
    itemName = req.itemName;
    if (itemName == "Goal_Position")
      writeSetData =
          dxlTls.convertRadian2Value(dxlID, Angle2Radian(req.setData));
    else
      writeSetData = req.setData;
    dxlTls.writeRegister(dxlID, itemName.c_str(), writeSetData, &log);
    res.complete = true;
  } else {
    res.complete = false;
    ROS_WARN("YOU ARE NOT IN directOperate");
  }

  return true;
}

bool SyncWriteSrvCallback(bodyhub::SrvSyncWrite::Request &req,
                          bodyhub::SrvSyncWrite::Response &res) {
  bool result = false;
  const char *log = NULL;
  uint8_t idArray[30] = {0};
  int32_t writeSetData[20];
  uint8_t idCnt = 0;
  uint8_t handleIndex = 0;

  //服务被请求
  if (bodyhubState == StateEnum::ready) UpdateState(StateEnum::directOperate);

  if (bodyhubState == StateEnum::directOperate) {
    idCnt = req.idCnt;

    if (req.itemName == "Goal_Position") {
      handleIndex = SYNC_WRITE_HANDLER_FOR_GOAL_POSITION;
      for (uint8_t idNum = 0; idNum < idCnt; idNum++) {
        idArray[idNum] = req.idArray[idNum];
        writeSetData[idNum] = dxlTls.convertRadian2Value(
            idArray[idNum], Angle2Radian(req.setData[idNum]));
      }
    } else if (req.itemName == "Moving_Speed") {
      handleIndex = SYNC_WRITE_HANDLER_FOR_GOAL_VELOCITY;
      for (uint8_t idNum = 0; idNum < idCnt; idNum++) {
        idArray[idNum] = req.idArray[idNum];
        writeSetData[idNum] = req.setData[idNum];
      }
    } else {
      res.complete = result;
      return true;
    }

    result = dxlTls.syncWrite(handleIndex, idArray, idCnt, writeSetData, 1,
                              &log);  //同步写指令
  } else
    ROS_WARN("YOU ARE NOT IN directOperate");

  res.complete = result;
  return true;
}

bool SetServoLockStateCallback(bodyhub::SrvServoWrite::Request &req,
                               bodyhub::SrvServoWrite::Response &res) {
  bool result = false;
  const char *log = NULL;
  uint8_t dxlID = 0;
  int32_t writeSetData = 0;

  //服务被请求
  if (bodyhubState == StateEnum::ready) UpdateState(StateEnum::directOperate);

  if (bodyhubState == StateEnum::directOperate) {
    dxlID = req.dxlID;
    writeSetData = (int32_t)req.setData;
    if (writeSetData == 0)
      dxlTls.torqueOff(dxlID, &log);
    else
      dxlTls.torqueOn(dxlID, &log);
    res.complete = true;
  } else {
    res.complete = false;
    ROS_WARN("YOU ARE NOT IN directOperate");
  }

  return true;
}
bool SetServoLockStateAllCallback(bodyhub::SrvServoAllWrite::Request &req,
                                  bodyhub::SrvServoAllWrite::Response &res) {
  bool result = false;
  const char *log = NULL;
  uint8_t idArray[30] = {0};
  int32_t writeSetData[20];
  uint8_t idCnt = 0;

  //服务被请求
  if (bodyhubState == StateEnum::ready) UpdateState(StateEnum::directOperate);

  if (bodyhubState == StateEnum::directOperate) {
    idCnt = req.idCnt;
    for (uint8_t idNum = 0; idNum < idCnt; idNum++) {
      idArray[idNum] = req.idArray[idNum];       // test
      writeSetData[idNum] = req.setData[idNum];  // test
      if (writeSetData[idNum] == 0)
        dxlTls.writeRegister(idArray[idNum], "Torque_Enable", 0, &log);
      else if (writeSetData[idNum] == 1)
        dxlTls.writeRegister(idArray[idNum], "Torque_Enable", 1, &log);
    }
    res.complete = true;
  } else {
    res.complete = false;
    ROS_WARN("YOU ARE NOT IN directOperate");
  }

  return true;
}
bool GetServoLockStateAllCallback(bodyhub::SrvServoAllRead::Request &req,
                                  bodyhub::SrvServoAllRead::Response &res) {
  uint8_t idArray[30] = {0};
  uint8_t idCnt = 0;
  int32_t readGetData[20] = {0};

  //服务被请求
  if (bodyhubState == StateEnum::ready) UpdateState(StateEnum::directOperate);

  if (bodyhubState == StateEnum::directOperate) {
    idCnt = req.idCnt;
    for (uint8_t idNum = 0; idNum < idCnt; idNum++) {
      idArray[idNum] = req.idArray[idNum];
    }
    ServoBulkRead(idArray, idCnt, "Torque_Enable", &readGetData[0]);
    for (uint8_t idNum = 0; idNum < idCnt; idNum++)
      res.getData.push_back(readGetData[idNum]);
  } else {
    res.getData.push_back(0);
    ROS_WARN("YOU ARE NOT IN directOperate");
  }

  return true;
}
bool SetServoTarPositionCallback(bodyhub::SrvServoWrite::Request &req,
                                 bodyhub::SrvServoWrite::Response &res) {
  bool result = false;
  const char *log = NULL;
  uint8_t dxlID = 0;
  int32_t writeSetData = 0;

  //服务被请求
  if (bodyhubState == StateEnum::ready) UpdateState(StateEnum::directOperate);

  if (bodyhubState == StateEnum::directOperate) {
    dxlID = req.dxlID;
    writeSetData = dxlTls.convertRadian2Value(dxlID, Angle2Radian(req.setData));
    dxlTls.writeRegister(dxlID, "Goal_Position", writeSetData, &log);
    res.complete = true;
  } else {
    res.complete = false;
    ROS_WARN("YOU ARE NOT IN directOperate");
  }

  return true;
}
bool SetServoTarPositionAllCallback(bodyhub::SrvServoAllWrite::Request &req,
                                    bodyhub::SrvServoAllWrite::Response &res) {
  bool result = false;
  const char *log = NULL;
  uint8_t idArray[30] = {0};
  int32_t writeSetData[20];
  uint8_t idCnt = 0;

  //服务被请求
  if (bodyhubState == StateEnum::ready) UpdateState(StateEnum::directOperate);

  if (bodyhubState == StateEnum::directOperate) {
    idCnt = req.idCnt;
    for (uint8_t idNum = 0; idNum < idCnt; idNum++) {
      idArray[idNum] = req.idArray[idNum];
      writeSetData[idNum] = dxlTls.convertRadian2Value(
          req.idArray[idNum], Angle2Radian(req.setData[idNum]));
    }
    result = dxlTls.syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_POSITION, idArray,
                              idCnt, writeSetData, 1, &log);  //同步写指令
    res.complete = true;
  } else {
    res.complete = false;
    ROS_WARN("YOU ARE NOT IN directOperate");
  }

  return true;
}
bool GetServoPositionAllCallback(bodyhub::SrvServoAllRead::Request &req,
                                 bodyhub::SrvServoAllRead::Response &res) {
  uint8_t idArray[30] = {0};
  uint8_t idCnt = 0;
  int32_t readGetData[20] = {0};

  //服务被请求
  if (bodyhubState == StateEnum::ready) UpdateState(StateEnum::directOperate);

  if (bodyhubState == StateEnum::directOperate) {
    idCnt = req.idCnt;
    for (uint8_t idNum = 0; idNum < idCnt; idNum++) {
      idArray[idNum] = req.idArray[idNum];
    }
    ServoBulkRead(idArray, idCnt, "Present_Position", &readGetData[0]);
    for (uint8_t idNum = 0; idNum < idCnt; idNum++)
      res.getData.push_back(Radian2Angle(
          dxlTls.convertValue2Radian(req.idArray[idNum], readGetData[idNum])));
  } else {
    res.getData.push_back(0);
    ROS_WARN("YOU ARE NOT IN directOperate");
  }

  return true;
}

bool StateSrvCallback(bodyhub::SrvState::Request &req,
                      bodyhub::SrvState::Response &res) {
  if ((req.masterID == masterIDControl) || (masterIDControl == 0)) {
    if (req.stateReq == "setStatus") {
      masterIDControl = req.masterID;
      if (bodyhubState == StateEnum::preReady) UpdateState(StateEnum::ready);
    } else if (req.stateReq == "resetStatus") {
    } else if (req.stateReq == "break") {
    } else if (req.stateReq == "stop") {
      if ((bodyhubState == StateEnum::running) ||
          (bodyhubState == StateEnum::pause))
        UpdateState(StateEnum::stoping);
    } else if (req.stateReq == "reset") {
      if (bodyhubState != StateEnum::stoping) UpdateState(StateEnum::preReady);
    }
    res.stateRes = bodyhubState;
  } else
    res.stateRes = masterIDControl;
  return true;
}

bool GetStatusCallback(bodyhub::SrvString::Request &req,
                       bodyhub::SrvString::Response &res) {
  if (req.str != "") res.data = stateNewStr;
  return true;
}

bool GetJointAngleCallback(bodyhub::SrvServoAllRead::Request &req,
                           bodyhub::SrvServoAllRead::Response &res) {
  res.getData = motoDataAngleStore;
  return true;
}

bool MasterIDSrvCallback(bodyhub::SrvTLSstring::Request &req,
                         bodyhub::SrvTLSstringResponse &res) {
#ifdef DEBUG
  ROS_INFO("MasterIDSrvCallback get masterID %s", req.str.c_str());
#endif
  res.data = masterIDControl;
  return true;
}

void QueueThread() {
  ros::NodeHandle n;
  ros::CallbackQueue topicQueue;

  n.setCallbackQueue(&topicQueue);

  ros::Subscriber MotoPositionSub =
      n.subscribe("MediumSize/BodyHub/MotoPosition", 1, MotoPositionCallback);
  ros::Subscriber HeadPositionSub =
      n.subscribe("MediumSize/BodyHub/HeadPosition", 1, HeadPositionCallback);

  while (n.ok()) topicQueue.callAvailable(ros::WallDuration(0.01));
}

bool initSDKHandlers(void) {
  bool result = false;
  const char *log = NULL;

  result = dxlTls.addSyncWriteHandler(scannedId[0], "Goal_Position", &log);
  if (result == false) {
    ROS_ERROR("%s", log);
    return result;
  }

  result = dxlTls.addSyncWriteHandler(scannedId[0], "Moving_Speed",
                                      &log);  //第一个舵机号要为存在舵机
  if (result == false) {
    ROS_ERROR("%s", log);
    return result;
  }

  result = dxlTls.initBulkRead(&log);
  if (result == false) {
    ROS_ERROR("%s", log);
    return result;
  }

  result = dxlTls.initBulkWrite(&log);
  if (result == false) 
  {
    ROS_ERROR("%s", log);
    return result;
  }

  return result;
}

void UpdateState(uint8_t stateNew) {
  uint8_t readCount = 20;
  int32_t readGetData = 0;
  bodyhub::ServoPositionAngle servoPositionsMsg;
  const char *log = NULL;
  bool result = false;

  if ((stateNew == StateEnum::preReady) &&
      (bodyhubState == StateEnum::directOperate)) {
    for (uint8_t idNum = 0; idNum < readCount; idNum++) {
      result = dxlTls.readRegister(idNum + 1, "Present_Position", &readGetData,
                                   &log);
      if (result == false) {
        ROS_WARN("%s\n", log);
        motoDataValueStore[idNum] = -1;   // dataerror
        motoDataAngleStore[idNum] = 999;  // dataerror
      } else {
        motoDataAngleStore[idNum] =
            Radian2Angle(dxlTls.convertValue2Radian(idNum, readGetData));
        motoDataValueStore[idNum] = readGetData;
      }
    }
    servoPositionsMsg.angle = motoDataAngleStore;
    ServoPositionPub.publish(servoPositionsMsg);
#ifdef DEBUG
    for (uint8_t idNum = 0; idNum < readCount; idNum++)
      ROS_INFO("directOperate exit Servo Present_Position ID%d: %f", idNum,
               motoDataValueStore[idNum]);
#endif
  }
  if (stateNew == StateEnum::preReady) {
    RobotStand(scannedId, dxlIdCnt, 1000);
    sensorDataMap.clear();
    sensorsList.clear();
    ClearTimerQueue();
  }

  bodyhubState = stateNew;
  budyhubStateMsg.data = stateNew;
  StatusPub.publish(budyhubStateMsg);

  switch (stateNew) {
    case StateEnum::init:
      stateNewStr = "init";
      break;
    case StateEnum::preReady:
      stateNewStr = "preReady";
      break;
    case StateEnum::ready:
      stateNewStr = "ready";
      break;
    case StateEnum::running:
      stateNewStr = "running";
      break;
    case StateEnum::pause:
      stateNewStr = "pause";
      break;
    case StateEnum::stoping:
      stateNewStr = "stoping";
      break;
    case StateEnum::error:
      stateNewStr = "error";
      break;
    case StateEnum::directOperate:
      stateNewStr = "directOperate";
      break;
    default:
      break;
  }
  ROS_INFO("The new bodyhubState: %s--%d ", stateNewStr.c_str(), stateNew);
}

void ClearTimerQueue() {
  if ((!motoQueue.empty()) || (!headCtrlQueue.empty())) {
    ClearQueue(motoQueue);
    ClearQueue(headCtrlQueue);
  }
}

void STATEinit(ros::NodeHandle nh) {
  StatusPub = nh.advertise<std_msgs::UInt16>("MediumSize/BodyHub/Status", 0);
  ServoPositionPub = nh.advertise<bodyhub::ServoPositionAngle>(
      "MediumSize/BodyHub/ServoPositions", 0);
  SensorRawDataPub =
      nh.advertise<bodyhub::SensorRawData>("MediumSize/BodyHub/SensorRaw", 0);

  StateService =
      nh.advertiseService("MediumSize/BodyHub/StateJump", StateSrvCallback);
  GetStatusService =
      nh.advertiseService("MediumSize/BodyHub/GetStatus", GetStatusCallback);
  MasterIDService = nh.advertiseService("MediumSize/BodyHub/GetMasterID",
                                        MasterIDSrvCallback);
  GetJointAngleService = nh.advertiseService("MediumSize/BodyHub/GetJointAngle",
                                             GetJointAngleCallback);

  InstReadService = nh.advertiseService(
      "MediumSize/BodyHub/DirectMethod/InstRead", InstReadSrvCallback);
  InstWriteService = nh.advertiseService(
      "MediumSize/BodyHub/DirectMethod/InstWrite", InstWriteSrvCallback);
  SyncWriteService = nh.advertiseService(
      "MediumSize/BodyHub/DirectMethod/SyncWrite", SyncWriteSrvCallback);

  SetLockStateService =
      nh.advertiseService("MediumSize/BodyHub/DirectMethod/SetServoLockState",
                          SetServoLockStateCallback);
  SetLockStateAllService = nh.advertiseService(
      "MediumSize/BodyHub/DirectMethod/SetServoLockStateAll",
      SetServoLockStateAllCallback);
  GetLockStateAllService = nh.advertiseService(
      "MediumSize/BodyHub/DirectMethod/GetServoLockStateAll",
      GetServoLockStateAllCallback);
  SetTarPositionService =
      nh.advertiseService("MediumSize/BodyHub/DirectMethod/SetServoTarPosition",
                          SetServoTarPositionCallback);
  SetTarPositionAllService = nh.advertiseService(
      "MediumSize/BodyHub/DirectMethod/SetServoTarPositionAll",
      SetServoTarPositionAllCallback);
  GetPositionAllService =
      nh.advertiseService("MediumSize/BodyHub/DirectMethod/GetServoPositionAll",
                          GetServoPositionAllCallback);
  RegistSensorService = nh.advertiseService("MediumSize/BodyHub/RegistSensor",
                                            RegistSensorCallback);
  DeleteSensorService = nh.advertiseService("MediumSize/BodyHub/DeleteSensor",
                                            DeleteSensorCallback);

  UpdateState(StateEnum::init);

  const char *portName = "/dev/ttyUSB0";
  int baudrate = 1000000;

  const char *log = NULL;
  bool result = false;

  result = dxlTls.init(portName, baudrate, &log);  // initWorkbench
  if (result == false) {
    ROS_ERROR("%s\n", log);
    return;
  } else
    ROS_INFO("Succeed to init(%d)\n", baudrate);
  dxlTls.scan(scannedId, &dxlIdCnt, scanRange, &log);
  ROS_INFO("scaned ID num: %d \n", dxlIdCnt);
  // load offset
  if (offsetFile != "") LoadOffset(offsetFile);
  // load dxlinitpose
  if (dxlInitPoseFile != "") LoadDxlInitPose(dxlInitPoseFile);
  // load sensorNameID
  if (sensorNameIDFile != "") LoadSensorNameID(sensorNameIDFile);

  result = initSDKHandlers();

  pthread_mutex_init(&mtxMo, NULL);
  pthread_mutex_init(&mtxHe, NULL);
  pthread_mutex_init(&mtxSL, NULL);

  //更新下一个状态
  UpdateState(StateEnum::preReady);
}

void STATEpreReady() { masterIDControl = 0; }

void STATEready() { const char *log = NULL; }
void STATErunning() {
  // check empty & Moving
  if ((motoQueue.empty()) && (headCtrlQueue.empty()))
    UpdateState(StateEnum::pause);  //更新下一个状态
}
void STATEpause() { const char *log = NULL; }
void STATEstoping() {
  ClearTimerQueue();
  RobotStand(scannedId, dxlIdCnt, 1000);
  //更新下一个状态
  UpdateState(StateEnum::ready);
}
void STATEerror() { const char *log = NULL; }

void STATEdirectOperate() { const char *log = NULL; }


bool SensorControlCallback( bodyhub::SrvSensorControl::Request &req,
                            bodyhub::SrvSensorControl::Response &res)
{
  if(sensorNameIDMap.empty())
  {
    res.Result = "The list of names is empty!";
    return false;
  }
  if(sensorNameIDMap.count(req.SensorName) == 0)
  {
    res.Result = "Name of not found!";
    return false;
  }
  SensorControl_t ControlParamete;
  ControlParamete.id = sensorNameIDMap.at(req.SensorName);
  ControlParamete.addr = req.SetAddr;
  ControlParamete.lenght = sizeof(req.Paramete)/sizeof(req.Paramete[0]);
  if(ControlParamete.lenght > (CONTROL_DATA_SIZE_MAX-1))
  {
    res.Result = "Too many parameters!";
    return false;
  }
  for(uint8_t i=0;i<ControlParamete.lenght;i++)
    ControlParamete.data[i] = req.Paramete[i];

  pthread_mutex_lock(&MtuexSensorControl);
  SensorControlQueue.push(ControlParamete);
  pthread_mutex_unlock(&MtuexSensorControl);

  res.Result = "Operate successfully";
  return true;
}

void SensorControlInit(ros::NodeHandle node)
{
  if (sensorNameIDFile != "")
    LoadSensorNameID(sensorNameIDFile);
  pthread_mutex_init(&MtuexSensorControl, NULL);
  SensorControlService = node.advertiseService("/BodyHub/SensorControl", SensorControlCallback);
  ROS_INFO("Create service of /BodyHub/SensorControl");
}


int main(int argc, char **argv) {
  //初始化节点
  ros::init(argc, argv, "BodyHubNode");
  ros::NodeHandle nodeHandle("");

  /* Load ROS Parameter */
  nodeHandle.param<std::string>("poseOffsetPath", offsetFile, "");
  nodeHandle.param<std::string>("poseInitPath", dxlInitPoseFile, "");
  nodeHandle.param<std::string>("sensorNameIDPath", sensorNameIDFile, "");
  
  SensorControlInit(nodeHandle);

  boost::thread communicateThread(QueueThread);
  boost::thread timerThread(threadTimer);
  STATEinit(nodeHandle);

  ros::Rate loop_rate(10);
  while (ros::ok()) {
    if (bodyhubState == StateEnum::preReady)
      STATEpreReady();
    else if (bodyhubState == StateEnum::ready)
      STATEready();
    else if (bodyhubState == StateEnum::running)
      STATErunning();
    else if (bodyhubState == StateEnum::pause)
      STATEpause();
    else if (bodyhubState == StateEnum::stoping)
      STATEstoping();
    else if (bodyhubState == StateEnum::error)
      STATEerror();
    else if (bodyhubState == StateEnum::directOperate)
      STATEdirectOperate();

    ros::spinOnce();
    loop_rate.sleep();
  }

  communicateThread.join();
  return 0;
}
