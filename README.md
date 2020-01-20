# bodyhub Package

## 1.概述
bodyhub节点是上位机其他节点与下位机通信的中间节点，机器人的所有控制指令和数据获取都通过此节点实现。  
BodyHub实现了一个状态机，用BodyHub统一管理下位机可确保如舵机这类设备在同一时刻只有一个上层节点控制，避免节点之间的干扰导致控制异常。
## 2.如何运行
在终端运行以下命令，回车后输入用户的密码确认，修改USB设备权限
```
sudo chmod 666 /dev/ttyUSB0
```
在终端运行以下命令，设置串口参数
```
setserial /dev/ttyUSB0 low_latency
```
在终端运行以下命令，启动节点
```
roslaunch bodyhub bodyhub.launch
```
## 3.节点API
### 3.1.话题
#### 3.1.1.Subscribed Topics
**机器人运动部分**

无

**外接传感器部分**

|名称|类型|说明|
|:-:|:-:|:-:|
|/BodyHub/SensorControl|SensorControl|执行器的控制消息|
- /BodyHub/SensorControl  
  订阅其他节点发送的外接执行器控制消息，消息类型定义
  ```
  string SensorName
  uint16 SetAddr
  uint8[] ParamList
  ```
  分别表示执行器名称；执行器参数地址；执行器参数列表。

**仿真控制部分**

|名称|类型|说明|
|:-:|:-:|:-:|
|/simulationStepDone|[std_msgs/Bool][std_msgs/Bool]|-|
|/simulationState|[std_msgs/Int32][std_msgs/Int32]|-|
#### 3.1.2.Published Topics
**机器人运动部分**

|名称|类型|说明|
|:-:|:-:|:-:|
|/MediumSize/BodyHub/Status|[std_msgs/UInt16][std_msgs/UInt16]|节点状态机的状态变化时，发布新状态|
|/MediumSize/BodyHub/ServoPositions|ServoPositionAngle|机器人舵机角度改变时，发布舵机新角度|
|/jointPosTarget|[std_msgs/Float64MultiArray][std_msgs/Float64MultiArray]|-|
|/jointPosMeasure|[std_msgs/Float64MultiArray][std_msgs/Float64MultiArray]|-|
|/jointVelTarget|[std_msgs/Float64MultiArray][std_msgs/Float64MultiArray]|-|
|/jointVelMeasure|[std_msgs/Float64MultiArray][std_msgs/Float64MultiArray]|-|
|/cpref|[std_msgs/Float64][std_msgs/Float64]|-|
|/cpC|[std_msgs/Float64][std_msgs/Float64]|-|
|/copm|[std_msgs/Float64][std_msgs/Float64]|-|
|/copD|[std_msgs/Float64][std_msgs/Float64]|-|
|/copref|[std_msgs/Float64][std_msgs/Float64]|-|
|/comm|[std_msgs/Float64][std_msgs/Float64]|-|
|/comD|[std_msgs/Float64][std_msgs/Float64]|-|
|/comref|[std_msgs/Float64][std_msgs/Float64]|-|
|/comEsti|[std_msgs/Float64][std_msgs/Float64]|-|
|/comvm|[std_msgs/Float64][std_msgs/Float64]|-|
|/comvD|[std_msgs/Float64][std_msgs/Float64]|-|
|/comvref|[std_msgs/Float64][std_msgs/Float64]|-|
|/comvEsti|[std_msgs/Float64][std_msgs/Float64]|-|
|/LFootZ|[std_msgs/Float64][std_msgs/Float64]|-|
|/RFootZ|[std_msgs/Float64][std_msgs/Float64]|-|
|/footDisRef|[std_msgs/Float64][std_msgs/Float64]|-|
|/footDis|[std_msgs/Float64][std_msgs/Float64]|-|
|/contactState|[std_msgs/Float64][std_msgs/Float64]|-|
|/stepPhase|[std_msgs/Float64][std_msgs/Float64]|-|
- /MediumSize/BodyHub/Status  
  节点状态变化时，通过此话题发布新状态，消息定义  
  [std_msgs/UInt16][std_msgs/UInt16]  
  节点状态定义
  ```
  namespace StateEnum
  {
  enum StateStyle
  {
    init = 20,
    preReady,
    ready,
    running,
    pause,
    stoping,
    error,
    directOperate,
    walking
  };
  }
  ```
- /MediumSize/BodyHub/ServoPositions  
  机器人舵机角度改变时，发布舵机新的角度值，消息定义
  ```
  float64[] angle
  ```

**外接传感器部分**

|名称|类型|说明|
|:-:|:-:|:-:|
|/MediumSize/BodyHub/SensorRaw|SensorRawData|传感器原始数据|
- /MediumSize/BodyHub/SensorRaw  
  获取外接传感器的原始数据，消息定义
  ```
  uint8[]   sensorReadID
  uint16[]  sensorStartAddress
  uint16[]  sensorReadLength
  int32[]   sensorData
  uint8     sensorCount
  uint8     dataLength
  ```

**仿真控制部分**

|名称|类型|说明|
|:-:|:-:|:-:|
|/joint_command|[std_msgs/Float64MultiArray][std_msgs/Float64MultiArray]|-|
|/startSimulation|[std_msgs/Bool][std_msgs/Bool]|-|
|/stopSimulation|[std_msgs/Bool][std_msgs/Bool]|-|
|/pauseSimulation|[std_msgs/Bool][std_msgs/Bool]|-|
|/enableSyncMode|[std_msgs/Bool][std_msgs/Bool]|-|
|/triggerNextStep|[std_msgs/Bool][std_msgs/Bool]|-|
### 3.2.服务
**机器人运动部分**

|名称|类型|说明|
|:-:|:-:|:-:|
|/MediumSize/BodyHub/StateJump|SrvState|状态机状态跳转接口|
|/MediumSize/BodyHub/GetStatus|SrvString|状态机状态获取接口|
|/MediumSize/BodyHub/GetMasterID|SrvTLSstring|获取占用当前节点的ID值|
|/MediumSize/BodyHub/GetJointAngle|SrvServoAllRead|-|
|/MediumSize/BodyHub/DirectMethod/InstReadVal|SrvInstRead|-|
|/MediumSize/BodyHub/DirectMethod/InstWriteVal|SrvInstWrite|-|
|/MediumSize/BodyHub/DirectMethod/SyncWriteVal|SrvSyncWrite|-|
|/MediumSize/BodyHub/DirectMethod/SetServoTarPositionVal|SrvServoWrite|-|
|/MediumSize/BodyHub/DirectMethod/SetServoTarPositionValAll|SrvServoAllWrite|-|
|/MediumSize/BodyHub/DirectMethod/GetServoPositionValAll|SrvServoAllRead|-|
|/MediumSize/BodyHub/DirectMethod/InstRead|SrvInstRead|-|
|/MediumSize/BodyHub/DirectMethod/InstWrite|SrvInstWrite|-|
|/MediumSize/BodyHub/DirectMethod/SyncWrite|SrvSyncWrite|-|
|/MediumSize/BodyHub/DirectMethod/SetServoLockState|SrvServoWrite|-|
|/MediumSize/BodyHub/DirectMethod/SetServoLockStateAll|SrvServoAllWrite|-|
|/MediumSize/BodyHub/DirectMethod/GetServoLockStateAll|SrvServoAllRead|-|
|/MediumSize/BodyHub/DirectMethod/SetServoTarPosition|SrvServoWrite|-|
|/MediumSize/BodyHub/DirectMethod/SetServoTarPositionAll|SrvServoAllWrite|-|
|/MediumSize/BodyHub/DirectMethod/GetServoPositionAll|SrvServoAllRead|-|
- /MediumSize/BodyHub/StateJump  
  状态机状态跳转接口，服务定义
  ```
  uint8 masterID
  string stateReq
  ---
  int16 stateRes
  ```
  分别代表占用ID；状态跳转控制字符；操作返回的状态值。
- /MediumSize/BodyHub/GetStatus  
  状态机状态获取接口，服务定义
  ```
  string str
  ---
  string data
  ```
  分别代表请求字符串，可任意设置，不能为空；返回获取的状态字符串。
- /MediumSize/BodyHub/GetMasterID  
  获取占用当前节点的ID值，服务定义
  ```
  string str
  ---
  uint8 data
  ```
  分别代表请求字符串，可任意设置；返回获取的ID值。

**外接传感器部分**

|名称|类型|说明|
|:-:|:-:|:-:|
|/MediumSize/BodyHub/RegistSensor|SrvInstWrite|注册需要获取数据的传感器|
|/MediumSize/BodyHub/DeleteSensor|SrvTLSstring|删除注册的传感器|
- /MediumSize/BodyHub/RegistSensor  
  注册需要获取数据的外接传感器，服务定义
  ```
  string itemName
  uint8 dxlID
  float64 setData
  ---
  bool complete
  ```
  分别代表注册传感器的名称；要获取传感器数据的寄存器地址；要获取传感器数据的地址长度；返回注册的结果。
- /MediumSize/BodyHub/DeleteSensor  
  删除注册的传感器，服务定义
  ```
  string str
  ---
  uint8 data
  ```
  分别代表要删除的传感器名称；返回删除的结果。

**仿真控制部分**

无
### 3.3.参数
|名称|类型|默认值|说明|
|:-:|:-:|:-:|:-:|
|poseOffsetPath|string|$(find bodyhub)/config/offset.yaml|机器人零点文件路径|
|poseInitPath|string|$(find bodyhub)/config/dxlInitPose.yaml|机器人初始动作配置文件路径|
|sensorNameIDPath|string|$(find bodyhub)/config/sensorNameID.yaml|传感器名称与ID配置文件路径|
|simenable|bool|false|节点运行仿真控制的开关|
## 附录
[std_msgs/Bool]: http://docs.ros.org/api/std_msgs/html/msg/Bool.html
[std_msgs/Int32]: http://docs.ros.org/api/std_msgs/html/msg/Int32.html
[std_msgs/UInt16]: http://docs.ros.org/api/std_msgs/html/msg/UInt16.html
[std_msgs/Float64]: http://docs.ros.org/api/std_msgs/html/msg/Float64.html
[std_msgs/Float64MultiArray]: http://docs.ros.org/api/std_msgs/html/msg/Float64MultiArray.html