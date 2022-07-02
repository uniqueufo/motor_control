/**
* @file        RMDL_RS485_cfg.h
* @author      Nan Lin
* @contact     fhln@mail.ustc.edu.cn
* @date        22-6-2
* @description 

* Copyright (c) 2022 Nan Lin
*/

#ifndef OUR_HW_SDK_SRC_HAL_USER_CFG_MOTOR_CFG_PAN_TILT_MOTOR_TYPES_H_
#define OUR_HW_SDK_SRC_HAL_USER_CFG_MOTOR_CFG_PAN_TILT_MOTOR_TYPES_H_

#include "RMDL_RS485_def.h"

//struct PanTiltMotorStateFeedback {
////  int32_t accel;      //加速度，单位1dps/s
//  uint8_t  node_id;         //设备总线id
//  uint16_t circleAngle;    //单圈角度值，以编码器零点为起始点，顺时针增加，再到达零点时数值回0，单位0.01度/LSB。范围0~35999
//  uint8_t  errorState;     //错误标志，[bit 0] 电压状态，0代表正常，1代表低压; [bit 3]温度状态，0代表温度正常，1代表过温保护
//  int16_t  iq;             //转矩电流值，-2048~2048,对应-33A~33A
//  int16_t  speed;          //电机转速, 1dps/LSB
//  PID_cfg  pid_cfg;
//};
struct PanTiltMotorStateFeedback{
  int8_t temperature; //电机温度
  double position; // 当前位置，单位rad， 为0~2 pi
  double speed;    // 当前速度，单位rad/s
  double torque;   // 当前力矩，单位Nm
  double torque_current; //当前转矩电流,单位A

};
struct PanTiltMotorConfig {
  double offset;   //角度偏移， 单位rad
  PID_cfg  pid_cfg; //三环PI参数
  double max_speed; //最大速度，单位rad/s
  double max_aspeed; //最大加速度
  double max_angle; //最大角度
  double min_angle; //最小角度
  uint8_t coefficient;
};

struct PanTiltMotorControlCmd {
  //  CirclePositionControl position_control_frame;
  double speed_;
  double circle_pose_;
  double max_speed_limit_;
};

/// \brief for serial receving data processing
typedef enum {
  WAIT_FOR_HEADER_ID = 0, // 帧头字节
  WAIT_FOR_CMD, // 命令字节
  WAIT_FOR_NODE_ID, // 设备ID字节
  WAIT_FOR_DATA_LEN, //数据长度字节
  WAIT_FOR_HEADER_CHECKSUM, //帧头校验字节
  WAIT_FOR_SPECIFIC_DATA, // 帧数据(当数据长度不为零时才存在)
  WAIT_FOR_DATA_CHECKSUM // 数据校验字节(当数据长度不为零时才存在)
}                        PanTiltSerialDecodeState;

//#define SERIAL_FRAME_HEADER_ID ((uint8_t)0x3E) //frame id


/// \brief 用于串口逐字节解析时临时数据存贮
//static struct {
//  uint8_t frame_id;
//  uint8_t cmd;
//  uint8_t motor_id;
//  uint8_t frame_data_len; //帧数据长度
//  uint8_t header_check_sum;
//
//  bool has_data_field;
//  uint8_t frame_data[FRAME_MAX_DATA_LEN];
//  uint8_t data_check_sum;
//}uart_parsing_data;

RS485_FRAME_MACRO(FRAME_MAX_DATA_LEN, TEMP_PARSING_DATA, SERIAL_FRAME_HEADER_ID, 0x00)
//static TEMP_PARSING_DATA uart_parsing_data(0x00);

// statisctics
typedef struct {
  uint32_t frame_parsed;
  uint32_t frame_with_wrong_checksum;
} UARTParsingStats;

static UARTParsingStats uart_parsing_stats = {.frame_parsed = 0, .frame_with_wrong_checksum = 0};

template<class T1>
void SERIAL_DEBUG_PRINT(std::string pre_info, T1 content) {
  if (std::is_same<T1, uint8_t>::value) {
    std::cout << pre_info << std::hex << static_cast<int>(content) << std::dec << std::endl;
  } else if (std::is_same<T1, std::string>::value) {
    std::cout << std::string(pre_info) << content << std::endl;
  } else {}
}

struct DeviceConfiguration {
  bool runBase;
  bool runArm;
  bool runPanTilt;
  bool runStopButton;

  std::string PanTiltComName;
  std::string StopButtonComName;
  BaudRate Baudrate;

  double baseLoopRate;
  double armLoopRate;
  double panTiltLoopRate;
  double stopButtonLoopRate;

  // for wheels
  struct 
  {
    double maxLinearSpeed;
    double maxTurnSpeed;
    double wheelDist;
    double wheelPerimeter;
  } wheelConfiguration;

  struct
  {
    double elevatorUnitHeight;
    double minElevatorHeight;
    double maxElevatorHeight;
  } elevator;

  struct 
  {
    uint8_t stopChar;
    uint8_t normalChar;
  } stopButton;
};


//bool DecodeMsgFromUART(uint8_t c, PanTiltMotorStateFeedback *msg);
//bool ExtractStateMessage(PanTiltMotorStateFeedback *state_msg);
//bool ParseChar(uint8_t c, PanTiltMotorStateFeedback *msg);

#endif //OUR_HW_SDK_SRC_HAL_USER_CFG_MOTOR_CFG_PAN_TILT_MOTOR_TYPES_H_
